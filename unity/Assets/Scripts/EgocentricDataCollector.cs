#if !PICO_OPENXR_SDK
using System;
using System.Collections;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading;
using Unity.XR.PICO.TOBSupport;
using Unity.XR.PXR;
using UnityEngine;
using UnityEngine.Android;
using UnityEngine.UI;
using UnityEngine.XR;
using TOBPose = Unity.XR.PICO.TOBSupport.Pose;

public class EgocentricDataCollector : MonoBehaviour
{
    [Header("Camera Settings")]
    [Tooltip("Per-eye resolution width")]
    [SerializeField] private int imageWidth = 1280;
    [Tooltip("Per-eye resolution height")]
    [SerializeField] private int imageHeight = 960;
    [SerializeField] [Range(50, 100)] private int jpegQuality = 85;
    [SerializeField] private bool enableMCTF = false;
    [SerializeField] private bool outputRawData = true;
    [SerializeField] private bool rotateImage180BeforeEncode = true;
    [SerializeField] private bool flipImageHorizontallyBeforeEncode = false;
    [SerializeField] private PXRCaptureRenderMode renderMode = PXRCaptureRenderMode.PXRCapture_RenderMode_LEFT;

    [Header("Recording Control")]
    [SerializeField] private bool autoStart = false;
    [SerializeField] private int maxQueueSize = 90;

    [Header("Runtime Status")]
    [SerializeField] private bool isCameraReady;
    [SerializeField] private bool isRecording;
    [SerializeField] private int recordedFrames;
    [SerializeField] private float currentFps;
    [SerializeField] private string currentSessionPath;

    private byte[] imgBuffer;
    private GCHandle imgBufferHandle;
    private Texture2D texture;
    private RGBCameraParamsNew cameraParams;
    private string sessionPath;
    private string framesPath;

    private ConcurrentQueue<RawFrame> encodeQueue = new ConcurrentQueue<RawFrame>();
    private ConcurrentQueue<FrameRecord> writeQueue = new ConcurrentQueue<FrameRecord>();
    private Thread writerThread;
    private volatile bool writerRunning;
    private StreamWriter posesWriter;
    private readonly object posesWriterLock = new object();

    private ConcurrentBag<byte[]> bufferPool = new ConcurrentBag<byte[]>();
    private int rawBufferSize;

    private int outputWidth;
    private int outputHeight;

    private int frameCount;
    private int droppedFrames;
    private int encodeBacklog;
    private float fpsTimer;
    private int fpsFrameCount;
    private bool pendingReopen;
    private string initStatus = "Starting...";

    private static readonly CultureInfo Inv = CultureInfo.InvariantCulture;
    private const float LocalHeadPoseGapMeters = 0.8f;

    // HUD
    private Canvas hudCanvas;
    private Text hudText;
    private Transform mainCamTransform;

    // Controller input
    private bool prevBButton;
    private bool remoteControlled;

    struct RawFrame
    {
        public int frameId;
        public byte[] rawData;
        public string poseJson;
    }

    struct FrameRecord
    {
        public int frameId;
        public byte[] jpgData;
        public string poseJson;
    }

    struct JointProjection
    {
        public float u;
        public float v;
        public float depth;
        public bool valid;
    }

    struct HandProjectionSet
    {
        public JointProjection[] framePose;
        public JointProjection[] unityNow;
        public JointProjection[] headPoseApi;
    }

    struct HeadPoseProjectionSource
    {
        public bool hasRawTobPose;
        public UnityEngine.Pose rawTobPose;
        public bool hasProjectionPose;
        public UnityEngine.Pose projectionPose;
        public string source;
    }

    struct HandQueryDiagnostics
    {
        public bool queryOk;
        public uint isActive;
        public uint jointCount;
        public bool hasJoints;
    }

    struct HandSnapshot
    {
        public long timestampBootNs;
        public HandJointLocations left;
        public bool leftValid;
        public HandJointLocations right;
        public bool rightValid;
    }

    private const int HandBufferCapacity = 128;
    private readonly HandSnapshot[] handBuffer = new HandSnapshot[HandBufferCapacity];
    private int handBufferHead;
    private int handBufferCount;
    private long clockOffsetNs;

    byte[] RentBuffer()
    {
        if (bufferPool.TryTake(out byte[] buf)) return buf;
        return new byte[rawBufferSize];
    }

    void ReturnBuffer(byte[] buf)
    {
        if (buf != null && buf.Length == rawBufferSize)
            bufferPool.Add(buf);
    }

    // ───────────────────── Lifecycle ─────────────────────

    void Awake()
    {
        PXR_Manager.EnableVideoSeeThrough = true;
        EnableGlobalPose("Awake");

        bool isStereo = renderMode == PXRCaptureRenderMode.PXRCapture_RenderMode_3D
                     || renderMode == PXRCaptureRenderMode.PXRCapture_RenderMode_Interlace;
        outputWidth = isStereo ? imageWidth * 2 : imageWidth;
        outputHeight = imageHeight;

        rawBufferSize = outputWidth * outputHeight * 4;
        imgBuffer = new byte[rawBufferSize];
        imgBufferHandle = GCHandle.Alloc(imgBuffer, GCHandleType.Pinned);
        texture = new Texture2D(outputWidth, outputHeight, TextureFormat.RGBA32, false);

        for (int i = 0; i < 6; i++)
            bufferPool.Add(new byte[rawBufferSize]);

        CreateHUD();
        StartCoroutine(InitWithPermission());
    }

    void EnableGlobalPose(string context)
    {
        try
        {
            PXR_Enterprise.UseGlobalPose(true);
            Debug.Log($"[EgoDC] UseGlobalPose(true) applied at {context}");
        }
        catch (Exception e)
        {
            Debug.LogWarning($"[EgoDC] UseGlobalPose(true) failed at {context}: {e.Message}");
        }
    }

    IEnumerator InitWithPermission()
    {
        initStatus = "Requesting camera permission...";

        if (!Permission.HasUserAuthorizedPermission(Permission.Camera))
        {
            Permission.RequestUserPermission(Permission.Camera);
            float timeout = 30f;
            while (!Permission.HasUserAuthorizedPermission(Permission.Camera) && timeout > 0f)
            {
                timeout -= Time.deltaTime;
                yield return null;
            }

            if (!Permission.HasUserAuthorizedPermission(Permission.Camera))
            {
                initStatus = "ERROR: Camera permission DENIED\nGrant camera permission in Settings";
                Debug.LogError("[EgoDC] Camera permission denied");
                yield break;
            }
        }

        initStatus = "InitEnterpriseService...";
        PXR_Enterprise.InitEnterpriseService();
        initStatus = "BindEnterpriseService...";
        PXR_Enterprise.BindEnterpriseService(OnServiceBound);
    }

    void Update()
    {
        // FPS counter
        fpsTimer += Time.deltaTime;
        if (fpsTimer >= 1f)
        {
            currentFps = fpsFrameCount / fpsTimer;
            fpsFrameCount = 0;
            fpsTimer = 0f;
        }

        if (pendingReopen && isRecording)
        {
            pendingReopen = false;
            PXR_Enterprise.StartGetImageDatafor4U(renderMode, outputWidth, outputHeight);
        }

        SampleHandToBuffer();

        ProcessEncodeQueue();

        HandleControllerInput();
        UpdateHUD();
    }

    void LateUpdate()
    {
        SampleHandToBuffer();
    }

    void SampleHandToBuffer()
    {
        if (!isRecording || !isCameraReady) return;

        HandSnapshot snap = new HandSnapshot();
        snap.timestampBootNs = GetBootTimeNs() - clockOffsetNs;
        try
        {
            snap.leftValid = PXR_HandTracking.GetJointLocations(HandType.HandLeft, ref snap.left);
            snap.rightValid = PXR_HandTracking.GetJointLocations(HandType.HandRight, ref snap.right);
        }
        catch (Exception)
        {
            return;
        }

        PushHandSnapshot(ref snap);
    }

    void PushHandSnapshot(ref HandSnapshot snap)
    {
        int idx = handBufferHead % HandBufferCapacity;
        handBuffer[idx] = snap;
        handBufferHead++;
        if (handBufferCount < HandBufferCapacity)
            handBufferCount++;
    }

    static long GetBootTimeNs()
    {
#if UNITY_ANDROID && !UNITY_EDITOR
        using (var systemClock = new AndroidJavaClass("android.os.SystemClock"))
        {
            return systemClock.CallStatic<long>("elapsedRealtimeNanos");
        }
#else
        return (long)(Time.realtimeSinceStartupAsDouble * 1e9);
#endif
    }

    HandSnapshot FindClosestActiveHandSnapshot(long targetBootNs)
    {
        HandSnapshot best = default;
        long bestDelta = long.MaxValue;
        int start = handBufferHead - handBufferCount;
        for (int i = start; i < handBufferHead; i++)
        {
            int idx = ((i % HandBufferCapacity) + HandBufferCapacity) % HandBufferCapacity;
            ref HandSnapshot s = ref handBuffer[idx];
            bool hasActive = (s.leftValid && s.left.isActive != 0 && s.left.jointLocations != null)
                          || (s.rightValid && s.right.isActive != 0 && s.right.jointLocations != null);
            if (!hasActive) continue;
            long delta = Math.Abs(s.timestampBootNs - targetBootNs);
            if (delta < bestDelta)
            {
                bestDelta = delta;
                best = s;
            }
        }
        return best;
    }

    void ProcessEncodeQueue()
    {
        int processed = 0;
        while (processed < 4 && encodeQueue.TryDequeue(out RawFrame raw))
        {
            byte[] jpg = EncodeRawFrameToJpg(raw.rawData);

            ReturnBuffer(raw.rawData);

            writeQueue.Enqueue(new FrameRecord
            {
                frameId = raw.frameId,
                jpgData = jpg,
                poseJson = raw.poseJson
            });
            processed++;
        }
        encodeBacklog = encodeQueue.Count;
    }

    void OnApplicationPause(bool paused)
    {
        if (!isCameraReady) return;

        if (paused)
        {
            PXR_Enterprise.CloseCamerafor4U();
        }
        else
        {
            var camSettings = new Dictionary<string, string>
            {
                { PXRCapture.KEY_MCTF, enableMCTF ? PXRCapture.VALUE_TRUE : PXRCapture.VALUE_FALSE },
                { PXRCapture.KEY_EIS, PXRCapture.VALUE_FALSE },
                { PXRCapture.KEY_MFNR, PXRCapture.VALUE_FALSE }
            };
            PXR_Enterprise.OpenCameraAsyncfor4U(ret =>
            {
                if (ret && isRecording) pendingReopen = true;
            }, camSettings);
        }
    }

    void OnDestroy()
    {
        StopRecording();
        if (isCameraReady) PXR_Enterprise.CloseCamerafor4U();
        if (imgBufferHandle.IsAllocated) imgBufferHandle.Free();
        if (texture != null) Destroy(texture);
        if (hudCanvas != null) Destroy(hudCanvas.gameObject);
    }

    // ───────────────────── HUD & Input ─────────────────────

    void CreateHUD()
    {
        mainCamTransform = Camera.main != null ? Camera.main.transform : null;

        var canvasGo = new GameObject("EgoDC_HUD");
        hudCanvas = canvasGo.AddComponent<Canvas>();
        hudCanvas.renderMode = RenderMode.WorldSpace;

        var scaler = canvasGo.AddComponent<CanvasScaler>();
        scaler.dynamicPixelsPerUnit = 10f;

        var rt = hudCanvas.GetComponent<RectTransform>();
        rt.sizeDelta = new Vector2(400, 200);
        rt.localScale = Vector3.one * 0.001f;

        var bgGo = new GameObject("BG");
        bgGo.transform.SetParent(canvasGo.transform, false);
        var bgImage = bgGo.AddComponent<Image>();
        bgImage.color = new Color(0, 0, 0, 0.6f);
        var bgRt = bgGo.GetComponent<RectTransform>();
        bgRt.anchorMin = Vector2.zero;
        bgRt.anchorMax = Vector2.one;
        bgRt.offsetMin = Vector2.zero;
        bgRt.offsetMax = Vector2.zero;

        var textGo = new GameObject("Text");
        textGo.transform.SetParent(canvasGo.transform, false);
        hudText = textGo.AddComponent<Text>();
        hudText.font = Font.CreateDynamicFontFromOSFont("Arial", 24);
        hudText.fontSize = 24;
        hudText.alignment = TextAnchor.MiddleCenter;
        hudText.color = Color.white;
        hudText.horizontalOverflow = HorizontalWrapMode.Overflow;
        hudText.verticalOverflow = VerticalWrapMode.Overflow;
        var textRt = hudText.GetComponent<RectTransform>();
        textRt.anchorMin = Vector2.zero;
        textRt.anchorMax = Vector2.one;
        textRt.offsetMin = new Vector2(10, 10);
        textRt.offsetMax = new Vector2(-10, -10);
    }

    void HandleControllerInput()
    {
        bool bButton = false;
        var devices = new List<UnityEngine.XR.InputDevice>();
        InputDevices.GetDevicesAtXRNode(XRNode.RightHand, devices);
        if (devices.Count > 0)
            devices[0].TryGetFeatureValue(CommonUsages.secondaryButton, out bButton);

        if (bButton && !prevBButton)
        {
            remoteControlled = false;
            if (isRecording)
                StopRecording();
            else
                StartRecording();
        }
        prevBButton = bButton;
    }

    void UpdateHUD()
    {
        if (hudCanvas == null || mainCamTransform == null)
        {
            if (mainCamTransform == null && Camera.main != null)
                mainCamTransform = Camera.main.transform;
            return;
        }

        // Position: 1.5m in front of camera, slightly below eye level
        Vector3 pos = mainCamTransform.position
                      + mainCamTransform.forward * 1.5f
                      + mainCamTransform.up * -0.3f;
        hudCanvas.transform.position = pos;
        hudCanvas.transform.rotation = mainCamTransform.rotation;

        string status;
        if (!isCameraReady)
        {
            hudText.color = Color.yellow;
            status = "[ INITIALIZING... ]\n\n" + initStatus;
        }
        else if (isRecording)
        {
            hudText.color = Color.red;
            string recLabel = remoteControlled ? "● REC [REMOTE]" : "● REC";
            status = $"<b>{recLabel}</b>\n\n"
                     + $"Frames: {frameCount}  |  FPS: {currentFps:F1}\n"
                     + $"Dropped: {droppedFrames}\n\n"
                     + $"Press <b>[B]</b> to STOP";
        }
        else
        {
            hudText.color = Color.green;
            status = "<b>■ READY</b>\n\n"
                     + $"Press <b>[B]</b> to START recording";
            if (recordedFrames > 0)
                status += $"\n\nLast session: {recordedFrames} frames saved";
        }
        hudText.text = status;
    }

    // ───────────────────── Initialization ─────────────────────

    void OnServiceBound(bool success)
    {
        if (!success)
        {
            initStatus = "ERROR: Enterprise service bind FAILED\nCheck: device is Enterprise edition?";
            Debug.LogError("[EgoDC] Enterprise service bind failed");
            return;
        }

        EnableGlobalPose("OnServiceBound");
        initStatus = "Service bound OK\nConfiguring camera...";

        var config = new Dictionary<string, string>
        {
            { PXRCapture.KEY_OUTPUT_CAMERA_RAW_DATA, outputRawData ? PXRCapture.VALUE_TRUE : PXRCapture.VALUE_FALSE }
        };
        PXR_Enterprise.Configurefor4U(config);

        string mctfVal = enableMCTF ? PXRCapture.VALUE_TRUE : PXRCapture.VALUE_FALSE;
        initStatus = $"Opening camera...\nMCTF={enableMCTF} Raw={outputRawData}";

        var camSettings = new Dictionary<string, string>
        {
            { PXRCapture.KEY_MCTF, mctfVal },
            { PXRCapture.KEY_EIS, PXRCapture.VALUE_FALSE },
            { PXRCapture.KEY_MFNR, PXRCapture.VALUE_FALSE }
        };
        PXR_Enterprise.OpenCameraAsyncfor4U(OnCameraOpened, camSettings);
    }

    void OnCameraOpened(bool success)
    {
        isCameraReady = success;
        if (!success)
        {
            initStatus = "ERROR: Camera open FAILED\n"
                         + "Check:\n"
                         + "1. Camera permission granted?\n"
                         + "2. No other app using camera?\n"
                         + "3. PICO 4 Ultra Enterprise?";
            Debug.LogError("[EgoDC] Camera open failed");
            return;
        }
        initStatus = "Camera opened OK\nFetching parameters...";
        EnableGlobalPose("OnCameraOpened");
        Debug.Log("[EgoDC] Camera opened successfully");
        Invoke(nameof(FetchCameraParams), 1f);
    }

    void FetchCameraParams()
    {
        cameraParams = PXR_Enterprise.GetCameraParametersNewfor4U(imageWidth, imageHeight);
        Debug.Log($"[EgoDC] Intrinsics: fx={cameraParams.fx:F4} fy={cameraParams.fy:F4} " +
                  $"cx={cameraParams.cx:F4} cy={cameraParams.cy:F4}");
        Debug.Log($"[EgoDC] Extrinsics L: pos={cameraParams.l_pos} rot={cameraParams.l_rot}");
        Debug.Log($"[EgoDC] Extrinsics R: pos={cameraParams.r_pos} rot={cameraParams.r_rot}");

        initStatus = "";
        if (autoStart) StartRecording();
    }

    // ───────────────────── Recording Control ─────────────────────

    public void StartRecording()
    {
        if (isRecording)
        {
            Debug.LogWarning("[EgoDC] Already recording");
            return;
        }
        if (!isCameraReady)
        {
            Debug.LogError("[EgoDC] Camera not ready");
            return;
        }

        EnableGlobalPose("StartRecording");

        string ts = DateTime.Now.ToString("yyyyMMdd_HHmmss");
        sessionPath = Path.Combine(Application.persistentDataPath, "egocentric_data", ts);
        framesPath = Path.Combine(sessionPath, "frames");
        Directory.CreateDirectory(framesPath);

        SaveMetadata();

        posesWriter = new StreamWriter(
            Path.Combine(sessionPath, "poses.jsonl"), false, new UTF8Encoding(false));

        encodeQueue = new ConcurrentQueue<RawFrame>();
        writeQueue = new ConcurrentQueue<FrameRecord>();
        writerRunning = true;
        writerThread = new Thread(WriterLoop) { IsBackground = true, Name = "EgoDC_Writer" };
        writerThread.Start();

        frameCount = 0;
        droppedFrames = 0;
        fpsFrameCount = 0;
        fpsTimer = 0f;
        handBufferHead = 0;
        handBufferCount = 0;
        clockOffsetNs = 0;

        IntPtr data = imgBufferHandle.AddrOfPinnedObject();
        PXR_Enterprise.SetCameraFrameBufferfor4U(outputWidth, outputHeight, ref data, OnFrameAvailable);
        bool ret = PXR_Enterprise.StartGetImageDatafor4U(renderMode, outputWidth, outputHeight);

        isRecording = true;
        currentSessionPath = sessionPath;
        Debug.Log($"[EgoDC] Recording started (StartGetImageData={ret}) → {sessionPath}");
    }

    public void StopRecording()
    {
        if (!isRecording) return;
        isRecording = false;

        while (encodeQueue.TryDequeue(out RawFrame raw))
        {
            byte[] jpg = EncodeRawFrameToJpg(raw.rawData);
            ReturnBuffer(raw.rawData);
            writeQueue.Enqueue(new FrameRecord
            {
                frameId = raw.frameId,
                jpgData = jpg,
                poseJson = raw.poseJson
            });
        }

        writerRunning = false;
        if (writerThread != null && writerThread.IsAlive)
            writerThread.Join(5000);

        lock (posesWriterLock)
        {
            if (posesWriter != null)
            {
                posesWriter.Flush();
                posesWriter.Close();
                posesWriter = null;
            }
        }

        recordedFrames = frameCount;
        Debug.Log($"[EgoDC] Recording stopped. {frameCount} frames saved, " +
                  $"{droppedFrames} dropped → {sessionPath}");
    }

    public string GetSessionPath() => sessionPath;
    public bool IsRecording() => isRecording;
    public bool IsCameraReady() => isCameraReady;
    public int GetFrameCount() => frameCount;
    public float GetCurrentFps() => currentFps;
    public void SetRemoteFlag(bool remote) { remoteControlled = remote; }

    byte[] EncodeRawFrameToJpg(byte[] rawData)
    {
        if (rotateImage180BeforeEncode)
        {
            // 相机原始缓冲区朝向与最终导出图像相差 180 度，这里按需做校正。
            EgocentricDataTransforms.RotateRgbaImage180InPlace(rawData, outputWidth, outputHeight);
        }

        if (flipImageHorizontallyBeforeEncode)
        {
            // 按输出图像宽度做逐行左右镜像，便于校正单目视角的水平反向观感。
            EgocentricDataTransforms.FlipRgbaImageHorizontallyInPlace(rawData, outputWidth, outputHeight);
        }

        texture.LoadRawTextureData(rawData);
        texture.Apply();
        return ImageConversion.EncodeToJPG(texture, jpegQuality);
    }

    // ───────────────────── Frame Capture ─────────────────────

    void OnFrameAvailable(Frame frame)
    {
        if (!isRecording) return;

        if (encodeQueue.Count + writeQueue.Count > maxQueueSize)
        {
            droppedFrames++;
            return;
        }

        int fid = frameCount++;
        long ts = (long)frame.timestamp;

        byte[] rawCopy = RentBuffer();
        Buffer.BlockCopy(imgBuffer, 0, rawCopy, 0, rawBufferSize);

        UnityEngine.Pose rawFramePose = frame.pose;
        UnityEngine.Pose headPose = EgocentricDataTransforms.ConvertRightHandedPoseToUnity(rawFramePose);
        int sensorStatus = frame.status;

        HandJointLocations leftHand = new HandJointLocations();
        HandJointLocations rightHand = new HandJointLocations();
        bool leftOk = false;
        bool rightOk = false;
        try
        {
            leftOk = PXR_HandTracking.GetJointLocations(HandType.HandLeft, ref leftHand);
            rightOk = PXR_HandTracking.GetJointLocations(HandType.HandRight, ref rightHand);
        }
        catch (Exception e)
        {
            Debug.LogWarning($"[EgoDC] Hand tracking query failed: {e.Message}");
        }
        HandQueryDiagnostics leftDiag = BuildHandQueryDiagnostics(leftOk, ref leftHand);
        HandQueryDiagnostics rightDiag = BuildHandQueryDiagnostics(rightOk, ref rightHand);
        ActiveInputDevice activeInputDevice = PXR_HandTracking.GetActiveInputDevice();
        long handQueryUnixMs = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();

        long rawBootNs = GetBootTimeNs();
        if (fid == 0)
            clockOffsetNs = rawBootNs - ts;
        long callBootNs = rawBootNs - clockOffsetNs;
        HandSnapshot callSnap = new HandSnapshot
        {
            timestampBootNs = callBootNs,
            left = leftHand, leftValid = leftOk,
            right = rightHand, rightValid = rightOk
        };
        PushHandSnapshot(ref callSnap);

        long handSnapshotBootNs = callBootNs;
        if (handBufferCount > 1)
        {
            HandSnapshot closer = FindClosestActiveHandSnapshot(ts);
            if (closer.timestampBootNs != 0
                && Math.Abs(closer.timestampBootNs - ts) < Math.Abs(callBootNs - ts))
            {
                leftHand = closer.left;
                rightHand = closer.right;
                leftOk = closer.leftValid;
                rightOk = closer.rightValid;
                handSnapshotBootNs = closer.timestampBootNs;
            }
        }

        // 旧版有效链依赖 GetHeadPose(ts) 的 global 输出；首帧额外刷新一次，避免开始录制时状态未同步。
        if (fid == 0)
            RefreshGlobalPoseForFrame();

        TOBPose headTobPose = null;
        try { headTobPose = PXR_Enterprise.GetHeadPose(ts); }
        catch (Exception) { }

        bool hasHeadPoseApiRaw = TryConvertTobPoseToUnity(headTobPose, out UnityEngine.Pose headPoseApiRaw);
        HeadPoseProjectionSource headPoseApi = ResolveHeadPoseProjectionSource(
            hasHeadPoseApiRaw, headPoseApiRaw,
            ref leftHand, leftOk,
            ref rightHand, rightOk);

        bool hasUnityNowPose = TryGetMainCameraPose(out UnityEngine.Pose unityNowPose);

        HandProjectionSet leftProjections = BuildHandProjectionSet(
            ref leftHand, leftOk,
            headPose,
            hasUnityNowPose, unityNowPose,
            headPoseApi.hasProjectionPose, headPoseApi.projectionPose);
        HandProjectionSet rightProjections = BuildHandProjectionSet(
            ref rightHand, rightOk,
            headPose,
            hasUnityNowPose, unityNowPose,
            headPoseApi.hasProjectionPose, headPoseApi.projectionPose);

        List<TOBPose> ctrlPoses = null;
        try { ctrlPoses = PXR_Enterprise.GetControllerPose(ts); }
        catch (Exception) { }

        IMUData headImu = null;
        try { headImu = PXR_Enterprise.GetHeadIMUData(ts); }
        catch (Exception) { }

        List<IMUData> ctrlImu = null;
        try { ctrlImu = PXR_Enterprise.GetControllerIMUData(ts); }
        catch (Exception) { }

        string json = BuildFrameJson(
            fid, ts, sensorStatus,
            handQueryUnixMs, handSnapshotBootNs, activeInputDevice,
            headPose,
            hasUnityNowPose, unityNowPose,
            headPoseApi,
            leftHand, leftOk, leftDiag, leftProjections,
            rightHand, rightOk, rightDiag, rightProjections,
            ctrlPoses, headImu, ctrlImu);

        encodeQueue.Enqueue(new RawFrame
        {
            frameId = fid,
            rawData = rawCopy,
            poseJson = json
        });

        fpsFrameCount++;
    }

    void RefreshGlobalPoseForFrame()
    {
        try
        {
            PXR_Enterprise.UseGlobalPose(true);
        }
        catch (Exception)
        {
            // 该调用只是刷新 SDK global pose 状态；失败时保留后续诊断字段继续写出。
        }
    }

    bool TryGetMainCameraPose(out UnityEngine.Pose pose)
    {
        if (mainCamTransform == null && Camera.main != null)
            mainCamTransform = Camera.main.transform;

        if (mainCamTransform == null)
        {
            pose = new UnityEngine.Pose(Vector3.zero, Quaternion.identity);
            return false;
        }

        pose = new UnityEngine.Pose(mainCamTransform.position, mainCamTransform.rotation);
        return true;
    }

    HandProjectionSet BuildHandProjectionSet(
        ref HandJointLocations hand, bool valid,
        UnityEngine.Pose framePose,
        bool hasUnityNowPose, UnityEngine.Pose unityNowPose,
        bool hasHeadPoseApi, UnityEngine.Pose headPoseApi)
    {
        return new HandProjectionSet
        {
            framePose = BuildHandProjection(ref hand, valid, true, framePose),
            unityNow = BuildHandProjection(ref hand, valid, hasUnityNowPose, unityNowPose),
            headPoseApi = BuildHandProjection(ref hand, valid, hasHeadPoseApi, headPoseApi)
        };
    }

    JointProjection[] BuildHandProjection(
        ref HandJointLocations hand, bool valid,
        bool hasPose, UnityEngine.Pose headPose)
    {
        if (!valid || hand.isActive == 0 || hand.jointLocations == null || !hasPose)
            return null;

        return ProjectHandJoints(ref hand, headPose);
    }

    HandQueryDiagnostics BuildHandQueryDiagnostics(bool queryOk, ref HandJointLocations hand)
    {
        return new HandQueryDiagnostics
        {
            queryOk = queryOk,
            isActive = hand.isActive,
            jointCount = hand.jointCount,
            hasJoints = hand.jointLocations != null
        };
    }

    bool TryConvertTobPoseToUnity(TOBPose tobPose, out UnityEngine.Pose pose)
    {
        if (tobPose == null)
        {
            pose = new UnityEngine.Pose(Vector3.zero, Quaternion.identity);
            return false;
        }

        EgocentricDataTransforms.ConvertRightHandedPoseToUnity(
            tobPose.x, tobPose.y, tobPose.z,
            tobPose.rx, tobPose.ry, tobPose.rz, tobPose.rw,
            out double px, out double py, out double pz,
            out double rx, out double ry, out double rz, out double rw);
        pose = new UnityEngine.Pose(
            new Vector3((float)px, (float)py, (float)pz),
            new Quaternion((float)rx, (float)ry, (float)rz, (float)rw));
        return true;
    }

    HeadPoseProjectionSource ResolveHeadPoseProjectionSource(
        bool hasRawTobPose,
        UnityEngine.Pose rawTobPose,
        ref HandJointLocations leftHand,
        bool leftOk,
        ref HandJointLocations rightHand,
        bool rightOk)
    {
        HeadPoseProjectionSource result = new HeadPoseProjectionSource
        {
            hasRawTobPose = hasRawTobPose,
            rawTobPose = rawTobPose,
            hasProjectionPose = hasRawTobPose,
            projectionPose = rawTobPose,
            source = hasRawTobPose ? "head_pose_api" : "none"
        };

        if (!hasRawTobPose)
            return result;

        if (!HeadPoseLooksLocalAgainstHands(rawTobPose, ref leftHand, leftOk, ref rightHand, rightOk))
            return result;

        // 实测发现 TOB ConvertCoordinate 的完整位姿会改变 x/z 和旋转，导致相机几何错位。
        // 这里只借用转换后的全局高度 y，x/z/rotation 保留 GetHeadPose(ts) 原始结果。
        if (TryConvertPoseCoordinate(
                PXR_EnterprisePlugin.ConvertCoordinateType.kGlobal2Local,
                rawTobPose,
                out UnityEngine.Pose globalPoseFromReverseDirection)
            && TryBuildHeightCorrectedHeadPose(
                rawTobPose,
                globalPoseFromReverseDirection,
                ref leftHand,
                leftOk,
                ref rightHand,
                rightOk,
                out UnityEngine.Pose heightCorrectedFromReverseDirection))
        {
            result.projectionPose = heightCorrectedFromReverseDirection;
            result.source = "head_pose_api_global2local_y_fallback";
            return result;
        }

        if (TryConvertPoseCoordinate(
                PXR_EnterprisePlugin.ConvertCoordinateType.kLocal2Global,
                rawTobPose,
                out UnityEngine.Pose globalPoseFromNamedDirection)
            && TryBuildHeightCorrectedHeadPose(
                rawTobPose,
                globalPoseFromNamedDirection,
                ref leftHand,
                leftOk,
                ref rightHand,
                rightOk,
                out UnityEngine.Pose heightCorrectedFromNamedDirection))
        {
            result.projectionPose = heightCorrectedFromNamedDirection;
            result.source = "head_pose_api_local2global_y_fallback";
        }
        else
        {
            result.source = "head_pose_api_coordinate_fallback_failed";
        }

        return result;
    }

    bool TryBuildHeightCorrectedHeadPose(
        UnityEngine.Pose rawPose,
        UnityEngine.Pose convertedPose,
        ref HandJointLocations leftHand,
        bool leftOk,
        ref HandJointLocations rightHand,
        bool rightOk,
        out UnityEngine.Pose correctedPose)
    {
        correctedPose = new UnityEngine.Pose(
            new Vector3(rawPose.position.x, convertedPose.position.y, rawPose.position.z),
            rawPose.rotation);
        return !HeadPoseLooksLocalAgainstHands(correctedPose, ref leftHand, leftOk, ref rightHand, rightOk);
    }

    bool HeadPoseLooksLocalAgainstHands(
        UnityEngine.Pose pose,
        ref HandJointLocations leftHand,
        bool leftOk,
        ref HandJointLocations rightHand,
        bool rightOk)
    {
        float palmYSum = 0f;
        int palmCount = 0;
        AccumulatePalmY(ref leftHand, leftOk, ref palmYSum, ref palmCount);
        AccumulatePalmY(ref rightHand, rightOk, ref palmYSum, ref palmCount);

        if (palmCount == 0)
            return false;

        float avgPalmY = palmYSum / palmCount;
        return avgPalmY - pose.position.y > LocalHeadPoseGapMeters;
    }

    void AccumulatePalmY(ref HandJointLocations hand, bool valid, ref float palmYSum, ref int palmCount)
    {
        if (!valid || hand.isActive == 0 || hand.jointLocations == null || hand.jointCount == 0)
            return;

        palmYSum += hand.jointLocations[0].pose.Position.ToVector3().y;
        palmCount++;
    }

    bool TryConvertPoseCoordinate(
        PXR_EnterprisePlugin.ConvertCoordinateType type,
        UnityEngine.Pose srcPose,
        out UnityEngine.Pose destPose)
    {
        destPose = srcPose;
        try
        {
            int ret = PXR_Enterprise.ConvertPoseCoordinate(
                type,
                srcPose,
                ref destPose);
            return ret == 0;
        }
        catch (Exception e)
        {
            Debug.LogWarning($"[EgoDC] ConvertPoseCoordinate({type}) failed: {e.Message}");
            return false;
        }
    }

    JointProjection[] ProjectHandJoints(ref HandJointLocations hand, UnityEngine.Pose headPose)
    {
        int count = Mathf.Min((int)hand.jointCount, (int)HandJoint.JointMax);
        JointProjection[] output = new JointProjection[count];
        GetProjectionCameraExtrinsics(out Vector3 cameraLocalPos, out Quaternion cameraLocalRot);

        // SDK 相机外参位置是右手系，进入 Unity 世界坐标前需要翻转 Z；旋转保持 SDK 原值更贴合实测 overlay。
        cameraLocalPos = EgocentricDataTransforms.ConvertRightHandedPositionToUnity(cameraLocalPos);
        Vector3 cameraWorldPos = headPose.position + headPose.rotation * cameraLocalPos;
        Quaternion cameraWorldRot = headPose.rotation * cameraLocalRot;
        Quaternion worldToCameraRot = Quaternion.Inverse(cameraWorldRot);

        for (int i = 0; i < count; i++)
        {
            var joint = hand.jointLocations[i];
            Vector3 jointWorld = joint.pose.Position.ToVector3();
            Vector3 jointCamera = worldToCameraRot * (jointWorld - cameraWorldPos);

            // PICO RGB 相机外参的本地前向轴与 Unity 常规相机相反，图像前方对应 -Z。
            float depth = -jointCamera.z;
            float u = 0f;
            float v = 0f;
            bool ok = depth > 0.0001f;
            if (ok)
            {
                u = (float)(cameraParams.fx * jointCamera.x / depth + cameraParams.cx);
                v = (float)(cameraParams.cy - cameraParams.fy * jointCamera.y / depth);
                ApplyExportImageTransform(ref u, ref v);
                ok = u >= 0f && u < outputWidth && v >= 0f && v < outputHeight;
            }

            output[i] = new JointProjection
            {
                u = u,
                v = v,
                depth = depth,
                valid = ok
            };
        }

        return output;
    }

    void GetProjectionCameraExtrinsics(out Vector3 position, out Quaternion rotation)
    {
        if (renderMode == PXRCaptureRenderMode.PXRCapture_RenderMode_RIGHT)
        {
            position = cameraParams.r_pos;
            rotation = cameraParams.r_rot;
            return;
        }

        position = cameraParams.l_pos;
        rotation = cameraParams.l_rot;
    }

    void ApplyExportImageTransform(ref float u, ref float v)
    {
        if (rotateImage180BeforeEncode)
        {
            u = outputWidth - 1 - u;
            v = outputHeight - 1 - v;
        }

        if (flipImageHorizontallyBeforeEncode)
            u = outputWidth - 1 - u;
    }

    // ───────────────────── Background Writer ─────────────────────

    void WriterLoop()
    {
        while (writerRunning || !writeQueue.IsEmpty)
        {
            if (writeQueue.TryDequeue(out FrameRecord record))
            {
                try
                {
                    string imgPath = Path.Combine(framesPath, $"{record.frameId:D6}.jpg");
                    File.WriteAllBytes(imgPath, record.jpgData);

                    lock (posesWriterLock)
                    {
                        if (posesWriter != null)
                        {
                            posesWriter.WriteLine(record.poseJson);
                            if (record.frameId % 30 == 0)
                                posesWriter.Flush();
                        }
                    }
                }
                catch (Exception e)
                {
                    Debug.LogError($"[EgoDC] Write error frame {record.frameId}: {e.Message}");
                }
            }
            else
            {
                Thread.Sleep(2);
            }
        }
    }

    // ───────────────────── Metadata ─────────────────────

    void SaveMetadata()
    {
        var sb = new StringBuilder(1024);
        sb.AppendLine("{");
        sb.AppendLine($"  \"device\": \"PICO 4 Ultra Enterprise\",");
        sb.AppendLine($"  \"sdk\": \"PXR Enterprise (non-OpenXR)\",");
        sb.AppendLine($"  \"session_time\": \"{DateTime.UtcNow:O}\",");
        sb.AppendLine($"  \"per_eye_width\": {imageWidth},");
        sb.AppendLine($"  \"per_eye_height\": {imageHeight},");
        sb.AppendLine($"  \"output_width\": {outputWidth},");
        sb.AppendLine($"  \"output_height\": {outputHeight},");
        sb.AppendLine($"  \"target_fps\": 30,");
        sb.AppendLine($"  \"jpeg_quality\": {jpegQuality},");
        sb.AppendLine($"  \"render_mode\": \"{renderMode}\",");
        sb.AppendLine($"  \"raw_data\": {(outputRawData ? "true" : "false")},");
        sb.AppendLine($"  \"eis\": false,");
        sb.AppendLine($"  \"mctf\": {(enableMCTF ? "true" : "false")},");
        sb.AppendLine($"  \"image_rotation_correction_degrees\": {(rotateImage180BeforeEncode ? 180 : 0)},");
        sb.AppendLine($"  \"image_horizontal_flip_correction\": {(flipImageHorizontallyBeforeEncode ? "true" : "false")},");
        sb.AppendLine($"  \"coordinate_system\": \"unity_left_handed_y_up\",");
        sb.AppendLine($"  \"pose_note\": \"Head/controller poses are converted from the SDK right-handed convention to Unity left-handed Y-up during export. Hand joints use the SDK Unity conversion. Quaternion order is (x,y,z,w).\",");
        sb.AppendLine($"  \"imu_note\": \"IMU data is stored as returned by the SDK and is not handedness-corrected.\",");
        sb.AppendLine($"  \"camera_note\": \"Camera intrinsics/extrinsics are stored exactly as returned by GetCameraParametersNewfor4U. Exported images are transformed by image_rotation_correction_degrees and image_horizontal_flip_correction after capture.\",");
        sb.AppendLine($"  \"hand_projection\": {{");
        sb.AppendLine($"    \"field\": \"joints_2d\",");
        sb.AppendLine($"    \"format\": \"[u_pixel, v_pixel, depth, valid]\",");
        sb.AppendLine($"    \"debug_fields\": [\"joints_2d_head_pose_api\", \"joints_2d_unity_now\", \"joints_2d_frame_pose\"],");
        sb.AppendLine($"    \"camera_eye\": \"{(renderMode == PXRCaptureRenderMode.PXRCapture_RenderMode_RIGHT ? "right" : "left")}\",");
        sb.AppendLine($"    \"pose_source\": \"PXR_Enterprise.GetHeadPose(frame.timestamp), with ConvertPoseCoordinate Y-only fallback selected by active-hand height compatibility when the returned pose looks local\",");
        sb.AppendLine($"    \"projection_formula\": \"camera_pos.z is converted from SDK right-handed to Unity, camera_depth=-camera_z; u=fx*x/depth+cx; v=cy-fy*y/depth; then apply exported image rotation/flip\",");
        sb.AppendLine($"    \"extrinsics_note\": \"cameraParams l_pos/r_pos are converted with z=-z before projection; l_rot/r_rot are kept as returned by GetCameraParametersNewfor4U because this matched device overlay validation best\"");
        sb.AppendLine($"  }},");
        sb.AppendLine($"  \"hand_tracking_project_enabled\": {(PXR_ProjectSetting.GetProjectConfig().handTracking ? "true" : "false")},");
        sb.AppendLine($"  \"hand_tracking_support_type\": \"{PXR_ProjectSetting.GetProjectConfig().handTrackingSupportType}\",");
        sb.AppendLine($"  \"timestamp_unit\": \"nanoseconds_boottime\",");
        sb.AppendLine($"  \"hand_query_time_note\": \"hand_query_unix_ms is DateTimeOffset.UtcNow.ToUnixTimeMilliseconds sampled immediately after GetJointLocations in OnFrameAvailable; it is diagnostic only and not BOOTTIME\",");
        sb.AppendLine($"  \"hand_query_diagnostics_note\": \"When active=false, each hand still records query_ok, sdk_is_active, joint_count, and has_joints so inactive recordings can be diagnosed.\",");
        sb.AppendLine($"  \"camera_intrinsics\": {{");
        sb.AppendLine($"    \"fx\": {Fd(cameraParams.fx)},");
        sb.AppendLine($"    \"fy\": {Fd(cameraParams.fy)},");
        sb.AppendLine($"    \"cx\": {Fd(cameraParams.cx)},");
        sb.AppendLine($"    \"cy\": {Fd(cameraParams.cy)}");
        sb.AppendLine($"  }},");
        sb.AppendLine($"  \"camera_extrinsics_left\": {{");
        sb.AppendLine($"    \"position\": [{Ff(cameraParams.l_pos.x)}, {Ff(cameraParams.l_pos.y)}, {Ff(cameraParams.l_pos.z)}],");
        sb.AppendLine($"    \"rotation_xyzw\": [{Ff(cameraParams.l_rot.x)}, {Ff(cameraParams.l_rot.y)}, {Ff(cameraParams.l_rot.z)}, {Ff(cameraParams.l_rot.w)}]");
        sb.AppendLine($"  }},");
        sb.AppendLine($"  \"camera_extrinsics_right\": {{");
        sb.AppendLine($"    \"position\": [{Ff(cameraParams.r_pos.x)}, {Ff(cameraParams.r_pos.y)}, {Ff(cameraParams.r_pos.z)}],");
        sb.AppendLine($"    \"rotation_xyzw\": [{Ff(cameraParams.r_rot.x)}, {Ff(cameraParams.r_rot.y)}, {Ff(cameraParams.r_rot.z)}, {Ff(cameraParams.r_rot.w)}]");
        sb.AppendLine($"  }},");
        sb.AppendLine($"  \"hand_joint_order\": [");
        sb.AppendLine($"    \"Palm\",\"Wrist\",");
        sb.AppendLine($"    \"ThumbMetacarpal\",\"ThumbProximal\",\"ThumbDistal\",\"ThumbTip\",");
        sb.AppendLine($"    \"IndexMetacarpal\",\"IndexProximal\",\"IndexIntermediate\",\"IndexDistal\",\"IndexTip\",");
        sb.AppendLine($"    \"MiddleMetacarpal\",\"MiddleProximal\",\"MiddleIntermediate\",\"MiddleDistal\",\"MiddleTip\",");
        sb.AppendLine($"    \"RingMetacarpal\",\"RingProximal\",\"RingIntermediate\",\"RingDistal\",\"RingTip\",");
        sb.AppendLine($"    \"LittleMetacarpal\",\"LittleProximal\",\"LittleIntermediate\",\"LittleDistal\",\"LittleTip\"");
        sb.AppendLine($"  ],");
        sb.AppendLine($"  \"hand_joint_format\": \"[px, py, pz, rx, ry, rz, rw, radius]\",");
        sb.AppendLine($"  \"imu_fields\": {{");
        sb.AppendLine($"    \"lv\": \"linear_velocity (x,y,z)\",");
        sb.AppendLine($"    \"la\": \"linear_acceleration (x,y,z)\",");
        sb.AppendLine($"    \"av\": \"angular_velocity (x,y,z)\",");
        sb.AppendLine($"    \"aa\": \"angular_acceleration (x,y,z)\"");
        sb.AppendLine($"  }}");
        sb.AppendLine("}");

        File.WriteAllText(Path.Combine(sessionPath, "metadata.json"), sb.ToString());
        Debug.Log($"[EgoDC] Metadata saved");
    }

    // ───────────────────── JSON Serialization ─────────────────────

    string BuildFrameJson(
        int fid, long timestamp, int sensorStatus,
        long handQueryUnixMs, long handSnapshotBootNs, ActiveInputDevice activeInputDevice,
        UnityEngine.Pose headPose,
        bool hasUnityNowPose, UnityEngine.Pose unityNowPose,
        HeadPoseProjectionSource headPoseApi,
        HandJointLocations leftHand, bool leftOk, HandQueryDiagnostics leftDiag, HandProjectionSet leftProjections,
        HandJointLocations rightHand, bool rightOk, HandQueryDiagnostics rightDiag, HandProjectionSet rightProjections,
        List<TOBPose> ctrlPoses,
        IMUData headImu, List<IMUData> ctrlImu)
    {
        var sb = new StringBuilder(8192);
        sb.Append('{');

        sb.Append("\"fid\":"); sb.Append(fid);
        sb.Append(",\"ts\":"); sb.Append(timestamp);
        sb.Append(",\"status\":"); sb.Append(sensorStatus);
        sb.Append(",\"hand_query_unix_ms\":"); sb.Append(handQueryUnixMs);
        sb.Append(",\"hand_snapshot_boot_ns\":"); sb.Append(handSnapshotBootNs);
        sb.Append(",\"active_input_device\":\""); sb.Append(activeInputDevice); sb.Append('"');

        // Head pose
        sb.Append(",\"head\":{\"p\":[");
        AppendV3(sb, headPose.position);
        sb.Append("],\"r\":[");
        AppendQ4(sb, headPose.rotation);
        sb.Append("]}");

        sb.Append(",\"head_raw_frame_pose\":{\"p\":[");
        AppendV3(sb, headPose.position);
        sb.Append("],\"r\":[");
        AppendQ4(sb, headPose.rotation);
        sb.Append("]}");

        sb.Append(",\"head_unity_camera_main\":");
        if (hasUnityNowPose)
        {
            sb.Append("{\"p\":[");
            AppendV3(sb, unityNowPose.position);
            sb.Append("],\"r\":[");
            AppendQ4(sb, unityNowPose.rotation);
            sb.Append("]}");
        }
        else
        {
            sb.Append("null");
        }

        sb.Append(",\"head_tob_pose_source\":\"");
        sb.Append(headPoseApi.source);
        sb.Append('"');

        sb.Append(",\"head_tob_pose_raw\":");
        if (headPoseApi.hasRawTobPose)
        {
            sb.Append("{\"p\":[");
            AppendV3(sb, headPoseApi.rawTobPose.position);
            sb.Append("],\"r\":[");
            AppendQ4(sb, headPoseApi.rawTobPose.rotation);
            sb.Append("]}");
        }
        else
        {
            sb.Append("null");
        }

        sb.Append(",\"head_tob_pose\":");
        if (headPoseApi.hasProjectionPose)
        {
            sb.Append("{\"p\":[");
            AppendV3(sb, headPoseApi.projectionPose.position);
            sb.Append("],\"r\":[");
            AppendQ4(sb, headPoseApi.projectionPose.rotation);
            sb.Append("]}");
        }
        else
        {
            sb.Append("null");
        }

        // Left hand
        sb.Append(",\"lh\":");
        AppendHandJson(sb, ref leftHand, leftOk, leftDiag, leftProjections);

        // Right hand
        sb.Append(",\"rh\":");
        AppendHandJson(sb, ref rightHand, rightOk, rightDiag, rightProjections);

        // Controller poses (TOBPose has x/y/z/rx/ry/rz/rw as doubles)
        sb.Append(",\"ctrl\":[");
        if (ctrlPoses != null)
        {
            for (int i = 0; i < ctrlPoses.Count; i++)
            {
                if (i > 0) sb.Append(',');
                var cp = ctrlPoses[i];
                EgocentricDataTransforms.ConvertRightHandedPoseToUnity(
                    cp.x, cp.y, cp.z,
                    cp.rx, cp.ry, cp.rz, cp.rw,
                    out double px, out double py, out double pz,
                    out double rx, out double ry, out double rz, out double rw);
                sb.Append("{\"ts\":"); sb.Append(cp.timestamp);
                sb.Append(",\"p\":["); sb.Append(Fd(px)); sb.Append(','); sb.Append(Fd(py)); sb.Append(','); sb.Append(Fd(pz));
                sb.Append("],\"r\":["); sb.Append(Fd(rx)); sb.Append(','); sb.Append(Fd(ry)); sb.Append(','); sb.Append(Fd(rz)); sb.Append(','); sb.Append(Fd(rw));
                sb.Append("],\"type\":"); sb.Append(cp.type);
                sb.Append(",\"conf\":"); sb.Append(cp.confidence);
                sb.Append('}');
            }
        }
        sb.Append(']');

        // Head IMU
        sb.Append(",\"imu_head\":");
        AppendImuJson(sb, headImu);

        // Controller IMU
        sb.Append(",\"imu_ctrl\":[");
        if (ctrlImu != null)
        {
            for (int i = 0; i < ctrlImu.Count; i++)
            {
                if (i > 0) sb.Append(',');
                AppendImuJson(sb, ctrlImu[i]);
            }
        }
        sb.Append(']');

        sb.Append('}');
        return sb.ToString();
    }

    void AppendHandJson(
        StringBuilder sb,
        ref HandJointLocations hand,
        bool valid,
        HandQueryDiagnostics diag,
        HandProjectionSet projections)
    {
        if (!valid || hand.isActive == 0 || hand.jointLocations == null)
        {
            sb.Append("{\"active\":false");
            AppendHandDiagnostics(sb, diag);
            sb.Append('}');
            return;
        }

        sb.Append("{\"active\":true,\"scale\":");
        sb.Append(Ff(hand.handScale));
        AppendHandDiagnostics(sb, diag);
        sb.Append(",\"joints\":[");

        int count = Mathf.Min((int)hand.jointCount, (int)HandJoint.JointMax);
        for (int i = 0; i < count; i++)
        {
            if (i > 0) sb.Append(',');

            var joint = hand.jointLocations[i];
            Vector3 pos = joint.pose.Position.ToVector3();
            Quaternion rot = joint.pose.Orientation.ToQuat();

            sb.Append('[');
            AppendV3(sb, pos);
            sb.Append(',');
            AppendQ4(sb, rot);
            sb.Append(',');
            sb.Append(Ff(joint.radius));
            sb.Append(']');
        }

        sb.Append(']');

        sb.Append(",\"joints_2d\":");
        AppendProjectionArray(sb, projections.headPoseApi);
        sb.Append(",\"joints_2d_head_pose_api\":");
        AppendProjectionArray(sb, projections.headPoseApi);
        sb.Append(",\"joints_2d_unity_now\":");
        AppendProjectionArray(sb, projections.unityNow);
        sb.Append(",\"joints_2d_frame_pose\":");
        AppendProjectionArray(sb, projections.framePose);
        sb.Append('}');
    }

    void AppendHandDiagnostics(StringBuilder sb, HandQueryDiagnostics diag)
    {
        sb.Append(",\"query_ok\":"); sb.Append(diag.queryOk ? "true" : "false");
        sb.Append(",\"sdk_is_active\":"); sb.Append(diag.isActive);
        sb.Append(",\"joint_count\":"); sb.Append(diag.jointCount);
        sb.Append(",\"has_joints\":"); sb.Append(diag.hasJoints ? "true" : "false");
    }

    void AppendProjectionArray(StringBuilder sb, JointProjection[] projections)
    {
        if (projections == null)
        {
            sb.Append("null");
            return;
        }

        sb.Append('[');
        for (int i = 0; i < projections.Length; i++)
        {
            if (i > 0) sb.Append(',');
            JointProjection p = projections[i];
            sb.Append('[');
            sb.Append(Ff(p.u)); sb.Append(',');
            sb.Append(Ff(p.v)); sb.Append(',');
            sb.Append(Ff(p.depth)); sb.Append(',');
            sb.Append(p.valid ? '1' : '0');
            sb.Append(']');
        }
        sb.Append(']');
    }

    void AppendImuJson(StringBuilder sb, IMUData imu)
    {
        if (imu == null)
        {
            sb.Append("null");
            return;
        }

        sb.Append("{\"ts\":"); sb.Append(imu.timestamp);
        sb.Append(",\"lv\":["); sb.Append(Fd(imu.vx)); sb.Append(','); sb.Append(Fd(imu.vy)); sb.Append(','); sb.Append(Fd(imu.vz));
        sb.Append("],\"la\":["); sb.Append(Fd(imu.ax)); sb.Append(','); sb.Append(Fd(imu.ay)); sb.Append(','); sb.Append(Fd(imu.az));
        sb.Append("],\"av\":["); sb.Append(Fd(imu.wx)); sb.Append(','); sb.Append(Fd(imu.wy)); sb.Append(','); sb.Append(Fd(imu.wz));
        sb.Append("],\"aa\":["); sb.Append(Fd(imu.w_ax)); sb.Append(','); sb.Append(Fd(imu.w_ay)); sb.Append(','); sb.Append(Fd(imu.w_az));
        sb.Append("]}");
    }

    void AppendV3(StringBuilder sb, Vector3 v)
    {
        sb.Append(Ff(v.x)); sb.Append(',');
        sb.Append(Ff(v.y)); sb.Append(',');
        sb.Append(Ff(v.z));
    }

    void AppendQ4(StringBuilder sb, Quaternion q)
    {
        sb.Append(Ff(q.x)); sb.Append(',');
        sb.Append(Ff(q.y)); sb.Append(',');
        sb.Append(Ff(q.z)); sb.Append(',');
        sb.Append(Ff(q.w));
    }

    static string Ff(float v) => v.ToString("G7", Inv);
    static string Fd(double v) => v.ToString("G9", Inv);
}
#endif
