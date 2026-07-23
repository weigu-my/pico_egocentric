#if !PICO_OPENXR_SDK
using System;
using System.Collections.Concurrent;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine;

public class RecordingControlServer : MonoBehaviour
{
    [SerializeField] private int port = 9877;

    private EgocentricDataCollector collector;
    private TcpListener listener;
    private Thread listenThread;
    private volatile bool serverRunning;

    private TcpClient currentClient;
    private StreamWriter clientWriter;
    private readonly object clientLock = new object();

    private ConcurrentQueue<string> incomingCommands = new ConcurrentQueue<string>();
    private ConcurrentQueue<string> outgoingResponses = new ConcurrentQueue<string>();

    void Awake()
    {
        collector = GetComponent<EgocentricDataCollector>();
        if (collector == null)
            Debug.LogError("[RecCtlSrv] EgocentricDataCollector not found on this GameObject");
    }

    void Start()
    {
        serverRunning = true;
        listenThread = new Thread(ListenLoop) { IsBackground = true, Name = "RecCtl_Listen" };
        listenThread.Start();
        Debug.Log($"[RecCtlSrv] TCP server starting on port {port}");
    }

    void Update()
    {
        while (incomingCommands.TryDequeue(out string cmdJson))
        {
            string response = ProcessCommand(cmdJson);
            if (response != null)
                outgoingResponses.Enqueue(response);
        }

        while (outgoingResponses.TryDequeue(out string resp))
            SendToClient(resp);
    }

    void OnDestroy()
    {
        serverRunning = false;
        CloseClient();
        try { listener?.Stop(); } catch { }
        if (listenThread != null && listenThread.IsAlive)
            listenThread.Join(2000);
        Debug.Log("[RecCtlSrv] Server stopped");
    }

    void ListenLoop()
    {
        try
        {
            listener = new TcpListener(IPAddress.Any, port);
            listener.Start();
            Debug.Log($"[RecCtlSrv] Listening on 0.0.0.0:{port}");
        }
        catch (Exception e)
        {
            Debug.LogError($"[RecCtlSrv] Failed to start listener: {e.Message}");
            return;
        }

        while (serverRunning)
        {
            try
            {
                if (!listener.Pending())
                {
                    Thread.Sleep(100);
                    continue;
                }

                var newClient = listener.AcceptTcpClient();
                newClient.NoDelay = true;
                newClient.ReceiveTimeout = 0;

                lock (clientLock)
                {
                    CloseClientUnsafe();
                    currentClient = newClient;
                    clientWriter = new StreamWriter(newClient.GetStream(), new UTF8Encoding(false))
                    {
                        AutoFlush = true
                    };
                }

                string ep = newClient.Client.RemoteEndPoint?.ToString() ?? "unknown";
                Debug.Log($"[RecCtlSrv] Client connected: {ep}");

                ReadClientLoop(newClient);
            }
            catch (SocketException) when (!serverRunning) { break; }
            catch (Exception e)
            {
                if (serverRunning)
                    Debug.LogWarning($"[RecCtlSrv] Accept error: {e.Message}");
            }
        }
    }

    void ReadClientLoop(TcpClient client)
    {
        try
        {
            using (var reader = new StreamReader(client.GetStream(), Encoding.UTF8))
            {
                while (serverRunning && client.Connected)
                {
                    string line = reader.ReadLine();
                    if (line == null) break;
                    line = line.Trim();
                    if (line.Length == 0) continue;
                    incomingCommands.Enqueue(line);
                }
            }
        }
        catch (IOException) { }
        catch (ObjectDisposedException) { }
        catch (Exception e)
        {
            if (serverRunning)
                Debug.LogWarning($"[RecCtlSrv] Read error: {e.Message}");
        }

        Debug.Log("[RecCtlSrv] Client disconnected");
        lock (clientLock)
        {
            if (currentClient == client)
            {
                CloseClientUnsafe();
                currentClient = null;
                clientWriter = null;
            }
        }
    }

    string ProcessCommand(string cmdJson)
    {
        string cmd;
        try
        {
            cmd = JsonUtility.FromJson<CmdMsg>(cmdJson).cmd;
        }
        catch
        {
            return "{\"type\":\"error\",\"error\":\"invalid json\"}";
        }

        if (string.IsNullOrEmpty(cmd))
            return "{\"type\":\"error\",\"error\":\"missing cmd field\"}";

        switch (cmd)
        {
            case "start_recording":
                return HandleStart();
            case "stop_recording":
                return HandleStop();
            case "status":
                return HandleStatus();
            case "ping":
                return "{\"type\":\"pong\"}";
            default:
                return $"{{\"type\":\"error\",\"error\":\"unknown cmd: {cmd}\"}}";
        }
    }

    string HandleStart()
    {
        if (collector == null)
            return AckJson("start_recording", false, "collector not found");
        if (!collector.IsCameraReady())
            return AckJson("start_recording", false, "camera not ready");
        if (collector.IsRecording())
            return AckJson("start_recording", false, "already recording");

        collector.SetRemoteFlag(true);
        collector.StartRecording();
        return $"{{\"type\":\"ack\",\"cmd\":\"start_recording\",\"ok\":true,\"session\":\"{EscapeJson(collector.GetSessionPath())}\"}}";
    }

    string HandleStop()
    {
        if (collector == null)
            return AckJson("stop_recording", false, "collector not found");
        if (!collector.IsRecording())
            return AckJson("stop_recording", false, "not recording");

        collector.SetRemoteFlag(true);
        collector.StopRecording();
        int frames = collector.GetFrameCount();
        string session = collector.GetSessionPath() ?? "";
        return $"{{\"type\":\"ack\",\"cmd\":\"stop_recording\",\"ok\":true,\"frames\":{frames},\"session\":\"{EscapeJson(session)}\"}}";
    }

    string HandleStatus()
    {
        if (collector == null)
            return "{\"type\":\"status\",\"camera_ready\":false,\"recording\":false,\"error\":\"collector not found\"}";

        bool rec = collector.IsRecording();
        bool cam = collector.IsCameraReady();
        int frames = collector.GetFrameCount();
        float fps = collector.GetCurrentFps();
        string session = collector.GetSessionPath() ?? "";

        return $"{{\"type\":\"status\",\"recording\":{BoolStr(rec)},\"camera_ready\":{BoolStr(cam)}" +
               $",\"frames\":{frames},\"fps\":{fps:F1},\"session\":\"{EscapeJson(session)}\"}}";
    }

    static string AckJson(string cmd, bool ok, string error = null)
    {
        string s = $"{{\"type\":\"ack\",\"cmd\":\"{cmd}\",\"ok\":{BoolStr(ok)}";
        if (error != null)
            s += $",\"error\":\"{EscapeJson(error)}\"";
        s += "}";
        return s;
    }

    static string BoolStr(bool v) => v ? "true" : "false";

    static string EscapeJson(string s)
    {
        if (s == null) return "";
        return s.Replace("\\", "\\\\").Replace("\"", "\\\"").Replace("\n", "\\n");
    }

    void SendToClient(string json)
    {
        lock (clientLock)
        {
            if (clientWriter == null) return;
            try
            {
                clientWriter.WriteLine(json);
            }
            catch
            {
                CloseClientUnsafe();
                currentClient = null;
                clientWriter = null;
            }
        }
    }

    void CloseClient()
    {
        lock (clientLock) { CloseClientUnsafe(); }
    }

    void CloseClientUnsafe()
    {
        try { clientWriter?.Close(); } catch { }
        try { currentClient?.Close(); } catch { }
    }

    [Serializable]
    private struct CmdMsg
    {
        public string cmd;
    }
}
#endif
