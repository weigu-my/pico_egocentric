# 项目记忆与交接说明

> 这是一份面向个人 GitHub 归档的项目记忆文档。文档只保留可复用的技术路线、模块说明、运行命令和排障经验；上传公开仓库前请删除真实采集数据、日志、设备序列号、内网 IP、公司私有路径和任何未授权的第三方二进制文件。

## 1. 项目总览

本项目围绕机器人遥操作、第一视角示教数据采集、数据格式转换、真实机器人回放和灵巧手重定向展开，核心目标是把人类使用 PICO 头显采集到的第一视角图像、头部位姿、手腕轨迹和手部关节数据，转换为可用于机器人学习和真实机器人执行验证的数据与控制命令。

主要闭环包括：

- PICO 头显 Unity App 采集第一视角 RGB 图像、头部/手腕位姿和 26 关节手部追踪。
- PC 端通过 TCP 命令或 USB 踏板远程控制 PICO App 开始/停止录制。
- 离线将 PICO session 转成 LeRobot 风格数据，支持质量检查和关节点 overlay 可视化。
- 从 PICO 右手腕轨迹生成 Rokae 机械臂 replay 轨迹，支持 dry-run、滤波、限幅、坐标轴映射和真机前安全检查。
- 从 PICO 手部姿态中识别 grasp/release 事件，并在机械臂 replay 时同步给 BrainCo Revo2 灵巧手下发张合命令。
- 接入 BrainCo Revo2 官方 SDK，解决 CANFD 通信参数、Docker/ROS2 环境和手指位置控制问题。
- Manus 手套到三指夹爪/灵巧手方向的重定向工具链也在同一项目中积累了一部分实现和文档。

## 2. 目录结构

当前工作目录通常位于：

```bash
<repo>/src/pico_teleop_pkg/pico_egocentric_pkg
```

核心目录：

```text
pico/
  recording_ctl.py          # PC 端远程控制 PICO 录制：ping/start/stop/status
  pedal_ctl.py              # USB 踏板触发 PICO 录制
  overlay_hand_joints.py    # 将手部 2D 关节点叠加到图像/视频上
  hand_event_detector.py    # 从 PICO 手部关节中识别 grasp/release 事件
  replay_arm_cartesian.py   # PICO 手腕轨迹到 Rokae 笛卡尔 replay
  replay_joint_real.py      # 仿真导出的关节轨迹到真机 replay
  sim_replay_arm.py         # MuJoCo 中的机械臂轨迹预检/IK
  revo2_hand_node.py        # ROS2 节点：订阅手指目标并驱动 Revo2
  session_paths.py          # host/docker/session 名称路径解析
  revo2_retarget_optimizer.py
  offline_revo2_replay.py

manus/
  ergonomics_parser.py
  gripper_kinematics.py
  gripper_retarget.py
  gripper_retarget_node.py
  config/gripper_retarget_config.yaml

docs/
  pico_data_collection_guide.md
  manus_glove_linux.md
  manus_quickstart.md
  plan_manus_to_gripper_retarget.md
  resume_projects.md
  project_memory_for_github.md

test/
  test_hand_event_detector.py
  test_revo2_hand_node.py
  test_session_paths.py
  test_replay_arm_cartesian.py
  ...
```

PICO 端 Unity 工程不在该 Python 包内，之前定位到的本机工程是：

```bash
<unity-project>/pico_egocentric
```

Unity 工程中最重要的文件：

```text
Assets/Scenes/EgocentricCapture.unity
Assets/Scripts/EgocentricDataCollector.cs
Assets/Scripts/RecordingControlServer.cs
Assets/Scripts/EgocentricDataTransforms.cs
Assets/Plugins/Android/AndroidManifest.xml
Packages/manifest.json
ProjectSettings/ProjectVersion.txt
```

如果要迁移 Unity 工程，通常只需要保留 `Assets/`、`Packages/`、`ProjectSettings/`。`Library/` 是 Unity 缓存，体积很大，不建议上传。

## 3. PICO 数据采集链路

### 3.1 PICO 端 Unity App

Unity App 名称为 `EgocentricCapture`。核心脚本是 `EgocentricDataCollector.cs`，负责：

- 打开 PICO 4 Ultra Enterprise 的 passthrough/RGB 相机。
- 以约 1280x960、30fps 采集图像。
- 同步记录头部位姿、控制器位姿、IMU 和左右手 26 关节追踪。
- 保存 `metadata.json`、`poses.jsonl` 和 `frames/*.jpg`。
- 通过 HUD 显示 READY/REC/STOP 等状态。
- 支持右手控制器按键本地开始/停止录制。

`RecordingControlServer.cs` 负责在 PICO 上启动 TCP server，默认监听：

```text
0.0.0.0:9877
```

支持的 JSON 命令：

```json
{"cmd":"ping"}
{"cmd":"start_recording"}
{"cmd":"stop_recording"}
{"cmd":"status"}
```

PC 端 `recording_ctl.py` 和 `pedal_ctl.py` 都是这个 TCP server 的客户端。

### 3.2 PC 端控制命令

进入包目录后，手动控制：

```bash
cd <repo>/src/pico_teleop_pkg

python3 -m pico_egocentric_pkg.pico.recording_ctl --host <PICO_IP> ping
python3 -m pico_egocentric_pkg.pico.recording_ctl --host <PICO_IP> start
python3 -m pico_egocentric_pkg.pico.recording_ctl --host <PICO_IP> status
python3 -m pico_egocentric_pkg.pico.recording_ctl --host <PICO_IP> stop
```

踏板控制：

```bash
python3 -m pico_egocentric_pkg.pico.pedal_ctl --host <PICO_IP>
```

踏板映射：

```text
Ctrl+Space  -> start recording
Ctrl+Right  -> stop recording
```

### 3.3 数据保存与导出

文档早期写过 `/sdcard/EgocentricData/`，但当前 Unity 脚本实际使用：

```csharp
Application.persistentDataPath/egocentric_data/<session>
```

在 Android/PICO 上通常会落到类似：

```text
/sdcard/Android/data/<package-name>/files/egocentric_data/<session>
```

稳妥做法是先查实际目录：

```bash
adb shell find /sdcard -type d -name egocentric_data
```

再导出：

```bash
adb pull <PICO_EGOCENTRIC_DATA_DIR> <local-data-dir>/egocentric_data
```

一个 session 的标准结构：

```text
20260629_174011/
  metadata.json
  poses.jsonl
  frames/
    000000.jpg
    000001.jpg
    ...
```

## 4. 数据质量检查与可视化

检查帧数：

```bash
for d in <local-data-dir>/egocentric_data/20*; do
  echo "$(basename "$d"): $(wc -l < "$d/poses.jsonl") frames"
done
```

生成手部关节点 overlay：

```bash
cd <repo>/src/pico_teleop_pkg

python3 -m pico_egocentric_pkg.pico.overlay_hand_joints \
  <local-data-dir>/egocentric_data/<SESSION_NAME> \
  --field joints_2d \
  --output-dir <local-data-dir>/egocentric_data/<SESSION_NAME>/overlay \
  --video --fps 30
```

质量检查重点：

- `poses.jsonl` 是否每帧都有记录。
- `frames/*.jpg` 是否和有效 pose 帧匹配。
- 右手/左手 `active` 是否稳定。
- `joints_2d` 是否覆盖在真实手上。
- 是否有明显 tracking 突跳、手离开视野、时间戳 gap。
- 多个 session 合并时，图像分辨率、旋转/翻转、相机内参要一致。

## 5. BrainCo Revo2 灵巧手接入

### 5.1 SDK 安装结论

使用的硬件是 BrainCo Revo2 右手，CANFD 连接。官方仓库：

```text
https://github.com/BrainCoTech/brainco-hand-sdk
```

建议放在：

```bash
<repo>/third_party/brainco-hand-sdk
```

安装 Python 依赖时，官方脚本：

```bash
bash install_whl.sh 2.0.2
```

曾经遇到 OSS wheel 404。可用 PyPI 安装：

```bash
python3 -m pip install "numpy<2.0.0" colorlog asyncio
python3 -m pip install bc-stark-sdk==2.0.2
```

不要直接污染系统 Python。推荐在项目虚拟环境或 Docker 内单独创建 venv。

### 5.2 Docker/ROS2 环境结论

机械臂 replay 需要在 Docker 内执行，因为 Docker 内有 ROS2 和 Rokae 相关环境。Revo2 也最终在 Docker 内跑通。

Docker 内关键检查：

```bash
ip link show canfd0
python3 -c "import rclpy; print('rclpy ok')"
python3 -c "import bc_stark_sdk; print('bc_stark_sdk ok')"
```

如果系统 Python 报 `externally-managed-environment`，不要强行全局安装 pip 包，应该新建 Docker 内虚拟环境，例如：

```bash
cd <repo-root>
python3 -m venv --system-site-packages .venv-brainco-docker
source .venv-brainco-docker/bin/activate
python -m pip install -U pip
python -m pip install colorlog "numpy<2.0.0" asyncio
python -m pip install bc-stark-sdk==2.0.2
```

### 5.3 CANFD 排障记录

遇到过的问题：

- CAN 口显示 UP，`candump` 能看到 TX，但没有 RX。
- SDK demo 报 `No response from slave`。
- 同事笔记里也出现过“只能发送报文，不能收到 Revo2 回复”。

最终有效结论：

- 使用 CANFD。
- bitrate 1M，dbitrate 5M。
- extended frame。
- BRS 不能打开，Revo2 侧需要 BRS off。
- slave id 使用十进制 `127`，也就是 `0x7f`。

CANFD 初始化示例：

```bash
sudo ip link set canfd0 down
sudo ip link set canfd0 type can bitrate 1000000 dbitrate 5000000 fd on restart-ms 100
sudo ip link set canfd0 up
ip -details -statistics link show canfd0
```

抓包观察：

```bash
candump -tA -x -e canfd0
```

官方 demo 验证：

```bash
cd <repo-root>/third_party/brainco-hand-sdk/python
python3 demo/hand_demo.py -B canfd0 127 5
```

### 5.4 Revo2 手指命令含义

Revo2 Python API 中常用命令：

```python
await ctx.set_finger_positions(slave_id, [1000, 1000, 1000, 1000, 1000, 1000])
```

这里 6 个数是 Revo2 的 6 个主动通道，范围一般按 `0..1000` 使用：

- `0`：接近张开。
- `1000`：接近最大弯曲/握紧。
- 中间值：对应中间弯曲程度。

实际顺序以官方 SDK/设备定义为准。项目里为了抓瓶子使用过：

```python
[500, 750, 850, 900, 900, 900]  # 抓握
[0, 0, 0, 0, 0, 0]              # 张开
```

### 5.5 ROS2 Revo2 手节点

本项目增加了：

```text
pico/revo2_hand_node.py
```

作用：

- 在 Docker 内连接 Revo2。
- 订阅 ROS2 topic `/revo2/right_hand/target`。
- 接收 `std_msgs/msg/Float64MultiArray`。
- 将 6 个 `0..1000` 手指目标下发给 Revo2。

启动：

```bash
cd <repo>/src/pico_teleop_pkg/pico_egocentric_pkg
source <repo-root>/.venv-brainco-docker/bin/activate
python -m pico.revo2_hand_node --iface canfd0 --slave-id 127
```

测试张开/抓握：

```bash
ros2 topic pub --once /revo2/right_hand/target std_msgs/msg/Float64MultiArray \
  "{data: [0, 0, 0, 0, 0, 0]}"

ros2 topic pub --once /revo2/right_hand/target std_msgs/msg/Float64MultiArray \
  "{data: [1000, 1000, 1000, 1000, 1000, 1000]}"
```

重要踩坑：

- 最开始节点打印“命令已下发”但手不动。
- 官方 demo 使用的是 `set_finger_positions`。
- speed API `set_finger_positions_and_speeds` 在当前设备/固件组合下没有正常驱动。
- 所以节点默认改为 `DEFAULT_SPEED = 0`，默认走 plain position API；只有显式传 `--speed > 0` 才走 speed API。

## 6. PICO 手部事件识别与 Revo2 同步

### 6.1 hand_event_detector

新增：

```text
pico/hand_event_detector.py
```

作用是从 `poses.jsonl` 的右手 26 关节中计算手部开合分数，识别：

- `grasp`
- `release`

默认参数：

```text
grasp_threshold  = 0.45
release_threshold = 0.35
hold_frames = 3
min_event_gap = 0.5s
grasp command = [500, 750, 850, 900, 900, 900]
release command = [0, 0, 0, 0, 0, 0]
```

命令：

```bash
python3 -m pico.hand_event_detector <SESSION_DIR_OR_NAME>
```

示例输出：

```text
grasp   fid=44  score=0.482  cmd=[500, 750, 850, 900, 900, 900]
release fid=131 score=0.250  cmd=[0, 0, 0, 0, 0, 0]
```

关键逻辑修改：

- `hold_frames=3` 保留，用于避免单帧误触发。
- 事件 fid 记录为“连续确认窗口的第一帧”，不是第三帧确认时刻。
- 这样 replay 对齐时更接近真实人手开始抓/放的时间。

### 6.2 replay_arm_cartesian 中的手事件同步

`pico/replay_arm_cartesian.py` 增加了 `--hand-events`，用于机械臂 replay 过程中同步下发 Revo2 指令。

dry-run 检查：

```bash
python3 -m pico.replay_arm_cartesian <SESSION_NAME> \
  --dry-run --hand-events --resample-fps 30 --smooth 5 \
  --max-jump-mm 100 --wrist-frame head-ego-rh --axis-map=-z,y,x \
  --scale 0.3 --box 0.4
```

真机 replay 推荐走脚本：

```bash
bash pico/replay_real.sh <SESSION_NAME> --hand-events
```

如果手动写完整命令，注意加 `--home`：

```bash
python3 -m pico.replay_arm_cartesian <SESSION_NAME> \
  --home --wrist-frame head-ego-rh --axis-map=-z,y,x --scale 0.3 \
  --speed 0.1 --resample-fps 30 --smooth 5 --max-jump-mm 100 \
  --max-step 0.003 --box 0.4 --hand-events
```

曾经问过“为什么真机回放时没有先回到 home”。结论：

- `--home` 仍然存在。
- dry-run 不会真的回 home。
- `pico/replay_real.sh` 默认带 `--home`。
- 手动运行 `python -m pico.replay_arm_cartesian ...` 时如果没加 `--home`，就不会先回 home。

## 7. session 路径解析

新增：

```text
pico/session_paths.py
```

原因：

- host 上数据可能在 `<home>/anyverse/data/egocentric_data`。
- Docker 里可见路径可能是 `/anyverse/data/egocentric_data`。
- 早期命令里用过 `<home>/output/egocentric_data/<session>`，Docker 内可能读不到。

现在支持：

- 直接传完整 session 目录。
- 只传 session 名，例如 `20260629_174011`。
- host/docker 路径互相映射。
- 遇到旧的 `/output/egocentric_data/<session>` 风格路径时，取最后的 session 名去默认数据根目录查找。

推荐命令中直接传 session 名：

```bash
python3 -m pico.replay_arm_cartesian 20260629_174011 --dry-run --hand-events ...
```

## 8. Rokae 机械臂 replay 链路

### 8.1 笛卡尔 replay

核心文件：

```text
pico/replay_arm_cartesian.py
```

功能：

- 从 `poses.jsonl` 读取右手腕轨迹。
- 以首帧为原点计算相对位移。
- 使用 `--axis-map` 将 PICO/Unity 坐标映射到机器人基座坐标。
- 支持缩放、重采样、平滑、离群点过滤、单帧最大步长限制、工作空间盒约束。
- live 模式发布机械臂末端笛卡尔命令。

常用安全参数：

```text
--dry-run
--resample-fps 30
--smooth 5
--max-jump-mm 100
--max-step 0.003
--scale 0.3
--box 0.4
--speed 0.1
```

真机前建议流程：

1. `--dry-run` 看帧数、范围、是否有 hand events。
2. 画轨迹/检查方向。
3. 小 scale、低 speed 上机。
4. 确认方向正确后再逐步放开参数。

### 8.2 仿真与关节 replay

核心文件：

```text
pico/sim_replay_arm.py
pico/replay_joint_real.py
```

思路：

- `sim_replay_arm.py` 在 MuJoCo 里把末端轨迹通过 IK 转成 7 关节轨迹，并检查限位、残差、跳变等问题。
- `replay_joint_real.py` 读取 `.npz` 关节轨迹，通过 ROS2 关节位置控制器回放。

关节 replay 的优点：

- 避开不同控制器对笛卡尔基座坐标解释不一致的问题。
- 更容易在上真机前用仿真轨迹做预检。

## 9. Manus 与三指夹爪重定向

相关目录：

```text
manus/
docs/manus_glove_linux.md
docs/manus_quickstart.md
docs/plan_manus_to_gripper_retarget.md
docs/troubleshooting_manus_migration.md
```

已完成/积累内容：

- Manus Quantum 手套在 Linux/Docker/ROS2 下的接入说明。
- udev、USB 透传、SDK library、硬件访问权限、ROS2 topic 排查。
- 从 Manus ergonomics 字段解析拇指、食指、中指相关关节。
- 三指夹爪 URDF 解析、主动关节与 mimic 关节处理。
- `ManusGripperRetargeter` 通过 YAML 做 `q = gain * value + offset` 映射。
- 发布 `/gripper_joint_commands` 的 ROS2 节点。
- 单元测试覆盖 URDF、FK、字段解析、限位、平滑等逻辑。

## 10. 关于 pi0.5 / VLA 部署的结论

曾经讨论过“没有相机的 Rokae 双臂能不能部署 pi0.5 模型”。

结论：

- 不能按 VLA 模型的原始预期直接部署。
- pi0.5/类似 VLA 策略通常需要图像观测、语言指令和机器人状态一起输入。
- 如果机器人没有相机，就缺失关键视觉输入，只能做 replay、状态策略或非视觉控制。
- 要做 pi0.5/VLA 真机部署，至少需要补齐外部相机或腕部相机、完成相机标定、采集动作-视觉数据，并适配动作空间。

当前项目更适合的方向：

- 先用 PICO 示教数据做数据采集、重放验证和 LeRobot 转换。
- 有相机以后再做 VLA fine-tuning 或部署。
- 没相机时，可先做轨迹 replay、hand event replay、状态空间策略或离线数据格式准备。

## 11. 之前关键问答浓缩

### BrainCo SDK 应该放哪里？

建议放在项目的第三方依赖目录：

```bash
<repo-root>/third_party/brainco-hand-sdk
```

这样和项目代码、Docker 挂载、路径解析都更容易对齐。

### `install_whl.sh 2.0.2` 404 是不是报错？

是安装脚本找 OSS wheel 失败，不代表 SDK 不能用。实际使用：

```bash
python3 -m pip install bc-stark-sdk==2.0.2
```

可以正常安装。

### 要不要改环境？

建议不要改系统 Python。使用 venv，Docker 内另建 Docker 专用 venv。

### `python3 -m venv` 报 ensurepip 不存在怎么办？

Ubuntu/Debian 需要：

```bash
sudo apt install python3.10-venv
```

或使用当前 Docker/系统对应版本的 `python3-venv` 包。

### `rclpy` 找不到是因为没开机械臂/灵巧手电源吗？

不是。`ModuleNotFoundError: No module named 'rclpy'` 是 Python 环境问题，不是硬件电源问题。需要在有 ROS2 的 Docker 环境里运行，或 source 正确 ROS2 setup。

### Docker 能看到 `canfd0` 但找不到 `bc_stark_sdk` 怎么办？

说明 CAN 设备已经透传进容器，但 Python 包没装到容器当前解释器。需要在 Docker venv 里安装 `bc-stark-sdk==2.0.2`。

### Revo2 节点显示 ready 是否就可用？

如果日志能读到 device info，并显示订阅 `/revo2/right_hand/target`，说明节点已经连接成功。还需要用 ROS2 topic pub 实测手是否会动。

### 下发命令但手不动怎么办？

项目里最终发现默认 speed API 不可靠。改成默认 `set_finger_positions` 后手可以动。

### `5` 和 `1` 分别是什么含义？

在官方 demo 命令里，接口参数后面的数字通常是 demo 选择或动作编号，不是 CAN ID。实际设备从站 ID 是 `127`。具体 demo 参数要以官方 `demo/hand_demo.py` 的 argparse 为准。

### `0..1000` 是每个手指弯曲度吗？

可以按归一化开合/弯曲目标理解。`0` 接近张开，`1000` 接近最大弯曲/握紧。6 个通道对应 Revo2 的 6 个主动自由度。

### 如何在控制机械臂的同时控制 Revo2？

把机械臂 replay 和 Revo2 手节点拆成两个 ROS2 通道：

- 机械臂 replay 发布机械臂命令。
- `revo2_hand_node.py` 订阅 `/revo2/right_hand/target` 并控制手。
- `replay_arm_cartesian.py --hand-events` 在到达 grasp/release 帧时发布手部命令。

### 如何从 PICO 录制数据中读取手部开合？

使用 `hand_event_detector.py` 从右手 26 关节计算 curl/开合分数。当前只识别事件，不做连续精细重定向。

### release 阈值改了为什么 fid 还是靠后？

因为事件需要连续 `hold_frames` 帧确认。后来把事件 fid 改为连续确认窗口第一帧，而不是确认完成那一帧。

### Docker 读不到 host 侧旧数据目录怎么办？

把 session 放到 Docker 可见的数据目录，或者传 session 名让 `session_paths.py` 自动解析。推荐数据目录：

```bash
<repo-root>/data/egocentric_data/<SESSION_NAME>
```

Docker 内通常对应：

```bash
/anyverse/data/egocentric_data/<SESSION_NAME>
```

## 12. 测试与验证记录

常用测试：

```bash
python3 test/test_session_paths.py
python3 test/test_hand_event_detector.py
python3 test/test_revo2_hand_node.py
python3 -m py_compile pico/session_paths.py pico/replay_arm_cartesian.py pico/hand_event_detector.py pico/revo2_hand_node.py
```

曾经验证通过的能力：

- session 路径解析：完整路径、session 名、host/docker 映射、旧 output 路径兼容。
- hand event detector：grasp/release、hold_frames、事件 fid 取窗口第一帧、命令映射。
- Revo2 hand node：默认 plain position API，手指目标归一化。
- replay dry-run：能输出有效帧数、轨迹范围和手事件调度帧。

## 13. 上传个人 GitHub 前清理清单

建议上传：

- `pico/*.py`
- `manus/*.py`
- `manus/config/*.yaml`
- `docs/*.md`
- `test/*.py`
- 小型示例数据或脱敏 sample
- `setup.py`、`package.xml` 等包元数据

不要上传：

- 真实采集数据：`egocentric_data/`、`frames/*.jpg`、真实 `poses.jsonl`。
- 机器人运行日志：`logs/`、`_rokae_log_/`、`*.log`。
- Python 缓存：`__pycache__/`、`.pytest_cache/`。
- Unity 缓存：`Library/`、`Temp/`、`Obj/`、`Logs/`、`Builds/*BackUpThisFolder*`。
- 大体积 APK/模型权重，除非确认可以公开发布。
- 第三方闭源 SDK wheel、授权信息、硬件访问凭据。
- 设备序列号、公司内网 IP、真实机器人控制器地址、个人用户名路径。
- 公司私有 URDF/CAD/配置，除非确认有权开源。

建议加 `.gitignore`：

```gitignore
__pycache__/
*.pyc
.pytest_cache/
logs/
_rokae_log_/
*.log
egocentric_data/
data/
output/

# Unity
Library/
Temp/
Obj/
Logs/
Builds/*BackUpThisFolder*

# Python env
.venv*/

# Large/runtime files
*.bag
*.db3
*.mp4
*.jpg
*.npz
*.parquet
```

## 14. 后续可继续推进的方向

- 把 PICO Unity 工程整理成单独仓库，只保留 `Assets/Packages/ProjectSettings` 和 README。
- 把 `pico_data_collection_guide.md` 中旧的 `/sdcard/EgocentricData/` 更新为实际 `Application.persistentDataPath/egocentric_data` 路径说明。
- 为 Revo2 手指通道顺序写一份硬件标定表，逐个通道发 `0/500/1000` 记录对应动作。
- 把 grasp/release 事件识别扩展成连续手势强度输出，实现更平滑的手指同步。
- 在真实机器人上统一记录 replay 日志：session、参数、axis-map、scale、是否成功抓取、失败原因。
- 如后续部署 VLA/pi0.5，先补齐外部相机/腕部相机、标定和机器人动作数据格式。
