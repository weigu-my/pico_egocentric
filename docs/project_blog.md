# Project Blog（脱敏版）

> 这是本地工作 blog 的公开归档版。已将个人路径、真实内网 IP、容器名和具体硬件安全操作细节替换为占位符；真实数据、日志、模型权重和私有配置未包含在仓库中。

4.17总结
manus_quickstart.md     在docker里与manus通信及ros2发布数据及可视化步骤
plan_manus_to_gripper_retarget.md   manus手套重定向三指夹爪计划（等待硬件修改好夹爪urdf问题）
pico图像传输问题

4.21
配置pico sdk，配置unity
4.23
pico sdk sample无法控制打开相机，手柄无法移动
同事写了一版ego数据采集脚本  .cs文件

4.24
跑通unity里操控手柄打开相机等
AndroidManifest.xml要完全按照官方示例写，删除UnityPlayerGameActivity段
app里无法通过手柄射线操控的关键：https://developer.picoxr.com/zh/document/unity/create-an-xr-scene/   配置pico和unity时走完创建一个XR场景的流程
目前的unity采集程序位置：Assets/Scenes/EgocentricCapture.unity
可以采集1280*960,30fps的图像

后续需要编写脚本把pico采集的数据转为lerobot格式
4.28
脚本编写完成，位置在output文件夹里，名字为ego2lerobot.py

● ego2lerobot.py 功能总结                                                                                                                                                     
                                     
  核心转换流程                                                                                                                                                                
   
  1. 数据加载 — 读取 PICO 采集的 metadata.json + poses.jsonl + frames/*.jpg                                                                                                   
  2. 帧过滤 — 筛选右手 active、图像存在、时间戳间隔合理(≤50ms)的有效帧对
  3. 连续段切分 — 将有效帧按连续性切成多个 segment，每个 segment 独立生成一个 episode                                                                                         
  4. 相对位姿计算 — 头部/腕部相对首帧，26个手部关节在腕部坐标系下的 SE(3) 变换                                                                                                
  5. 四元数处理 — 归一化 + 按序列分别做符号连续性，action 从校正后的 state 派生                                                                                               
  6. 视频编码 — 有效帧 JPEG → resize → ffmpeg pipe → H.264/HEVC MP4                                                                                                           
  7. Parquet 输出 — 严格 LeRobot v2.1 episode-based 格式（state 196D, action 189D, hand_active 2D）                                                                           
  8. 元数据生成 — info.json, episodes.jsonl, episodes_stats.jsonl, tasks.jsonl, source_frames.jsonl                                                                           
                                                                                                                                                                              
  校验与安全机制                                                                                                                                                              
                                                                                                                                                                              
  - Wrist-in-wrist sanity check — 腕部在自身坐标系下应恒等于 [0,0,0, 0,0,0,1]                                                                                                 
  - 视频帧数验证 — 编码后 ffprobe/ffmpeg 检查 MP4 帧数 == parquet 行数，不一致报错
  - 多 session 元数据一致性 — 分辨率/旋转/翻转严格一致，intrinsics 容差 1e-4，不一致直接报错                                                                                  
  - 手部关节数检查 — joints 长度必须为 26，否则报错并指出 session/fid                                                                                                         
  - --overwrite 保护 — 输出目录已存在时默认报错，需显式 --overwrite 才删除重建                                                                                                
                                                                                                                                                                              
  CLI 参数                                                                                                                                                                    
                                                                                                                                                                              
  --input-dir, --output-dir, --sessions(可选指定), --resolution(默认 480×640), --fps(默认 30), --task, --codec(默认 h264), --overwrite                                        
   
  输出报告                                                                                                                                                                    
                                                        
  转换结束后打印每个 episode 的：原始帧数、有效帧数、hand inactive 过滤数、image missing 数、timestamp gap 数、parquet 行数、MP4 帧数。

  后续加入双手数据
  对比开源数据集质量

  5.8
  修改pico app功能，加入通过pc远程控制录制开始和结束的功能
  （已实现）
  1. 获取 Pico 的 IP 地址                                                                                                                                    
   
  通过 adb（USB 连接状态下）：                                                                                                                               
  adb shell ip addr show wlan0 | grep "inet "               
                                                                                                                                                             
  或者在 Pico 头显里：设置 → WiFi → 查看已连接网络的 IP。                                                                                                    
                                                                                                                                                             
  2. 启动 Pico 上的 App                                                                                                                                      
                                                                                                                                                             
  等 HUD 显示 ■ READY 后，TCP server 就已经在监听了。                                                                                                        
   
  3. PC 端测试连通性                                                                                                                                         
                                                            
  cd <LOCAL_HOME>/anyverse/sub_modules/roboteleop/src/pico_teleop_pkg                                                                                         
                                                                                                                                                             
  # 先 ping 测试
  python3 -m pico_egocentric_pkg.pico.recording_ctl --host <PICO_IP> ping                                                                                    
                                                                  <PICO_IP_OLD>老pico   <PICO_IP_NEW>新pico        <PICO_IP>                                                                                
  # 开始录制
  python3 -m pico_egocentric_pkg.pico.recording_ctl --host <PICO_IP> start                                                                                   
                                                                                                                                                             
  # 查看状态
  python3 -m pico_egocentric_pkg.pico.recording_ctl --host <PICO_IP> status                                                                                  
                                                                                                                                                             
  # 停止录制
  python3 -m pico_egocentric_pkg.pico.recording_ctl --host <PICO_IP> stop                                                                                    
                                                            
  也可以设环境变量省得每次输 IP：                                                                                                                            
  export PICO_DEVICE_HOST=<PICO_IP>
  python3 -m pico_egocentric_pkg.pico.recording_ctl ping
后续：
  manus手套重定向新版三指夹爪时，食指和中指没问题，但是拇指看起来是反的，后续需要解决
  pico采数据这边需要在数据处理时加一个手部关节点可视化功能
  撰写数采员使用教程

  5.11
  可视化链路完成，通过环形缓存区+双采样实现时间同步优化，可视化存在output对应的session的overlay文件夹下
  通过head_tob_pose来获得和手部统一坐标系的头部位姿（来自 PXR_Enterprise.GetHeadPose(ts)）
  目前的逻辑是如果raw GetHeadPose(ts)的y和手部不在一个高度，head_tob_pose得到的是raw GetHeadPose(ts)进行y修正后的结果，目的是使头部高度和手部在一个体系下
  可视化主要由codex完成，claude后续加入时间优化的部分
  可视化命令：
  python3 pico/overlay_hand_joints.py \
  <LOCAL_HOME>/output/egocentric_data/20260512_112057 \
  --field joints_2d \
  --output-dir <LOCAL_HOME>/output/egocentric_data/20260512_112057/overlay \
  --video --fps 30
  后续需要修改转换脚本来匹配新的时间同步优化数据采集方法

  5.12
  修改了设备待机休眠时间计入时间偏差的问题
  测试了手部离开相机视野的情况
  codex修改了ego2lerobot，但还没测试效果

  踏板控制录制：python3 -m pico_egocentric_pkg.pico.pedal_ctl --host <PICO_IP_OLD>
  左踏板start，右踏板stop

  加入坏点跳变帧过滤机制

  组会后续：试试真机回放
  
  6.12
  测试珞石真机
  容器里
  docker exec -it <DEV_CONTAINER> bash
  source /opt/ros/jazzy/setup.bash
  source /anyverse/install/setup.bash
  ros2 launch rokae_serl_config rokae_control.launch.py
  力控模式导致初始化失败，未解决
  6.15
  不用力控，纯笛卡尔位置控制：
  ros2 launch rokae_serl_config rokae_control.launch.py cartesian_control_type:=position
  replay时容易超限，卡死后需要按实验室硬件安全流程恢复控制器，并调整回安全范围内
  后续可能需要给replay数据加平滑和关节限制

  6.16
  在sim里回放数据
  python3 -m pico.sim_replay_arm <session_dir> --gui --scale 1.0 --resample-fps 30 --smooth 7 --axis-map=-z,-x,y
  原始poses文件里的坐标值是unity坐标系，需要加参数修改映射关系

  6.18
  ============ 命令速查(整条链路) ============

  [0] 进容器 + source(每个终端先做)
  docker exec -it <DEV_CONTAINER> bash
  source /opt/ros/jazzy/setup.bash && source /anyverse/install/setup.bash
  cd /anyverse/sub_modules/roboteleop/src/pico_teleop_pkg/pico_egocentric_pkg

  [1] 启动 launch(二选一);启动前确认控制器网络和硬件安全状态
  # 笛卡尔位置模式(笛卡尔回放 / probe 标定)
  ros2 launch rokae_serl_config rokae_control.launch.py cartesian_control_type:=position
  # 关节位置模式(关节回放,推荐路线)
  ros2 launch rokae_serl_config rokae_control.launch.py control_mode:=joint_position

  [2] dry-run 预览 + 画轨迹图(不连机器人,宿主机最方便)
  python3 pico/replay_arm_cartesian.py <session> --dry-run --max-jump-mm 100 --resample-fps 30 --smooth 7 --plot --plot-file /tmp/traj.png

  [3] 真机基座轴标定 probe(笛卡尔位置模式下)
  python3 -m pico.replay_arm_cartesian --probe-axes --probe-dist 0.05
  # 实测 +X=下 +Y=前 +Z=右 -> 真机 axis-map = -y,-z,x

  [4] 采集 home(笛卡尔位姿,存 arm_home.json)
  python3 -m pico.replay_arm_cartesian --set-home

  [5] 笛卡尔回放(真机);便捷脚本 bash pico/replay_real.sh <session>
  python3 -m pico.replay_arm_cartesian <session> --home --axis-map="-y,-z,x" --scale 0.3 --speed 0.3 --resample-fps 30 --smooth 7 --max-jump-mm 100 --max-step 0.003 --box 0.4

  [6] 仿真验证 + 导出关节轨迹(推荐路线);改 home 用 --home-deg "j1,..,j7"(度),改了要重新导出
  python3 -m pico.sim_replay_arm <session> --scale 1.0 --resample-fps 30 --smooth 7 --gui --export-joints /tmp/traj.npz

  [7] 真机关节回放(精确复现仿真,不受坐标系影响,推荐)
  python3 -m pico.replay_joint_real /tmp/traj.npz --dry-run
  python3 -m pico.replay_joint_real /tmp/traj.npz --speed 0.3 --rate 200

  [8] 杂项
  python3 /anyverse/move_ee.py                                  # 单轴 +z 1cm 测方向/验证控制器在发
  cp -r <LOCAL_OUTPUT>/egocentric_data/<id> <LOCAL_PROJECT_ROOT>/pico_sessions/ # session 拷到容器可见(容器内 /anyverse/pico_sessions/<id>)
  python3 -m pytest test/test_replay_arm_cartesian.py test/test_replay_joint_real.py -q

  关键提醒:
  - axis-map 两套不同:仿真用 -z,-x,y(URDF基座);真机笛卡尔用 -y,-z,x(真机基座,probe标定)
  - 帧率 = resample_fps × speed;笛卡尔有硬件SLERP低频也顺;关节通路无插值,靠 --rate 上采样(200Hz)防抽搐
  - 关节回放最稳(不看坐标系),但真机关节符号若与URDF不一致需再校
  - session 路径要容器可见(/anyverse/... 或 /tmp);长命令别粘成多行(\ 后别留空格),用 replay_real.sh 最省事


  调整初始姿态很重要，不然很容易卡死超限或者遇到奇异点

  6.29
  修改回放程序里的wrist-frame默认值为head-ego-rh，修复了机械臂运动方向和实际运动方向不一致的问题
  机械臂回放问题基本完成，灵巧手转接口还没有到，目前来看可能需要重新录制一些session，目前的session里没有手部开合动作

  7.3
  灵巧手接好了，运行时需要起三个docker，一个起灵巧手ros2节点，一个起机械臂ros2节点，一个发布replay命令
