# nav_outdoor — D1H 真机 GPS / LIO 导航

面向 NVIDIA AGX Orin、Ubuntu22.04、ROS2 Humble：使用 Livox MID360 激光雷达与内置IMU进行局部定位，接收LC29H GPS，在OSM道路网络上全局选路，通过Smac局部规划和RPP跟踪生成速度，经Orin协调器与配对UDP接收器控制D1H。

本文按 **2026-11-02 当前源码**整理。`/workspace/nxk`等是现场路径示例；更换机器必须核对。专题旧文档可能保留早期参数，以当前真机launch、源码和运行参数回读为准。

> 当前是有人看护的真机试验工程，不是无人值守、安全认证产品。有操作员反馈的LIO导航成功记录，不代表每次后续修改都已真机验收。GPS与人工方向提供初始配准，**不是持续GPS/LIO融合**。进程启动、接口READY、请求排队和机器人实际完成动作必须区分。

## 目录

- [系统架构](#系统架构)
- [设备接线与通信](#设备接线与通信)
- [构建部署](#构建部署)
- [统一启动与停止](#统一启动与停止)
- [坐标系与地理登记](#坐标系与地理登记)
- [LIO定位](#lio定位)
- [Patchwork地面分割与自身过滤](#patchwork地面分割与自身过滤)
- [二维观察记忆](#二维观察记忆)
- [无GPS导航](#无gps导航)
- [GPS与OSM导航](#gps与osm导航)
- [速度与机器人控制](#速度与机器人控制)
- [网页与接口](#网页与接口)
- [主要话题与动作](#主要话题与动作)
- [室外测试流程](#室外测试流程)
- [故障排查与停止条件](#故障排查与停止条件)
- [回归测试与已知限制](#回归测试与已知限制)

## 系统架构

```text
MID360点云+内置IMU → FAST-LIVO2纯LIO → 去畸变当前扫描
                         │                    ↓
                         │                Patchwork++
                         │                    ↓ nav / clear
                         ├────────────→ 二维观察记忆 → 完整候选A*
                         │                                ↓
                         │                          LIO Smac / RPP
GPS域96 → 域0桥接 → GPS+人工方向登记                        │
                         ↓                                │
                    虚拟水平导航坐标                        │
                         ↓                                │
OSM → 道路图A* → 稳定路径缓存 → 滚动5m Smac → RPP / smoother │
                         └───────────────┬────────────────┘
                                  Orin控制协调器
                                         ↓ HMAC UDP
                              D1接收器 → 厂家UserCommand
```

| 目录 | 用途 |
|---|---|
| `src/d1h_fast_lio` | FAST-LIVO2专用纯LIO副本，不启用相机，不保存全局PCD |
| `src/d1h_global_localization` | GNSS质量门控、定位辅助；不是完整GPS/LIO融合器 |
| `src/d1h_nav2/d1h_nav2` | 地面分割适配、观察记忆、OSM规划、局部执行、速度同步 |
| `src/d1h_nav2/launch` | 真机统一启动、独立LIO Nav2、GPS Nav2组合 |
| `src/d1h_robot_bridge` | D1本机只读FSM过滤桥，需单独编译部署 |
| `scripts` | 构建启停、传感器守护、网页、控制协调器、D1接收器、测试 |
| `web` | 本地HTML/CSS/JS，无必需在线CDN |
| `map` | OSM数据和示例配置，不是雷达自动建图结果 |
| `docs` | 分模块专题和历史部署说明 |
| `.local/patchworkpp` | 私有原生绑定安装，不提交Git |

部分历史仿真/室内模块保留用于兼容，不代表仓库包含完整仿真模型与场景。真机不要把所有launch同时启动。

## 设备接线与通信

### 当前Meridian GPS接入更新

统一启动现默认 `with_gps_driver:=false gps_source_mode:=meridian0`，复用外部Domain0驱动；`gnss_topic_adapter`把 `/meridian/observation/gnss/fix` 原样转发到 `/hardware/gnss/fix`。它不启动GPS硬件驱动、不修改fix/NaN/时间/协方差。室内status=-1仍是无定位，但内部消费者能区分收到消息与完全没消息。

旧Domain96部署显式使用 `gps_source_mode:=legacy96`；仅在需要管理旧驱动且配置路径核对后加 `with_gps_driver:=true`。两种接入互斥，不同时向内部fix话题发布。后文提到Domain96的说明是旧模式，不再是新版默认。

```bash
# 当前Meridian部署，先更新并重建d1h_nav2
colcon build --symlink-install --packages-select d1h_nav2 --executor sequential
source install/local_setup.bash
bash scripts/launch_real_workspace.sh \
  sensor_height_m:=0.46 \
  gps_cyclonedds_xml:=/workspace/D1h/d1h-autonomy/.local/robot/perception/cyclonedds.xml \
  with_gps_driver:=false gps_source_mode:=meridian0 \
  with_nav:=true control_config:=/root/.config/nxk/control.json
```

| 设备或连接 | 当前约定 |
|---|---|
| MID360及内置IMU | 通过以太网UDP接入USB RTL8152网卡，通常Orin `eth1` |
| 雷达地址 | `192.168.1.137` |
| 雷达接收主机地址 | `192.168.1.50`，现场曾使用/32＋明确路由，不能漏路由 |
| D1有线控制 | D1 `192.168.19.90`、Orin `192.168.19.91`、UDP10099 |
| LC29H | USB串口/串口GPS，设备名以现场配置为准 |
| YESENSE | 独立USB IMU，不是 `/livox/imu` 的来源 |
| D435 | USB3深度相机，不参与当前纯LIO主导航链 |
| 热点 | `wlan1`、`10.42.88.1`，用于电脑SSH/网页，不是雷达口 |

MID360配置示例：

```text
/workspace/D1h/d1h-autonomy/.local/sensor-driver/install/
  livox_ros_driver2/share/livox_ros_driver2/config/MID360_config.json
```

雷达发送点云56300→主机56301、IMU56400→主机56401；命令56100/56101、推送56200/56201。实际以JSON为准。

### ROS域与DDS

- Orin导航Domain0、Cyclone DDS；外部GPS当前Domain96，由桥接进入Domain0。
- D1厂家ROS当前Domain42/localhost，命名空间由厂家 `ROBOT_NS` 提供。
- 外部Livox可以用FastDDS；统一守护以真实数据决定复用，不因RMW名字不同重复启动。
- XML可能仅绑定127.0.0.1；`ROS_LOCALHOST_ONLY=0`不表示XML允许跨主机发现。
- ROS CLI daemon可能保留旧通信环境；`ros2 daemon stop`只停止CLI缓存，不停止驱动。

诊断时统一终端环境：

```bash
source /opt/ros/humble/setup.bash
source /workspace/ws_livox/install/setup.bash
source /workspace/nxk/install/local_setup.bash
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///workspace/D1h/d1h-autonomy/.local/robot/perception/cyclonedds.xml
unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE ROS_DISCOVERY_SERVER
```

## 构建部署

### Orin

需要ROS Humble、Nav2、编译器/colcon、PCL/Eigen、Livox消息和原工程可用的vikit等依赖。外部目录缺失时定位真实安装，不创建空目录冒充依赖；不复制Windows/WSL二进制到Orin。

```bash
sudo apt-get install -y python3-venv python3-dev build-essential cmake libeigen3-dev git
cd /workspace/nxk
git pull --ff-only
source /opt/ros/humble/setup.bash
source /workspace/ws_livox/install/setup.bash
source /workspace/fast-livo2-deep-robotics/install/local_setup.bash
bash scripts/build_real_outdoor.sh
source install/local_setup.bash
```

脚本先检查/安装Patchwork++，然后串行构建 `d1h_fast_lio d1h_global_localization d1h_nav2`。Patchwork++使用官方绑定，固定上游提交 `3e6903a1d5537a4cc2ace897b0bbb98a92d6014c`，见 `scripts/requirements_patchworkpp.txt`。安装脚本使用私有构建venv、系统Eigen，不升级系统ROS NumPy；首次依赖安装需要网络。

### D1接收器

Orin构建不会自动更新D1。修改协议/策略后需单独同步并重启旧进程，不能同时开两个接收器：

```bash
scp scripts/robot_udp_receiver.py scripts/robot_wire.py scripts/robot_policy.py \
  robot@192.168.19.90:/home/robot/nxk_receiver/
```

首次部署需两端配对密钥、厂家ROS环境和只读FSM桥编译，见 [机器人控制部署](docs/robot_control.md)。已有密钥不要覆盖，不提交Git、不贴到日志。已部署脚本时可在独立前台终端启动：

```bash
ssh -t robot@192.168.19.90 \
 'bash /home/robot/nxk_receiver/start_d1_receiver.sh --authorize-motion --allow-pose'
```

授权不代表自动SDK、起立或导航。普通SSH断开可能结束前台接收器；网页/机器人心跳停车与进程是否后台存活是不同问题。

## 统一启动与停止

```bash
cd /workspace/nxk
bash scripts/launch_real_workspace.sh \
  sensor_height_m:=0.46 \
  gps_cyclonedds_xml:=/workspace/D1h/d1h-autonomy/.local/robot/perception/cyclonedds.xml \
  with_gps_driver:=false \
  with_nav:=true \
  control_config:=/root/.config/nxk/control.json
```

高度必须是**启动时Livox IMU实测离地高度**，0.46m只对应已测低姿态。初始化期间静止，不能在起落颠簸中初始化。

| 启动参数 | 默认与作用 |
|---|---|
| with_lio | true，专用里程计 |
| with_livox_driver | true，检查后按需启动/复用单个驱动 |
| livox_setup / livox_config | 驱动安装setup和MID360 JSON绝对路径 |
| livox_wait_sec | 30秒，守护等真实流；与LIO内部同步超时不同 |
| with_gps_driver | true；现场GPS外部服务已运行时显式false |
| with_gps_bridge / with_gnss_gate | true，域桥和GNSS门控 |
| with_nav_cloud | true，地面过滤及nav/clear输出 |
| with_memory | true，观察记忆＋独立LIO导航后端 |
| with_nav | true，GPS登记后启OSM Nav2；false不会关闭LIO导航 |
| nav_profile | 可选OSM/datum profile；换场地必须核对 |
| control_config | 空值禁用UDP控制；私有JSON提供有效配置后连接 |
| log_level | error，启动就绪等另有日志 |

启动顺序：工作空间锁→并行起桥接/网页/参考/记忆/局部后端→守护检查网卡与驱动→真实LiDAR/IMU新鲜递增约1秒→起LIO→LIO自身同步和初始化→过滤精确配对→记忆配对→GPS登记后启动地理Nav2。不要把 `[STARTED]` 当成数据READY。

shell锁拒绝同一入口重复启动；直接手动启动节点可能绕过锁。主进程退出仅报错，其他模块继续，不自动重启。启动检查器完成一次READY/NOT READY审计后退出，不是持续健康监控。GPS收到消息即READY时可能仍是fix=-1，不能当作有效定位。

```bash
bash scripts/stop_real_workspace.sh --dry-run
bash scripts/stop_real_workspace.sh
```

Ctrl+C停止本次launch拥有的进程；外部服务的GPS/Livox不应被误杀。脚本不广泛pkill所有ROS/Python。文件锁存在不等于仍被持有，不要删锁绕过运行中进程。软件停止不是硬件急停，不自动落下。

## 坐标系与地理登记

| 坐标系 | 定义 |
|---|---|
| lio_odom | 启动形成的重力对齐局部世界，不自带真北 |
| lio_imu | Livox IMU实际位置/姿态，不冒充厂家base_link |
| lio_navigation_base | LIO Nav2水平yaw参考 |
| odom / navigation_base | GPS参考节点生成的水平导航参考，基于IMU投影 |
| map | datum对应地理平面；登记后通过map→odom锚定 |
| navigation_sensor | 当前雷达/IMU配置派生的传感器参考 |

红色急停按钮侧为机头；实物验证后采用 **IMU −X的水平投影**，相对早期+X假设修正180°。从四元数读取机头，不从路线线段猜姿态。前端“初始朝上”只是摄像视图旋转，不修改ROS朝向。

参考节点用GPS投影点、LIO位置和人工真北航向求二维刚体变换：旋转=地理航向−LIO机头航向，平移使当前LIO位置对应GPS。输入真北0°、东90°；不是屏幕朝上就填90°。

GPS天线与IMU杆臂暂按零，未完成厂家本体外参标定。导航参考是虚拟IMU地面投影，不等于标定base_link。LIO过期/重置会撤销登记，需要静止重新登记；没有持续GPS漂移修正或跨次重定位。

## LIO定位

入口：[LIVMapper.cpp](src/d1h_fast_lio/src/LIVMapper.cpp)、[navigation_lio.yaml](src/d1h_fast_lio/config/navigation_lio.yaml)。FAST-LIVO2纯雷达/IMU：IMU传播、重力/偏置初始化、去畸变、局部体素平面地图与迭代状态估计。不启视觉、不保存全局PCD。

| 参数 | 当前值 |
|---|---|
| 输入 | /livox/lidar CustomMsg、/livox/imu Imu |
| 解码 | livox_ros_driver2；本分支lidar_type=1是CustomMsg解码选择，不表示硬件为AVIA |
| 预处理 | scan_line=4、point_filter_num=1、surface filter=0.15m、**blind=0.8m** |
| IMU | imu_int_frame=50、acc_cov=0.5、gyr_cov=0.3、重力/偏置估计启用 |
| 配准 | max_iterations=5、voxel_size=0.5m、max_layer=2、max_points_num=50 |
| 测量模型 | dept_err=0.02、beam_err=0.05、min_eigen_value=0.0025 |
| 滑动地图 | half_map_size=20、sliding_thresh=2m |
| 雷达相对IMU平移 | [0.04165,0.02326,-0.0284]m，旋转单位阵；不是底盘安装外参 |
| 时间偏移 | 雷达/IMU均0，不能据此证明硬件绝对同步 |
| 新鲜度 | max_age=0.5s、未来容忍0.05s |

**重要：下游矩形自身过滤并未取消上游LIO的0.8m预处理blind。下游不能恢复上游已删近点，不能宣称已经得到13cm近障检测能力。**

启动时雷达队列滚动保留最新6帧，满时丢旧；初始化且未输出定位时，合法但过期的IMU丢弃，不刷新健康、不触发旧样本静止检查。初始同步最多30秒，按首个IMU时间覆盖丢不匹配旧扫描。IMU缓存上限400、正常间隔检查0.05s、初始化角速度限制0.15rad/s。

进入跟踪后仍检查数据时效、时间递增、有效向量、队列过载、重力对齐和协方差。滚动队列不是无限等待/运行自动恢复；退出后需要人工重启，旧队列不会跨进程残留。

## Patchwork地面分割与自身过滤

核心：[patchwork_ground.py](src/d1h_nav2/d1h_nav2/patchwork_ground.py)、[real_cloud_filter.py](src/d1h_nav2/d1h_nav2/real_cloud_filter.py)。使用官方Patchwork++原生绑定，不是旧版全局平面RANSAC。输入已重力对齐，转换到IMU位置并做虚拟z高度规范化，不能重复旋转倾斜姿态。

| 参数 | 值 |
|---|---|
| th_dist / th_seeds | 0.02 / 0.04m |
| th_dist_v / th_seeds_v | 0.05 / 0.1m |
| min/max_range | 0.3 / 6m，和上游范围共同决定可用点 |
| sectors / rings | [16,24,32,32] / [2,2,2,2] |
| num_min_pts / num_lpr / num_iter | 6 / 10 / 3 |
| uprightness | cos(20°) |
| 模块 | RVPF、TGR启用；无可信强度，RNR关闭 |
| 输入/地面支持 | 输入≥60点；2.5m内、先验地面±0.15m支持≥20点 |
| 高度 | 初始floor=IMU z−实测高度，中位数更新每帧≤±0.03m，估计高度0.15～2m |
| nav点 | 非地面/未分类、地面上0.03～0.9m、传感器距离≤5m |
| clear点 | 已分类返回、地面相对−0.02～0.9m；未分类不作为清空证据 |
| 配对 | odom/cloud采集时间完全一致、lio_odom帧、lio_imu子帧 |
| 健康 | 状态0.1s周期，年龄上限0.5s，单扫描≤10万点 |

自身矩形以IMU为原点、实际机头为前方：前13cm、后25cm、左右各23cm。Nav2多边形：

```text
[[0.13,0.23],[0.13,-0.23],[-0.25,-0.23],[-0.25,0.23]]
```

地面层按水平位置/yaw做矩形柱状排除；Nav2用同一多边形、padding=0。不是完整三维轮足自滤或不同姿态外形模型；原地转弯需考虑角部扫掠。

健康消费者用“发布时测量年龄＋接收后年龄”。分割成功但结果过期也会ready=false；原因字符串可能仍是ground accepted，必须同时看年龄。前端仍可能因单帧失效隐藏/恢复覆盖层；没有将旧图自动当成新鲜输入。

## 二维观察记忆

核心：[observed_memory.py](src/d1h_nav2/d1h_nav2/observed_memory.py)、[lio_memory_node.py](src/d1h_nav2/d1h_nav2/lio_memory_node.py)。

| 项目 | 当前行为 |
|---|---|
| 分辨率/状态 | 0.1m，0空闲、100障碍、−1/无记录未知 |
| 保留 | 真机retention_sec=0、max_cells=0：session常驻、不按10万格淘汰；内存随探索增长 |
| 落盘 | 不保存地图文件；退出、手动重置、LIO坐标重置清空 |
| 更新 | 观测/轻状态/栅格约5Hz，发布窗口48×48m不是历史保存范围 |
| 展示 | display约1Hz，附近最多2万格，抽样不代表存储上限 |
| 候选规划 | 单后台worker约1Hz，快照成本随历史增长 |
| 目标时限 | 当前仍有300秒目标保留；与地图常驻不同 |

障碍确认：3×3邻域合计≥3点，或同格连续2次出现。普通可信区域的红格需2次连续空闲观测才清除；无回波不等于空闲。

清障规则：后方180°不允许射线清红/标空闲，仍可记录障碍；已有红格距离IMU格中心≤0.6m时默认保留并阻断继续清空后方。例外是前方60°（±30°）且距IMU格中心>13cm，允许两次可信空闲清除，当前仍有障碍点时不清。0.1m格子中心判断不是连续空间厘米级边界。

当前占地矩形未知格临时支持起点，不覆盖红格、不把整个后方涂绿、下一观测撤回，不形成虚假历史空闲。没有行人语义或3秒临时障碍自动降未知功能，后近方可残留旧红格。

候选路径：已知空闲A*，按占地复查，真实起终点参与连接，局部简化/两轮Chaikin平滑仍需验证。未知目标可生成partial前沿，仅预览不可执行。已有完整候选尝试复用，但复查/新规划失败仍可能清空saved_route，**并非永久不删除路线**。

## 无GPS导航

核心：[lio_nav2_executor.py](src/d1h_nav2/d1h_nav2/lio_nav2_executor.py)、[real_lio_navigation.launch.py](src/d1h_nav2/launch/real_lio_navigation.launch.py)。

点相对目标→完整记忆候选→显式开始→协调器确认SDK/loco/会话→锁定run/epoch/目标→Smac局部规划→RPP→平滑器→执行状态v/w→协调器UDP。命名空间 `/lio_nav`，无需GPS登记，不与GPS任务同时占控制链。

| 项目 | 当前参数 |
|---|---|
| 规划 | SmacPlanner2D LocalGrid，容差0 |
| 代价地图 | rolling12×12m、0.1m、5Hz；StaticLayer观察记忆＋InflationLayer |
| 膨胀 | 0.25m、cost_scaling=5，是障碍代价场，不是矩形额外增加25cm硬外形 |
| RPP | 不倒车，原地对准门限约15°；默认v=0.15、w=0.35 |
| 进度 | 20秒约0.05m平移；纯转动可能报Failed to make progress，外层尝试重规划 |
| 目标 | 内层xy=0.08m、yaw容差π；外层停车另行判断 |
| 原地对准 | 统一由RPP处理，外层不再使用120°锁定/15°退出状态机或强制清零v；RPP自身门限约15° |
| 重规划 | 约0.2s节流，新路线成功才更新；转向期间不再由外层冻结 |

启动后保存main_path为任务参考，局部路径按实时地图更新；输入正常但暂时无路时零速等待并保留任务。没有旧60秒任务总限时，但输入失效、目标/epoch变化、会话失效仍结束任务。

LIO到站：最后0.8m限速≤0.15m/s，按假设减速度0.3m/s²做距离限速；进入0.15m锁零、取消跟踪、拒绝迟到速度。新鲜不同时间戳LIO估计v≤0.03m/s、w≤0.05rad/s连续0.7s且仍在0.15m内才成功；超出0.25m或5秒未停稳报告异常，不自动追赶。不是厂家实测轮速/机械刹车证明，也不能直接套到GPS到站行为。

## GPS与OSM导航

### GNSS与初始登记

`real_gnss`检查fix、经纬度、时间、协方差和跳变，默认max_age=1.5s、future=0.1s、minimum_std=1m、maximum_horizontal_std=10m、未知协方差不允许、fallback_std=5m、maximum_speed=2m/s、jump_margin=10m；ENU preview默认关闭。

**参考登记实际订阅原始 `/hardware/gnss/fix`，不是filtered fix。** 不能把GNSS gate已启动等同登记经过其所有门控。参考节点检查有效fix、有限数、采样存在、GPS/LIO差≤0.3s、GPS年龄<2s、选中LIO<1s及当前pose新鲜。GPS仍是初始锚定，不持续改正漂移。

### 全局与局部算法

OSM解析道路图并A*选路→StablePlanner缓存/校验→截前方5m和后方0.3m→局部SmacPlanner2D→RPP跟踪→平滑输出。全局约1Hz、局部约5Hz，局部段完成不等于最终任务完成。

无道路审批/测绘审核布尔开关；OSM只是拓扑先验，不证明现场无障碍、允许通行或地形适合。profile需真实绝对路径osm_file、datum_latitude_deg、datum_longitude_deg和道路宽度等，见 `map/`示例。换场地需同时核对网页底图和后端profile，不保证自动同步。

离道路起点：投影最近道路段，保留真实位置，连接最多20m；整段需已观察空闲证据和矩形占地检查。不支持自动进入20m未知区，也不保证绕找另一个入口或任意离道路终点可达。

StablePlanner默认：global_period=1s、local_period=0.2s、剩余3.5m刷新、目标位移0.03m、路径偏离0.5m、校验0.3s、接续6s、制动假设0.5m/s²、反应0.4s、停余量0.25m。

真机BT去掉倒车恢复，保留碰撞检查旋转/等待。behavior_server恢复旋转当前另设0.4rad/s，不等于网页的RPP原地对准参数；提高网页转向上限不代表每种行为都同比提高速度。

### 不同障碍层

| 数据层 | 用途/保留 |
|---|---|
| 共用观察记忆 | 0.1m session常驻，LIO及GPS接入检查/参考输入 |
| GPS独立obstacle_memory | 真机分辨率覆盖为0.5m、默认保留300s、stable_sec=0、max_tracks=256、max_cells=100000、max_range=5m |
| GPS直接ObstacleLayer | 消费nav/clear点云，参与局部碰撞/清除，不等于共用观察记忆 |

**后方180°及0.6m保护不能宣称已经覆盖GPS所有直接射线清除层。** 清除共用记忆也不等于清空GPS独立记忆和所有Nav2 costmap。无GPS修改同步必须逐层核对，而不是只改一个函数就认定两条链完全一致。

## 速度与机器人控制

入口：`scripts/navigation_control.py`（唯一UDP下发协调器）、`nav2_speed_sync.py`（服务确认/回读）、`robot_udp_receiver.py`、`robot_policy.py`、`robot_wire.py`（D1协议与策略）。

| 项目 | 当前设置 |
|---|---|
| 前端前进上限 | 可调0.1～1m/s，默认0.15 |
| 前端转向上限 | 可调0.1～1.5rad/s，默认0.35 |
| 协议包络 | v=0～1m/s，abs(w)≤1.5rad/s，禁止倒车 |
| 同步 | RPP desired_linear_vel、rotate_to_heading_angular_vel；smoother max/min_velocity，参数读回后READY |
| 最终限速 | 共同比例缩放v/w保持曲率，调高上限不保证弯道恒定达到该速度 |
| 协议 | JSON envelope＋HMAC-SHA256，自定义配对协议，不是厂家二进制UDP |
| 包约束 | ≤4096字节、时间偏差≤0.25s，需两机时钟同步 |
| 接收看门狗 | 约0.2s无新有效指令零速/撤销会话；接收器断电时另需厂家看门狗 |

接收器检查来源、签名、时间、会话nonce/序号、SDK、实时FSM与授权。Nav2/local不能绕开协调器各自向电机发送。SDK退出停止运动，不关闭所有网页模块；恢复SDK不自动恢复旧任务。

起立和loco独立：transform_up→STAND_READY→人工确认→loco→READY；支持接管人工已进入loco的机器人，先零速会话，不自动起立。姿态/落下与导航互斥；非中立姿态可拒绝导航。roll±0.2rad、pitch±0.4rad，高度默认不开放。电池只读，每60秒请求，不根据电压猜百分比。

私有配置示例：

```json
{"orin_ip":"192.168.19.91","robot_ip":"192.168.19.90","robot_port":10099,"key_file":"/root/.config/nxk/udp.key"}
```

绑定地址必须在本机存在。机器人没开机与Orin配置绑定错误不同。协议更新后必须重启D1旧Python进程，否则超过旧上限仍会被拒绝。

## 网页与接口

HTTP默认仅 `127.0.0.1:8765`，通过可信SSH隧道访问，不暴露公网：

```bash
ssh -N -L 8765:127.0.0.1:8765 root@10.42.88.1
# 浏览器 http://127.0.0.1:8765
```

目录：机器人状态、速度、GPS融合导航、无GPS导航、机器人控制、loco姿态、数据明细、日志。日志最多2000条、可清除，记录当前页面看到的事件，不是完整ROS日志，不包含页面关闭期间事件。

| 接口 | 用途 |
|---|---|
| GET /api/state | 采样及控制状态；value可能对象或JSON字符串，receive_age不等于测量年龄 |
| GET /api/osm | 网页OSM几何 |
| POST /api/operator/goal | 经纬度目标预览，不自动运动 |
| POST /api/operator/heading | 人工方向预览，不自动登记 |
| POST /api/operator/register | 用选中GPS/LIO采样登记导航 |
| POST /api/operator/lio_goal | LIO候选目标 |
| POST /api/operator/memory_reset | 清共用记忆/候选目标，epoch变化让前端清轨迹 |

控制请求字段以 `navigation_web.py::do_POST`和 `web/control.js`为准；queued表示排队，不代表成功。没有网页Bearer令牌，但仍检查同源Origin和私有硬件连接配置。

GPS面板“清除观察记忆与LIO轨迹”复用memory_reset，先停止导航再操作。它不重启LIO/里程计原点、不删OSM、不自动清登记，也不是所有代价层总清除；epoch变化可终止LIO任务。

轨迹线存浏览器内存、最近2000点，不按时间过期；刷新、LIO中断/坐标重置或记忆epoch变化清空。与后台常驻红绿记忆不同。

## 主要话题与动作

String状态正文通常为JSON。以下列主执行链接口，不把Nav2内部所有诊断和参数事件当成电机入口。

| 名称 | 类型 | 发布→消费/用途 |
|---|---|---|
| /livox/lidar | livox_ros_driver2/msg/CustomMsg | driver→LIO、守护 |
| /livox/imu | sensor_msgs/msg/Imu | driver→LIO、守护，不是YESENSE |
| /navigation/lio/odometry | nav_msgs/msg/Odometry | LIO→filter/memory/reference/local/web |
| /navigation/lio/deskewed_cloud | sensor_msgs/msg/PointCloud2 | LIO→filter，同采集时间 |
| /navigation/lio/status | std_msgs/msg/String | LIO健康→web/检查 |
| /livox/lidar/pointcloud_nav | PointCloud2 | filter→memory/GPS costmap/障碍记忆/web |
| /livox/lidar/pointcloud_clear | PointCloud2 | filter→memory/GPS clearing |
| /navigation/cloud_filter/status | String | filter→reference/local/memory/web |
| /hardware/gnss/fix | sensor_msgs/msg/NavSatFix | GPS/域桥→gate/reference/web |
| /localization/gnss/fix_filtered | NavSatFix | gate过滤输出 |
| /localization/gnss/status | String | GNSS质量/拒绝原因 |
| /localization/gnss_antenna_enu | geometry_msgs/msg/PointStamped | 可选ENU预览，默认关闭 |
| /navigation/memory/grid | nav_msgs/msg/OccupancyGrid | memory→LIO costmap/local/OSM连接检查 |
| /navigation/memory/status | String | memory→local/reference/web |
| /navigation/memory/display | String | 低频抽样cells→web |
| /navigation/memory/path_candidate | nav_msgs/msg/Path | 候选路径，空消息使旧候选失效 |
| /navigation/memory/goal_candidate | String | web→memory |
| /navigation/memory/reset | String | web→memory，带epoch/请求时间 |
| /navigation/reference/register | String | web→reference |
| /odometry/local | Odometry | reference→GPS Nav2，子帧navigation_base |
| /real_navigation/safety_status | String | reference→control/等待器/web |
| /navigation/local_execution/request | String | control→local start/heartbeat/stop |
| /navigation/local_execution/status | String | local→control/web，state/v/w/plan/main_path/诊断 |
| /lio_navigation/odometry | Odometry | local→独立LIO Nav2 |
| /lio_navigation/cmd_vel_raw | geometry_msgs/msg/Twist | LIO RPP→smoother |
| /lio_navigation/cmd_vel | Twist | LIO smoother→executor，不直达电机 |
| /navigation/speed_limits | String | control→两套参数同步/执行器 |
| /lio_navigation/speed_status | String | LIO后端参数读回 |
| /real_navigation/speed_status | String | GPS后端参数读回 |
| /map | OccupancyGrid | OSM道路走廊→costmap/记忆 |
| /navigation/global_memory_map | OccupancyGrid | GPS独立障碍记忆 |
| /navigation/remembered_obstacles | PointCloud2 | GPS记忆障碍 |
| /navigation/obstacle_memory_status | String | GPS独立记忆诊断 |
| /real_navigation/cmd_vel_raw | Twist | GPS RPP/behavior→smoother |
| /real_navigation/cmd_vel_candidate | Twist | GPS smoother→control |
| /lio_nav/compute_path_to_pose | nav2_msgs/action/ComputePathToPose | local→LIO Smac |
| /lio_nav/follow_path | nav2_msgs/action/FollowPath | local→LIO RPP |
| /navigate_to_pose | nav2_msgs/action/NavigateToPose | control→GPS BT |
| /navigation/compute_global_path | ComputePathToPose | BT→StablePlanner包装 |
| /navigation/compute_local_path | ComputePathToPose | BT→StablePlanner包装 |
| /roads/compute_path_to_pose | ComputePathToPose | StablePlanner→OSM |
| /local_planner/compute_path_to_pose | ComputePathToPose | StablePlanner→GPS Smac |
| /tf、/tf_static | tf2_msgs/msg/TFMessage | 参考变换，不是速度指令 |

D1输出为厂家命名空间下UserCommand/FSM接口，不是跨机直接透传cmd_vel，具体厂家环境见控制专题。驱动RELIABLE发布与BEST_EFFORT订阅可兼容；endpoint存在不证明流持续，UNKNOWN节点名不单独证明网络坏。

## 室外测试流程

1. 推送/Orin拉取构建；更新并重启D1接收器；核对测试地点OSM和datum。
2. 检查雷达IP/路由、GPS外部驱动、机器人有线链路，避免重复驱动和指令发布者。
3. 静止按实际高度统一启动；等IMU初始化、LIO跟踪、ground有效、记忆持续更新。
4. 人工站稳进入SDK/loco，确认新鲜D1反馈。起落导致定位退出时先恢复定位，不继续登记。
5. 应用低速，例如v=0.15m/s、w=0.3rad/s，确认参数读回；首轮不用最大速度。
6. GPS有效稳定后输入真北航向，静止点击GPS+方向登记。
7. 确认地理位置、点云和现场方向匹配，GPS Nav2就绪；顶部有信号不代表全链可执行。
8. 同一道路前方3～5m选目标，显式开始GPS导航，观察NAVIGATING和实际规划路线。
9. 验证方向、绕障、到站停稳，再增加距离/转弯/离道路接入；LIO与GPS任务不并发。
10. 保持网页心跳，故障先停并留日志，别连点开始绕过拒绝。换场地或LIO重启后重新登记。

## 故障排查与停止条件

| 提示/现象 | 解释与排查 |
|---|---|
| latest_imu=-1、imu_queue=0 | 新LIO未收到IMU，查DDS和真实数据，不是旧队列残留 |
| 首IMU过期直接退出 | 检查是否真正更新并重建C++；新版初始化丢旧等待 |
| ground age反复越0.5s | 区分上游延迟、处理耗时和接收延迟；看backend/processing_ms/测量年龄 |
| ground accepted但ready=false | 可能分割成功结果过期，不一定算法失败 |
| 无backend字段 | 可能旧源码/旧安装/旧进程，查模块加载路径和发布者 |
| footprint blocked | 查具体段、未知/障碍和真实起点；绿色中心线不代表整个矩形通过 |
| Full path unavailable | 启动需要完整路线，partial不能执行；候选可被复查清空 |
| Grid/memory mismatch | 查源时间配对、重复发布者、处理延迟，非无限扩大超时 |
| Failed to make progress | 查纯转动/受阻/动作重规划，不能直接当网络故障 |
| Browser/robot heartbeat | 网页约2s、机器人反馈约0.3s，分别检查 |
| outside commissioning limits | D1旧协议/未重启进程或真实超限 |
| 随后Operator stop | 可能上游故障触发自动stop，找最早拒绝原因 |
| HTTP400 HTML | 请求格式/Origin/配置/网关，不能直接认定机器人UDP坏 |
| control.jso | 配置文件拼错，示例control.json |
| Cannot assign requested address | 本机绑定地址不存在，先查ip addr/route |
| USB -71 / r8152断开 | 曾有真实网卡重枚举，需查供电/线/Hub，不能直接认定过热 |

```bash
# 先使用前面的统一DDS环境
pgrep -af '[l]ivox_ros_driver2_node|[l]io_odometry'
ros2 node list --no-daemon
ros2 topic info /livox/imu --verbose
timeout -s INT 8s ros2 topic hz /livox/imu
timeout -s INT 8s ros2 topic hz /livox/lidar
timeout -s INT 8s ros2 topic echo /navigation/cloud_filter/status std_msgs/msg/String --full-length --qos-reliability best_effort
ros2 param get /lio_nav/controller_server FollowPath.rotate_to_heading_angular_vel
ros2 param get /lio_nav/velocity_smoother max_velocity
curl -s http://127.0.0.1:8765/api/state
ip -br -4 addr
ip route get 192.168.1.137
timeout 5s tcpdump -ni any -c 5 'src host 192.168.1.137 and udp dst port 56401'
sudo journalctl -k --since '5 minutes ago' --no-pager
```

控制健康：LIO执行输入里程计接收/测量通常<0.4s、ground有效年龄<0.5s、memory/grid<1s且源时间匹配；任务lease约0.3s。D1/SDK/FSM、浏览器心跳、参数同步、参考登记、目标epoch变化都能停止任务。输入恢复后不自动恢复旧运动。

## 回归测试与已知限制

### 不连接硬件的测试

```bash
python3 scripts/test_observed_memory.py
python3 scripts/test_body_footprint.py
python3 scripts/test_memory_pair.py
python3 scripts/test_heading_align.py
python3 scripts/test_arrival.py
python3 scripts/test_ground_health.py
python3 scripts/test_navigation_speed.py
python3 scripts/test_robot_wire.py
python3 scripts/test_robot_policy.py
python3 scripts/test_navigation_control.py
python3 scripts/test_navigation_web.py
node scripts/test_web_control.js
node scripts/test_web_interactions.js
node scripts/test_navigation_log.js
```

实际ROS/原生绑定测试需要已安装依赖；**隔离域/localhost运行，不向真实Domain0发布模拟传感器**：

```bash
python3 scripts/test_patchwork_ground.py
ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1 python3 scripts/test_patchwork_ground_ros.py
ROS_DOMAIN_ID=94 ROS_LOCALHOST_ONLY=1 python3 scripts/test_real_navigation_execution_ros.py --local
```

脚本存在不代表每次都执行；测试通过不证明真机精度、地形适用性、最高速度或机械停车已验收。

### 仍需明确的边界

- 无持续GPS融合、自动航向/杆臂标定、回环闭合和跨运行地图重定位。
- 上游blind=0.8m与近障感知意图存在差异，需量测deskewed cloud覆盖再修改。
- 矩形规划姿态近似、转向扫掠、趴姿/站姿外形仍需真机验证；非完整3D机身自滤。
- GPS直接ObstacleLayer与共用观察记忆的清障策略不完全相同。
- 无行人语义/动静态生命周期分类；后近障保护可能保留残影。
- 地面分割不是完整可通行性/坑洞检测，OSM不保证真实可通行。
- session记忆不设容量，长期内存/快照/排序开销增长，暂无分块落盘地图。
- 浏览器轨迹、后台红绿记忆、LIO局部配准地图、GPS独立障碍记忆是不同数据。
- USB网络恢复不代表已退出LIO自动恢复，自动重连和恢复任务尚未实现。
- 软件零输出不能代替硬件急停；接收器断电时依赖厂家自身看门狗。

维护时逐层核对：帧/时间→参数→发布接口→消费者→拒绝原因→前端→测试→D1是否需更新。不能只改网页max漏协议，或只改LIO就声称GPS全部同步。Git采用发布白名单，新增文档/脚本可能需显式纳入。

专题索引：[统一启动](docs/workspace_startup.md)、[雷达守护](docs/livox_unified_startup.md)、[Patchwork++](docs/patchworkpp_ground.md)、[LIO导航](docs/lio_local_navigation.md)、[GPS执行](docs/navigation_execution.md)、[OSM导航](docs/real_osm_navigation.md)、[D1部署](docs/robot_control.md)。旧文档与本文冲突时核对当前真机源码，不照抄旧审批、半径和速度。

## 许可

见 [NOTICE.md](NOTICE.md)。FAST-LIVO2副本保留GPLv2说明；Patchwork++按其上游许可使用；OSM须保留OpenStreetMap contributors署名及ODbL说明。项目自定义UDP配对层不代表厂家官方网络协议。
