# AGX Orin 真机部署与验收手册

适用：Ubuntu 22.04、ROS 2 Humble、MID360（Livox CustomMsg）、LC29H(DA)。
当前选择纯雷达＋IMU定位，不使用D435参与估计；相机原始时间戳曾出现严重倒退，未解决前不要启用视觉融合。

## 0. 先理解当前交付边界

本仓库是三个可分别运行的模块，不是已完成的自主导航系统：

| 模块 | 已有实现 | 尚缺 |
|---|---|---|
| FAST-LIVO2专用LIO | 测量时间戳、IMU位姿/速度/协方差、滑窗、掉流停止 | 本体外参适配、重启epoch管理、真实运动精度/延迟验收 |
| GNSS门控 | 检查 `/hardware/gnss/fix` 并输出合格样本和状态 | 与LIO融合、天线杆臂补偿、初始航向和OSM对齐 |
| 道路导航 | 道路图A*、五米局部Smac规划、RPP、安全前缀续行、120秒障碍记忆 | 真实去地面/去自体点云、完整TF和道路现场核验 |

**没有完成GPS＋LIO融合。没有连接电机。没有真机闭环通过的结论。**
不得用仿真参考定位、单位外参、假协方差或重发旧样本补齐缺失接口。

## 1. 获取代码与保护现有工程

```bash
cd /workspace
git clone https://github.com/nanxk888/nav_outdoor.git
cd nav_outdoor
git log -1 --oneline
```

用独立目录，不覆盖 `/workspace/d1h-autonomy-final`、`/workspace/ws_livox`、
`/workspace/fast-livo2-deep-robotics`。现有GLIM、GNSS和相机服务由现场管理员管理；
不要使用旧 `start_mapping.py`、宽泛pkill、重置网卡或带 `--delete_db_on_start` 的命令。
配置实际标定、差分服务账号和日志存放在仓库外，例如 `/workspace/nav_outdoor_site/`，不要提交凭据。

## 2. 依赖与构建

现场已存在的可用依赖：

```bash
source /opt/ros/humble/setup.bash
source /workspace/ws_livox/install/setup.bash
source /workspace/fast-livo2-deep-robotics/install/local_setup.bash
ros2 pkg prefix livox_ros_driver2
ros2 pkg prefix vikit_common
ros2 pkg prefix vikit_ros
```

只加载环境，不启动原建图算法。若目录不同，替换成实际可用位置；
不能使用有 `/root/...` 失效路径的旧 `fast_ws` 安装环境。

ROS本身需要先安装。缺少系统依赖时，可由现场管理员安装：

```bash
sudo apt-get update
sudo apt-get install -y \
  build-essential cmake git python3-colcon-common-extensions \
  python3-numpy python3-yaml python3-scipy \
  libeigen3-dev libopencv-dev libpcl-dev libfmt-dev libboost-thread-dev \
  ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-robot-localization \
  ros-humble-pcl-ros ros-humble-pcl-conversions ros-humble-cv-bridge \
  ros-humble-image-transport ros-humble-tf2-ros ros-humble-sophus
```

`livox_ros_driver2`、`vikit_common`、`vikit_ros`源码未重复打包，需要来自现有已验证工作空间。
若已有自编译Sophus/PCL/OpenCV，先检查版本/动态库搜索路径，不盲目替换。
共享Python包的历史元数据含仿真辅助依赖；不要对整个旧工作空间盲目安装/启动仿真组件。
真机入口不需要Gazebo、机器人模型或ONNX模型。

```bash
cd /workspace/nav_outdoor
bash scripts/build_real_outdoor.sh
source install/local_setup.bash
ros2 pkg prefix d1h_fast_lio
ros2 pkg prefix d1h_global_localization
ros2 pkg prefix d1h_nav2
```

构建只选择上述三个包，限制并行编译以降低内存压力。必须在Orin本机编译，不能复用WSL的x86二进制。
失败时保留 `log/latest_build/`；缺包优先确认依赖环境，而不是复制其他架构的库。

## 3. 统一ROS通信与时间环境

每个终端加载相同ROS与本仓库环境；按现场真实配置设置ROS_DOMAIN_ID、RMW和DDS配置。
不能假定原传感器域0就是GLIM/感知服务所在域。空环境变量不等于已经与所有服务互通。

```bash
printenv ROS_DOMAIN_ID
printenv RMW_IMPLEMENTATION
printenv ROS_LOCALHOST_ONLY
ros2 node list
ros2 topic list -t
date -Is
```

系统启动时先完成校时，再初始化定位。运行过程中时钟倒退/突变，应撤销定位有效性并显式重新初始化。
LC29H的 `receiver_time_unqualified` 尚未认可为系统校时来源，不可直接用来修改主机时钟。
不要用“收到消息就重新打now()”掩盖延迟或设备时间跳变。

## 4. 验证传感器输入

本仓库不自动启动雷达、相机或GNSS串口驱动。若现有驱动已运行，不重复启动。

```bash
ros2 topic info /livox/lidar --verbose
ros2 interface show livox_ros_driver2/msg/CustomMsg
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
ros2 topic echo /hardware/gnss/fix --once
```

当前现场参考值：雷达约10Hz、IMU约200Hz。消息时间必须单调并接近主机ROS时钟，
雷达每点 `offset_time` 为原始ns，不能合成所有点同一时间。
输入频率正常不代表时间戳有效；此前D435约30Hz但时间倒退，故视觉保持关闭。

如果 `CustomMsg invalid/Package not found`，加载正确Livox消息环境，不能因此再启动一个驱动。

## 5. 终端A：专用雷达＋IMU里程计

保持机器人和传感器静止，先完成初始化。确认有急停且未运行自动运动任务。

```bash
cd /workspace/nav_outdoor
source /opt/ros/humble/setup.bash
source /workspace/ws_livox/install/setup.bash
source /workspace/fast-livo2-deep-robotics/install/local_setup.bash
source install/local_setup.bash
bash scripts/launch_real_lio_odometry.sh
```

现场配置另存后可指定：

```bash
bash scripts/launch_real_lio_odometry.sh params_file:=/workspace/nav_outdoor_site/navigation_lio.yaml
```

另一个相同通信环境的终端检查：

```bash
ros2 topic echo /navigation/lio/status
ros2 topic echo /navigation/lio/odometry --once
ros2 topic hz /navigation/lio/odometry
```

期望：`LIO_SENSOR_TRACKING`、测量时间持续增加、有效非全零协方差；
frame=`lio_odom`，child=`lio_imu`，速度在IMU子坐标系，角速度单位rad/s。
这些协方差是估计模型输出，不是硬件实测误差保证。

**本节点不发布TF，不输出base_link，不订阅图像。**
与旧GLIM并行短测虽不抢TF，但会竞争计算资源；正式测量性能应由人工只保留一个定位提供者。
输入超时、逆序、过大时间间隔或队列溢出会退出，禁止加自动respawn后恢复旧任务。
重启会重新建立局部原点，必须作废旧全局对齐、障碍记忆和路径。

## 6. 终端B：GNSS接入质量检查

先运行已有LC29H驱动。串口CP2102N、115200/8N1由原驱动维护，本仓库不重复打开设备。

```bash
cd /workspace/nav_outdoor
source /opt/ros/humble/setup.bash
source install/local_setup.bash
bash scripts/launch_real_gnss.sh
```

检查：

```bash
ros2 topic echo /localization/gnss/status
ros2 topic echo /localization/gnss/fix_filtered --once
ros2 topic echo /lc29h_gnss/health --once
```

GNSS_READY只表示样本通过门控，不代表机器人定位就绪；未知协方差默认拒绝。
NavSatStatus不是GGA Quality编码，不从status值推断RTK固定解。
默认标准差下限1m是保守软件初值，不是模块实测指标。GPS频率/延迟、室外漂移、差分状态需现场测量。
接收机TimeReference不用于自动校时。详见 [real_gnss.md](real_gnss.md)。

## 7. 必须完成的中间集成——当前尚未提供

下面不是现有可直接启动的功能，而是规划前的验收门槛：

1. 测量IMU/雷达到base_link的安装外参；做位姿、速度杆臂与协方差转换。
2. 由一个且仅一个提供者输出连续 `/odometry/local` 和 `odom→base_link`。
3. 人工输入经确认的初始机头航向，再接GPS位置约束和天线杆臂补偿。
4. 固定OSM地图datum，统一为东X/北Y米制坐标；实现匹配的 `map→odom`。
   现有地图/GPS目标采用局部经纬度近似投影，不能直接混接另一原点的UTM或任意初始yaw。
5. 不要让GPS直接跳变局部里程计；GPS失效时应明确降级，恢复后要检验跳变，不能无条件重定位。
6. 提供真实去地面、去自体点云和保留采样时间的清障观测；固定odom z=0过滤器不适合真实坡道。

不得简单把 `/navigation/lio/odometry` 重命名为 `/odometry/local`、把lio_imu改成base_link，
或把GPS天线ENU点直接冒充机器人位置。也不要启动旧仿真 `global_localization.launch.py`。

## 8. 终端C：OSM道路规划（满足上一节后）

先在仓库外创建现场配置：

```bash
mkdir -p /workspace/nav_outdoor_site
cp map/real_osm_profile.example.json /workspace/nav_outdoor_site/osm_profile.json
```

人工编辑并核验：

- `osm_file` 改为 `/workspace/nav_outdoor/map/xd_map.osm`，不能保留示例WSL路径。
- `datum_latitude_deg/longitude_deg` 与融合定位完全一致；示例值不是实测原点。
- `approved_way_ids` 是现场核验后的连通道路白名单，不自动批准全部校园道路。
- `minimum_road_width_m` 为所有白名单道路满足的实测最小净宽，需要考虑OSM中心线偏差。
- 最后才设置 `survey_confirmed=true`。默认配置会拒绝启动，这是预期保护。

规划所需输入：

| 输入 | 约定 |
|---|---|
| TF | `map→odom→base_link→lidar_link`，外参和时间已验证 |
| `/odometry/local` | 连续本体里程计，速度/协方差有效 |
| `/livox/lidar/pointcloud_nav` | 去地面/自体后的实时PointCloud2 |
| `/livox/lidar/pointcloud_clear` | 实测清障观测，不能用历史注册云伪造自由空间 |

```bash
bash scripts/launch_real_osm_navigation.sh /workspace/nav_outdoor_site/osm_profile.json
```

全局道路A*约1Hz评估；五米局部目标约5Hz评估，局部Smac规划，RPP跟踪。
雷达障碍120秒记忆参与全局检查；受地图更新周期影响，不是零延迟。
中心线段被占用时目前保守排除整段，尚未实现宽路剩余横向净宽判断。
全校园0.5m道路栅格有明显内存开销，初次只核验一小片道路。

检查 `/map`、`/navigation/display_plan`、`/navigation/replanning_status`，
RViz Fixed Frame=map。此入口不自动启动RViz；没有显示环境时先用话题检查。
候选速度只在 `/real_navigation/cmd_vel_candidate`，不会连接 `/command/cmd_twist`。
缺TF或过滤点云时不能宣称导航通过。

## 9. 先不控制电机的验收顺序

1. 静止记录：位姿漂移、速度偏置、消息时间/接收年龄、最长输出间隔。
2. 有人看护下人工移动/遥控：直线、原地转向、返回原点；独立量测误差，不能拿估计自证精度。
3. 检查真实本体位置和方向与OSM对应；不能仅看界面有线就通过。
4. 只运行规划：路口选路、路外目标拒绝、红色未知障碍进入记忆、堵路改道。
5. 测试输入停止和定位重启：撤销旧状态/路径，不自动继续任务。
6. 验证Orin温度、CPU/GPU、内存和持续运行，不把六秒静态合成测试当现场实时性验收。

控制接入另行设计急停、健康门控、指令超时、控制权和人工接管，本仓库当前不授权该步骤。

## 10. 回归、停止与更新

构建和source环境后：

```bash
bash scripts/check_real_outdoor.sh
# 或者只检查某个模块：
python3 scripts/test_gnss_quality.py
python3 scripts/test_osm_roads.py
bash scripts/check_navigation_lio.sh
ROS_DOMAIN_ID=89 python3 scripts/test_osm_road_ros.py
ROS_DOMAIN_ID=90 python3 scripts/test_real_gnss_ros.py
```

`check_navigation_lio.sh` 使用隔离ROS域93，运行C++回归和无相机静态合成传感器流，
不发布到真实传感器域。它不是运动或硬件精度测试。

2026-10-22 本地WSL/Humble检查：GNSS 14项、道路图6项、障碍记忆9项、
站位保持2项、路径缓存/取消压力和安全交接检查、OSM/GNSS ROS接口检查、
C++里程计合同及无相机合成流均通过。合成流在DDS就绪后运行6秒，收到55条里程计并验证掉流退出；
启动发现阶段单独有15秒上限。独立导出的发布树可识别且只包含三个目标包，
其纯Python GNSS/道路测试通过；没有宣称在全新系统自动装齐依赖或完成Orin实测。

发布前可用 `python3 scripts/audit_real_release.py` 检查Git暂存清单、语法和常见密钥格式。
该检查不是全面安全审计；现场配置仍应放仓库外并人工复核。

停止各自前台启动终端用Ctrl+C，不应因此重启Orin或宽泛pkill所有ROS进程。
随后确认对应节点已退出；本仓库不管理既有systemd服务，驱动仍运行不等于泄漏。
保存现场配置后更新仓库：

```bash
git status --short
git pull --ff-only
bash scripts/build_real_outdoor.sh
```

工作区有改动先确认，不使用 `git reset --hard` 覆盖现场文件。
已测本地构建/接口不代表未来每次更新自动真机验收通过。

## 11. 排错速查

| 现象 | 优先检查 |
|---|---|
| 能看见话题但CustomMsg invalid | Livox安装环境是否加载，当前终端是否可解析消息 |
| 只看见驱动，看不见定位 | ROS_DOMAIN_ID、RMW、DDS配置、容器/宿主机和生命周期状态 |
| 相机30Hz但image loop back | 原始时间单调性、设备到主机映射；保持视觉关闭，不重打now |
| GNSS门控无输出 | frame=gps_antenna、定位状态、真实数据年龄、协方差类型/大小 |
| LIO退出 | 保存具体错误，检查时间逆序/掉流/IMU间隔/队列/数据有效性；不要自动复活任务 |
| 地图看起来倾斜、机器人升空 | 坐标系、重力对齐、本体外参；不要强行z=0隐藏问题 |
| OSM规划拒绝启动 | 现场JSON白名单、实测宽度、确认标记和Linux绝对路径 |
| 规划线存在但不发电机命令 | 设计如此：只有candidate输出，控制链尚未接入 |

算法与许可分别见 [navigation_lio.md](navigation_lio.md)、[real_osm_navigation.md](real_osm_navigation.md) 和 [NOTICE](../NOTICE.md)。
