# 网页监视第一阶段

## 导航式界面

默认LIO相对视图占据主画面，GPS/OSM在右上角作为小地图。点击小地图画面或展开按钮可交换主/小视图，亦可全屏。该整合仅是布局与交互统一，不代表GPS和LIO已经融合；LIO仍是相对轨迹，不是实时雷达点云。

地图支持左键拖动、滚轮以鼠标位置为中心缩放及加减按钮。目标为小型十字标记，点选模式下拖动只平移、不提交终点；点击后只保存预览，不自动启动机器人。目标不能超出本地OSM数据包的矩形范围，但这个粗边界检查不代表通过道路可达性审核。初始方向拖拽仍只保存预览。缩放精细不代表GPS或地图有厘米级精度。

GPS按钮明确提示：未收到数据、无有效信号、数据过期、网关断开或“已居中显示原始GPS位置”。成功只代表画面居中，不表示融合或导航就绪。参数、原始JSON和方向数值收在右上角“参数与状态”抽屉；主画面只保留状态摘要。网页控制不再需要令牌，访问者均可操作，禁止公开暴露。地图离线读取本地OSM，无CDN或在线瓦片依赖。

前端检查：`node scripts/test_web_viewport.js`、`node scripts/test_web_interactions.js`（假DOM逻辑测试，不是浏览器截图验收）。HTTP路由测试：`python3 scripts/test_navigation_web.py`。Orin服务本身不需要Node.js。静态页面与后端路由均已改变，更新后需要重启网页进程/统一launch，并用Ctrl+F5刷新浏览器；仅刷新不更新后端会使新静态文件返回404。

## 统一启动（推荐）

新增 `d1h_nav2/launch/real_workspace.launch.py`，默认一次启动GPS驱动检查器、LIO、96→0 GPS桥接、GPS门控和网页，均无自动运动授权。不启动未验收导航、融合或D1接收器；Livox硬件驱动仍由外部启动。

GPS检查器在当前Linux进程可见范围内查找精确脚本名gnss_receiver.py，不会把real_gnss_gate误判为驱动。已有驱动就复用、不终止；没有时读取原项目gnss.yaml的port，USB路径可见后通过原system_exec.sh在Domain96启动原驱动。USB未插好时只报一次缺失并等待，不阻止LIO/网页。自行启动的驱动异常退出不循环重启，报告错误；统一启动不再因此关闭其他模块。Ctrl+C只清理本检查器拥有的子进程组；旧驱动不受影响。旧驱动退出时会再次检查并尝试接管；不要与其他自动管理器竞争，也不要跨容器同时打开同一串口（PID命名空间可能互不可见）。

参数：with_gps_driver默认true，可设false关闭。gps_driver_root默认/workspace/d1h-autonomy-final，gps_driver_config默认该项目.local/robot/perception/gnss.yaml，可按现场路径覆盖。驱动和私有配置不复制进仓库。修正DDS环境变量为真正unset，避免空路径XMLPARSER错误。

默认 `log_level:=error`：运行中仅输出ROS错误，逐帧点数/残差/滑窗信息为DEBUG，不刷屏。launch本身仍可能打印少量进程启动/退出信息；异常退出的stderr与安全保护错误保留。排障可临时使用 `log_level:=debug`。修改C++日志后需要重新构建d1h_fast_lio，不能只拉取源码重启。

更新后需要重建launch所属包一次：

```bash
cd /workspace/nxk
source /opt/ros/humble/setup.bash
source /workspace/ws_livox/install/setup.bash
source /workspace/fast-livo2-deep-robotics/install/local_setup.bash
colcon build --symlink-install --packages-select d1h_nav2
source install/setup.bash
bash scripts/launch_real_workspace.sh
```

前置：Livox驱动已在Domain0发布雷达与IMU；domain_bridge、rmw_cyclonedds_cpp及原GPS驱动依赖已安装。GPS专用XML默认使用现场原项目路径，可用 `gps_cyclonedds_xml:=/absolute/file.xml` 覆盖。Ctrl+C停止本入口拥有的模块，不杀外部进程。直接子进程退出只输出名称和退出码，不联动关闭其他模块、不自动重启；LIO缺传感器仍会保护退出，运动安全联锁仍保留。不要同时运行旧同名模块或第二个网页（8765端口）。不执行sudo、pkill或网卡修改；仅缺少GPS驱动且设备可见时打开配置中的串口。

已有部分模块在其他终端运行时，可明确关闭本入口相应模块，例如 `bash scripts/launch_real_workspace.sh with_lio:=false`。支持 with_gps_bridge、with_gnss_gate、with_web。整组统一Domain0/CycloneDDS，使用gps_cyclonedds_xml指定的同一发现配置（GPS驱动仍Domain96）。默认不读取NXK控制环境配置，防止新开终端继承运动授权。

现场XML限定127.0.0.1接口、禁组播并设置本机静态peer。桥接与网页/门控使用不同DDS发现配置时，即使同Domain也可能互不可见。2026-10-24已根据现场Domain0/Cyclone能够读取桥接GPS、FastDDS不能读取的证据统一配置。不是说不同DDS实现始终不能互通；这里是发现配置不兼容。原XML不修改，网页仍可通过SSH转发访问。

手动启动网页或CLI诊断时也需要匹配环境：

```bash
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///workspace/d1h-autonomy-final/.local/robot/perception/cyclonedds.xml
unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE ROS_DISCOVERY_SERVER
```

也可直接使用：`ros2 launch d1h_nav2 real_workspace.launch.py workspace:=/workspace/nxk`。统一入口不等于传感器/融合/导航验收已完成；GPS室内无定位时仍不会绘制位置。

可选机器人控制接入见 [robot_control.md](robot_control.md)。不设置私有控制配置时仍为只读，不打开UDP控制连接；方向预览与融合初始化仍未连接。配置授权后网页提供站立/loco、确认目标导航、停止按钮；任务执行状态以后端返回为准，不以HTTP排队成功为准。

支持地图点选目标（WGS84）和拖拽/数值设置初始方向（从北顺时针）。预览POST接口校验后仅保存到内存，GET /api/state 的 operator_preview 返回记录。预览本身没有ROS目标发布或融合初始化调用，目标未检查道路可达性；不要将预览当成已下发任务。控制接口需要审核过的私有配置与安全条件，但不再需要网页令牌；UDP配对密钥保留，见控制文档。重启清空预览。写接口只接受同源浏览器请求，仍只用于可信SSH隧道，不可公开暴露。

独立实现，未复制 d1h-autonomy 源码或素材，其复制授权尚待确认。
当前支持 GPS / LIO / 状态读取、有界 LIO XY 显示，以及本地 xd_map.osm 道路和建筑轮廓的离线显示。GPS按WGS84经纬度叠加，使用Web Mercator仅作显示；不把LIO相对轨迹叠加到地理地图。无有效定位、断网或接收年龄超过2秒时不绘制当前位置。默认模式不含目标下发或电机输出；点云、融合及融合初始化尚未实现。/plan 仅缓存预览数据，不作为已验收路径。

在 Orin 仓库根目录，加载 Humble 和 install/local_setup.bash，确认 Domain 0 能收到 GPS 桥接数据和 LIO 后运行：

```bash
ROS_DOMAIN_ID=0 python3 scripts/navigation_web.py
```

电脑建立 SSH 隧道：

```bash
ssh -N -L 8765:127.0.0.1:8765 用户名@Orin地址
```

打开 http://127.0.0.1:8765 。若后端在容器内，需要将容器端口只映射到宿主机回环地址；当前绑定回环的进程在容器网络中无法直接通过普通端口发布访问，应选择合适网络模式或容器内隧道，勿直接暴露公网。

后续阶段：时间戳明确的点云预览、统一导航地图投影、停车时融合初始化/校准确认、导航控制完整假接口及实机验收。界面有按钮不是功能已实机验收的证明。

## Orin增量部署（默认只读）

LIO主视图支持橙色实时障碍点叠加：来自 `/livox/lidar/pointcloud_nav`，不是Nav2代价地图或累计三维地图。网关最多约2Hz抽取每帧2000个展示点，规划输入不受展示抽样影响。仅在点云、LIO新鲜且坐标系一致、地面过滤状态ready时绘制；超过约1.5秒或失效时隐藏并提示，不保留旧障碍冒充实时观测。当前只叠加到LIO原生相对坐标，不叠到GPS地图，不伪造全球配准。原始点云不会通过网页持续全量传输。

控制按钮按操作员要求不再随流程置灰，也不弹二次确认；每次点击提交一次请求，按钮上方显示FSM、状态是否有效、后端阶段及拒绝原因。能点击不等于会执行，后端授权/会话/停稳/姿态范围/导航安全检查仍保留。点击loco或落下本身表示操作者已确认实际停稳及现场安全。心跳继续发送，不能取消接收端超时保护。

网线接收/发送端现支持统一 UserCommand 的起立、loco、落下及静止姿态设置，部署与现场逐项测试见 [robot_control.md](robot_control.md)。默认仍只读；需显式传入审核授权的 `control_config:=/私有路径/control.json` 才开放控制，无需网页令牌。启动统一工作空间不会自动授权运动。地图目标和初始方向预览不能替代定位融合与安全监督。

先 `git status --short`，有本地修改时保留并人工处理，禁止reset覆盖。工作树干净时：

```bash
cd /workspace/nxk
git pull --ff-only origin main
source /opt/ros/humble/setup.bash
source /workspace/ws_livox/install/setup.bash
source /workspace/fast-livo2-deep-robotics/install/local_setup.bash
source install/local_setup.bash
python3 scripts/test_robot_wire.py
python3 scripts/test_navigation_control.py
unset NXK_CONTROL_CONFIG
unset NXK_WEB_TOKEN
ROS_DOMAIN_ID=0 python3 scripts/navigation_web.py
```

仅网页静态文件更新时无需重新编译；新增或修改统一launch时需重新构建d1h_nav2。需确保GPS桥接和LIO在域0；单独启动网页不会自动拉起GPS驱动，统一入口会检查并按需启动。启动依赖nav2_msgs，导入接口不等于开启运动。

电脑通过上述SSH隧道访问。终端保持运行，Ctrl+C退出。该增量不会更改网卡、GPS驱动、DDS域或厂家SDK。
