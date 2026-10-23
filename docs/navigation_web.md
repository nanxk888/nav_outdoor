# 网页监视第一阶段

## 统一启动（推荐）

新增 `d1h_nav2/launch/real_workspace.launch.py`，默认一次启动我们的LIO、96→0 GPS桥接、GPS门控和网页，均无自动运动授权。不会启动原项目硬件驱动，不启动未验收导航、融合或D1接收器。

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

前置：Livox驱动已在Domain0发布雷达与IMU，GPS驱动已在96运行；domain_bridge、rmw_cyclonedds_cpp已安装。GPS专用XML默认使用现场原项目路径，可用 `gps_cyclonedds_xml:=/absolute/file.xml` 覆盖。Ctrl+C会停止本入口拥有的模块，不杀其他终端或原项目进程。任一直接子进程退出会关闭整组，不自动重启；LIO启动后缺传感器会保护退出。不要同时运行旧的同名模块或第二个网页（8765端口）；先在对应旧终端Ctrl+C。没有sudo、pkill、网卡修改或串口操作。

已有部分模块在其他终端运行时，可明确关闭本入口相应模块，例如 `bash scripts/launch_real_workspace.sh with_lio:=false`。支持 with_gps_bridge、with_gnss_gate、with_web。同组统一Domain0/FastDDS，桥接进程独立使用CycloneDDS源配置。默认不读取NXK控制环境配置，防止新开终端继承运动授权。

也可直接使用：`ros2 launch d1h_nav2 real_workspace.launch.py workspace:=/workspace/nxk`。统一入口不等于传感器/融合/导航验收已完成；GPS室内无定位时仍不会绘制位置。

可选机器人控制接入见 [robot_control.md](robot_control.md)。不设置私有控制配置时仍为只读，不打开UDP控制连接；方向预览与融合初始化仍未连接。配置授权后网页提供站立/loco、确认目标导航、停止按钮；任务执行状态以后端返回为准，不以HTTP排队成功为准。

支持地图点选目标（WGS84）和拖拽/数值设置初始方向（从北顺时针）。预览POST接口校验后仅保存到内存，GET /api/state 的 operator_preview 返回记录。预览本身没有ROS目标发布或融合初始化调用，目标未检查道路可达性；不要将预览当成已下发任务。独立的控制接口需要私有配置、令牌与安全条件，见控制文档。重启清空预览。写接口只接受同源浏览器请求，仍只用于可信SSH隧道，不可公开暴露。

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

本次增加的是脚本和静态页面，原三包已构建时无需为网页重新colcon编译。需确保GPS桥接和LIO在域0；当前终端若继承了Domain96专用DDS配置，应先与正常LIO终端环境匹配。接口缺失会显示无数据，不自动启动GPS驱动或导航。启动依赖 nav2_msgs，可通过 `ros2 interface show nav2_msgs/action/NavigateToPose` 检查；此处导入接口不等于开启运动。

电脑通过上述SSH隧道访问。终端保持运行，Ctrl+C退出。该增量不会更改网卡、GPS驱动、DDS域或厂家SDK。
