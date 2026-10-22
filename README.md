# nav_outdoor — D1H 真机室外导航

面向 NVIDIA AGX Orin / Ubuntu 22.04 / ROS 2 Humble：复用 FAST-LIVO2 的雷达＋IMU里程计，接入 LC29H GPS，并在 OSM 道路网络上全局选路、利用雷达进行局部避障。

**当前是分模块部署与验证版本，不是已经完成真机自主导航的交付版本。**
GPS＋LIO全局融合、本体外参适配、真实地面处理及安全控制接入尚未完成。
规划入口只发布候选速度，不连接电机。禁止直接用于无人值守运行。

## 部署文档

**从这里开始：[Orin 完整部署与验收说明](docs/deploy_orin.md)**。

| 模块 | 文档 | 当前状态 |
|---|---|---|
| 专用纯LIO里程计 | [navigation_lio.md](docs/navigation_lio.md) | 本地构建、数学回归及静态合成输入通过；未在Orin实测 |
| LC29H GPS接入 | [real_gnss.md](docs/real_gnss.md) | 质量门控已实现；非GPS＋LIO融合器 |
| OSM道路规划 | [real_osm_navigation.md](docs/real_osm_navigation.md) | 道路图A*＋五米局部Smac规划＋RPP，尚未真机闭环验收 |

## 仓库范围

- `src/d1h_fast_lio`：独立的 FAST-LIVO2 导航里程计副本；不开相机、不存全局地图，不发布TF。
- `src/d1h_global_localization`：真机GNSS质量门控及共享定位工具。
- `src/d1h_nav2`：OSM道路规划、局部规划、路径缓存、障碍记忆和RPP配置。
- `map/xd_map.osm`：西电南校区及周边OSM数据；示例配置默认拒绝启用道路规划。
- `scripts`：构建、分模块启动和回归检查。

两个Python包保留了一些与原工作空间共享的历史仿真/室内辅助模块，以保持包结构完整；
**这些不是本仓库的部署入口，也不包含运行仿真所需的机器人、场景和模型资产。**
请只使用文档列出的 `real_*` / `navigation_lio` 入口。

`.gitignore` 采用真机发布白名单：本地仿真、模型、点云和编译目录被排除而非删除。
后续新增部署包、脚本或文档时，需要同步更新白名单。

## 获取与构建

在Orin上克隆源码并本机编译，不能复制Windows/WSL的二进制产物：

```bash
cd /workspace
git clone https://github.com/nanxk888/nav_outdoor.git
cd nav_outdoor
source /opt/ros/humble/setup.bash
source /workspace/ws_livox/install/setup.bash
# 仅复用已有、可正常加载的vikit等依赖；不启动原建图程序。
source /workspace/fast-livo2-deep-robotics/install/local_setup.bash
bash scripts/build_real_outdoor.sh
source install/local_setup.bash
```

以上两个依赖目录来自当前现场环境，不存在时先按完整部署文档准备依赖。
不要加载已损坏的 `/workspace/fast_ws/install/setup.bash`。

## 分模块启动

传感器由现有驱动独立启动。本仓库不会清理进程、改网卡或启动机器人运动。
三个命令分别使用独立终端、加载同一ROS通信环境；它们**不是**已经接通的自主导航四终端流程。

```bash
# 雷达＋IMU里程计：输出IMU位置，不冒充base_link。
bash scripts/launch_real_lio_odometry.sh

# 已启动 /hardware/gnss/fix 驱动后：
bash scripts/launch_real_gnss.sh

# 只有本体定位、点云过滤、TF及道路配置全部通过验收后才启动规划：
bash scripts/launch_real_osm_navigation.sh /absolute/path/to/reviewed_profile.json
```

当前里程计输出 `/navigation/lio/odometry` 的 `lio_odom→lio_imu`；规划需要
`/odometry/local` 和 `map→odom→base_link`。**中间适配尚未实现，禁止仅改话题/帧名字冒充融合。**

## 回归检查

统一运行（需已构建并加载外部依赖）：

```bash
bash scripts/check_real_outdoor.sh
```

也可以分模块运行：

```bash
python3 scripts/test_gnss_quality.py
python3 scripts/test_osm_roads.py
bash scripts/check_navigation_lio.sh
# 以下应在隔离的ROS域运行，不向真实传感器域发布测试数据：
ROS_DOMAIN_ID=89 python3 scripts/test_osm_road_ros.py
ROS_DOMAIN_ID=90 python3 scripts/test_real_gnss_ros.py
```

执行ROS测试前需 `source install/local_setup.bash`。测试通过不代表真机精度、避障安全性或长期实时性已经验收。

## 许可与数据

见 [NOTICE.md](NOTICE.md)。FAST-LIVO2副本保留GPLv2许可；OSM数据保留OpenStreetMap contributors署名和ODbL说明。
