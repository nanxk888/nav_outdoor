# 真机专用 OSM 道路全局规划（离线开发版，尚未真机验收）

此入口不改变任何仿真启动文件、场景或配置。只启动规划，不启动 Gazebo、
仿真参考定位、模拟地面过滤器、机器人硬件或电机命令桥。

## 算法与接口

- 全局：OSM 原始共享节点道路图 + A*（欧氏距离启发函数）。
  起终点投影到道路线段，连接段不得越过已确认道路边界或占用区域；
  不存在可达道路时返回失败，不退回穿越任意空地的栅格规划。
- 局部：保留五米滚动目标、5 Hz 评估、Smac 栅格规划和 RPP，
  以及原有安全前缀续行/后台重规划逻辑。局部增加道路边界静态层，禁止绕到道路之外。
- 全局仍约每秒评估，安全路线沿用；雷达障碍记忆保留原来的 120 秒。
  全局后端基于更新后的 global_costmap 检查路段，受地图更新/通信周期影响，非零延迟。
- 显式道路白名单；禁止台阶、施工、干道、受限道路，以及本版尚不支持的桥梁/隧道/不同层级。
  保守遵守 oneway；未实现转向限制关系和按道路分别设置宽度。
- 一个已占用的道路中心线段会被整体排除，可能过于保守。
  宽路局部绕障和整段封路判定后续应升级为道路横断面剩余通行宽度评估。

## 真机必须先提供

1. 经外参和时间同步校验的 `map -> odom -> base_link -> lidar_link` TF。
   map 为固定东向 X、北向 Y 的米制坐标，与配置 datum 一致。
   LIO 输出连续 odom，GPS 约束 map->odom；本入口不实现该定位融合。
2. `/odometry/local`，以及采集时间真实、已去地面/去自体的
   `/livox/lidar/pointcloud_nav` 和 `/livox/lidar/pointcloud_clear`。
   不要套用仿真固定地面 z=0 的过滤器处理真实坡道。
3. 与 OSM 对齐的实测坐标。这里投影与 gps_goal_to_enu 使用相同局部近似，
   如果定位融合输出 UTM 或 navsat_transform 的其他坐标，必须先统一，不能直接接上。
4. 急停、定位失效停车、传感器失效停车和硬件命令接口的真机适配验收。

## 配置及启动

以 `map/real_osm_profile.example.json` 为参考另建实际部署 JSON：

- `osm_file`：Orin 上地图的绝对路径。
- `datum_*`：与真实定位融合完全一致的 WGS84 原点；示例值不是校准结果。
- `approved_way_ids`：现场核验可通行的 OSM way ID，必须显式填写。
- `minimum_road_width_m`：白名单全部道路共同满足的实测最小净宽，不是占用膨胀半径。
  当前保守道路边界以中心线对称生成，应确认中心线偏差、路沿和建筑不会侵入。
- `survey_confirmed`：完成上述核验后才设为 true；示例默认拒绝启动。

```bash
cd /mnt/e/nav_n
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select d1h_nav2
source install/setup.bash
bash scripts/launch_real_osm_navigation.sh /absolute/path/to/reviewed_profile.json
```

真机预先启动自身传感器与定位，不要启动 `launch_global_localization.sh`。
规划的速度命令只发布到 `/real_navigation/cmd_vel_candidate`，不会连接 `/command/cmd_twist`。
这是为了先验收定位对齐、道路约束、障碍记忆和重规划，再单独接入带急停/健康监控的真机桥。
RViz 设置 Fixed Frame=map，检查 `/map` 和 `/navigation/display_plan`。

本版道路栅格 0.5m、最多两千万格；整校园图对 Orin 有显著内存和更新开销，
初次验收仅选一小片连通道路。尚未完成实时性、实车避障与定位失效测试，不能直接无人运行。
OSM 数据使用时保留 OpenStreetMap contributors 署名并遵循原数据 ODbL 许可。

## 已完成的验证

`python3 scripts/test_osm_roads.py`：六项离线回归，覆盖线段接入、沿路搜索、
封路改道、路外终点拒绝、不可达拒绝、坐标投影。
`ROS_DOMAIN_ID=89 python3 scripts/test_osm_road_ros.py`：隔离 ROS 域动作接口测试，
覆盖成功路径、缺失代价地图拒绝、路外目标拒绝。另通过 Humble 构建和启动参数检查。
这些不是实车闭环验收，也没有用模拟定位代替真实 GPS/LIO 做“真机成功”判断。
