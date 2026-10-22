# LC29H 真机 GPS 接入：第一阶段

本配置消费现有 Orin 驱动，不替换 `gnss_receiver.py`，不重复打开串口。
硬件为 LC29H(DA) + CP2102N，115200/8N1；串口设备路径由原驱动维护。
目前不下发模块配置、不注入 RTCM、不进行接收机时间校时。
不改动任何仿真定位配置。

## 数据流

`/hardware/gnss/fix` (sensor_msgs/NavSatFix，frame=gps_antenna，主机接收时间)
→ real_gnss_gate → `/localization/gnss/fix_filtered`。

`/localization/gnss/status` 是独立 GNSS 健康状态 JSON；GNSS_READY 仅表示 GPS 样本通过门控，
不是融合定位就绪。因此故意不发布 `/localization/status`，不触发机器人运动许可。
不发布 TF、机器人里程计或速度。原始 NMEA、接收机诊断和未认可时间参考话题保留原状。

检查：有效经纬度/定位状态、gps_antenna frame、ROS 时间戳与主机接收超时、
重复/倒序时间戳、有限/对称/半正定协方差、水平不确定度上限、位置跳变。
主机接收时间不等于真实测量时刻：本门控无法检测串口之前的未知延迟。
不根据 HDOP 直接填写米制误差，不根据 NavSatStatus 判定 RTK 固定解。
RTK 质量后续应从原始 GGA Quality 或 `/lc29h_gnss/health` 中确认，
具体诊断键名称尚未提供，本版不猜测这些键、不提供 RTK-only 门控。

默认 1.5 秒超时，需按实测频率和延迟调整。
未知协方差默认拒绝；如明确临时允许，设置 allow_unknown_covariance=true，
使用 5m 标准差近似（25m² 方差），并将输出标为 APPROXIMATED。
有效协方差同样标为近似，默认标准差不小于 1m；这些是保守软件初值，不是硬件实测精度。
全零、非正定或无效的“已知”协方差拒绝，不偷换成高置信定位。
水平标准差大于 10m 默认拒绝。
跳变阈值是 10m + 2m/s × 距上次通过样本的时间；它是粗筛，不是定位滤波，
没有对 GPS 做平均/平滑来伪造精度。

## 构建与运行

先正常启动现有 GNSS 驱动，再运行：

```bash
cd /path/to/nav_n
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select d1h_global_localization
source install/setup.bash
bash scripts/launch_real_gnss.sh
```

使用现场配置：

```bash
bash scripts/launch_real_gnss.sh params_file:=/absolute/path/real_lc29h_gnss.yaml
ros2 topic echo /localization/gnss/status
ros2 topic echo /localization/gnss/fix_filtered --once
```

`config/real_lc29h_gnss.yaml` 是模板。Orin 的安装、路径和 ROS 版本尚待现场确认。
遇到 unknown_covariance，请先发原始 fix 消息和驱动配置，不要直接删除检查。

## 可选 OSM 坐标预览，不是机器人定位

仅在坐标原点经现场确认后，将配置中的 datum_latitude_deg / datum_longitude_deg
设成与真机 OSM JSON 完全一致，并设置 datum_confirmed=true、enable_enu_preview=true。
此时发布 `/localization/gnss_antenna_enu` (geometry_msgs/PointStamped，map frame)。
它使用与现有真机道路规划相同的局部东/北近似投影，z=0 只表示二维显示，
不是接收机的高度，也没有补偿天线到机器人本体的杆臂。
不要将它直接冒充 base_link 位置，更不要混接 UTM/自动首点原点的融合输出。

## 下一阶段仍缺少

FAST-LIVO2 里程计话题/坐标系/协方差与 TF 发布配置；
base_link→gps_antenna 外参；地图原点和轴向；可用的绝对航向或人工初始航向；
定位健康与失效停车接口；实测 GPS 频率/静态漂移/延迟与 RTK 差分状态。
RMC/VTG 航迹角不是静止机器人的朝向，本接收机当前也没有单独发布它。

收齐后再实现 LIO 连续 odom + GPS 全局位置约束的融合和 map→odom。
不要启动仿真的 Gazebo 参考定位或直接套用旧仿真 EKF/NavSat 配置。
