# 真机导航点云接入（局部平整地面试验版）

本模块补接真实LIO当前扫描到两个导航点云话题，不启动Nav2、不下发运动、不宣称完整避障已验收。

## 链路与边界

专用LIO在有效匹配更新后，按需输出 `/navigation/lio/deskewed_cloud`。它是当前去畸变扫描，经当前LIO估计变换到 `lio_odom`，使用与里程计一致的测量时间，不是累积地图、不重打当前时间、不冒充base_link。只有订阅者存在时转换，发布队列为2。

`real_cloud_filter` 仅处理与 `/navigation/lio/odometry` 时间戳及坐标系完全一致、新鲜度不超过0.5秒的扫描。缓存最多10个里程计、1帧待配对扫描；扫描最多10万点，地面拟合采样最多2000点、64次候选。

必须提供 **初始化姿态下Livox IMU到地面的实测高度**（米），不是初始世界z、不是base_link原点高度。用户反馈低位约0.46米：只在机器人确实处于该低位时使用0.46初始化，不能把它当作站高后的高度。默认0拒绝生成有效点云，不能照搬仿真值。首次以传感器位置减实测高度约束地面附近候选，之后在连续重力对齐LIO坐标中跟踪固定世界地面、重新拟合，不随站高把地面抬高；状态报告估计的当前IMU离地高度。每次必须有新的有效平面才输出，不重放旧点云。检测到时间/坐标或大幅位置不连续时清除参考，要求重新处于已知高度初始化；小幅LIO重置可能无法仅凭此检测，仍需后续定位监督。当前仅适用于局部近似平面的初次试验；坡度上限20°、高度带±15cm不等于已经验收坡道通行。

- `/livox/lidar/pointcloud_nav`：距离IMU约0.6–5米、相对估计地面3厘米以上、0.9米以下的障碍点。
- `/livox/lidar/pointcloud_clear`：约0.6–6米内，地面附近到0.9米高度范围的真实回波端点。不是填充的自由空间、没有回波处不虚构点，不使用屋顶回波去清除低障碍。
- `/navigation/cloud_filter/status`：质量与失败原因。地面不可见、支持不足、错误高度、时间/坐标不符时不发布导航/清除结果；不伪造空扫描表示安全。

网页LIO视图以橙色点显示障碍点预览，不显示地面清除点，不代表Nav2代价地图。需网关收到上述导航点云和过滤状态，并与LIO输出帧一致；过期或失效时隐藏。展示抽样最多2000点/帧，仅用于观察，不改变规划器的真实输入。

输出保留原时间和LIO原生坐标系，不能直接把名字改成odom/map。Nav2仍需真实TF和雷达射线原点外参。0.6米球形近场排除只是保守试验策略，**不是经过完整机器人包络验证的自体过滤**；近场盲区、3厘米阈值在传感器噪声下的可靠性、台阶/坑、植被、多层地面均未实机验收。不能用于替代安全监督或直接开放自主行走。起落时遮挡/形变可能使地面拟合失败，此时无有效点云输出；地面跟踪不是起立完成检测。

## Orin更新

```bash
cd /workspace/nxk
source /opt/ros/humble/setup.bash
source /workspace/ws_livox/install/setup.bash
source /workspace/fast-livo2-deep-robotics/install/local_setup.bash
source install/local_setup.bash
bash scripts/build_real_outdoor.sh
source install/local_setup.bash
```

先停止旧的统一工作空间启动程序（不要并行启动第二份LIO）。统一入口现默认开启过滤器并传入低位实测高度0.46米；若初始化姿态不同，必须覆盖此高度。过滤节点自身默认0仍拒绝生成有效点云。使用以下参数启动，替换文字占位值：

```bash
bash scripts/launch_real_workspace.sh \
  with_nav_cloud:=true sensor_height_m:=实际IMU离地高度米数 \
  control_config:=/root/.config/nxk/control.json
```

不带control_config即可进行只读点云验收。滤波器在统一入口内使用相同Domain0/CycloneDDS环境。另一终端检查时也必须加载相同DDS环境，不能只source工作空间就假定环境一致。

检查顺序：`/navigation/lio/status` 新鲜 → `/navigation/lio/deskewed_cloud` 有真实扫描 → `/navigation/cloud_filter/status` ready → 两个输出的帧、时间戳、频率、地面与障碍显示。先停稳观测，使用已知高度的障碍验证保留/清除，不发送行走指令。

## 本地回归

```bash
python3 scripts/test_real_ground.py
# 已安装ROS接口依赖，使用隔离域，绝不对真实传感器域发布fixture
ROS_DOMAIN_ID=94 python3 scripts/test_real_cloud_ros.py
bash scripts/check_navigation_lio.sh --stream-only
```

合成测试不代表真机点云质量、地面识别、外参或避障安全验收。
