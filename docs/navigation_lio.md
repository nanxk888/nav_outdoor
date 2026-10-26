# 导航专用 FAST-LIVO2 里程计副本

独立包 `src/d1h_fast_lio`，可执行文件 `lio_odometry`。
复制当前项目内 FAST-LIVO2 的算法源并独立修改；原 `fast_livo`、仿真和建图配置未改。
不是从 Orin 远程复制的精确版本，Orin 原工程和目录不受本地操作影响。
算法来源、GPLv2 许可和改动说明见包内 `UPSTREAM.md` / `LICENSE`。

## 做什么，不做什么

- 只消费 `/livox/lidar` (livox_ros_driver2/CustomMsg)、`/livox/imu`。
- 不启动驱动、相机、GLIM、RTAB-Map、RViz、GPS或控制器；没有相机订阅。
- 使用 FAST-LIVO2 原有 IMU传播、去畸变、点面匹配和滑动体素地图估计运动。
  里程计必须保留内部注册地图；“只做里程计”并非完全无地图。
- 无点云存盘、调试文件、累积轨迹或地图导出，不发布累积注册地图。
  导航滤波器订阅时按需输出 `/navigation/lio/deskewed_cloud`：仅当前去畸变扫描，坐标为lio_odom，保留与里程计一致的测量时间，发布队列2，不额外保存历史点云。见[真机导航点云](real_navigation_cloud.md)。
- 初始化时一次重力对齐；同步旋转世界位置/速度/重力相关状态协方差。
  这不提供绝对东/北航向，不等于校准本体安装角。
- 输出 `/navigation/lio/odometry`，frame=`lio_odom`，child=`lio_imu`，不发布 TF。
  时间戳取 IMU 去畸变/传播后的 `last_lio_update_time`，而不是主机发布时刻。
- 线速度转到 IMU 子坐标系；角速度为最近配对陀螺仪值减估计偏置。
  输出协方差由上游19维状态协方差和配置的陀螺仪噪声推导，包含坐标变换。
  这是模型估计值，不包含完整时间/安装标定误差，不是实测精度保证。
- 输入/状态超时、时钟倒退、输入队列溢出或无效协方差时退出，不自动重启。

启动缓冲优化：雷达订阅缓存和内部待处理容量为6帧；不是等待攒够6帧才计算。
单线程执行器每次最多执行一个回调，再尝试同步/消费，避免批量回调先灌满内部队列。
第一帧雷达到达前的有效IMU也会保留。启动各输出一次等待同步、同步成功、IMU初始化完成。
数据年龄上限仍是0.5秒；容量增大不代表允许发布0.6秒延迟的定位。溢出仍退出，
同时报告IMU队列、最新IMU时间、最老扫描起止时间，不随意丢帧继续导航。
本地合成测试可用 `scripts/test_navigation_lio_stream.py --startup-order imu-first`
或 `--startup-order lidar-first` 检查两种到达顺序；不代表真机重复启动验收。
  启动无输入3秒退出；已收到输入后超过0.5秒退出；IMU时间间隔>50ms退出。
  需要先启动传感器、保持静止，再启动本节点。
- 状态 `/navigation/lio/status` 标为 `LIO_SENSOR_TRACKING`、`base_link_ready=false`。
  退出后下游必须用状态/里程计超时判断失效，不能缓存最后一条“正常”。

本版不是完整真机定位：不输出 `base_link`、不发布 `map→odom`、不融合GPS、
不发送运动命令。没有经验证的本体安装外参前，禁止仅改 child_frame_id 冒充本体定位。
传感器安装有30°倾角时，即使世界已经Z向上，IMU姿态仍应包含真实安装倾角。

## 构建

在当前项目根目录执行（本地WSL示例）：

```bash
bash scripts/build_real_lio_odometry.sh
```

依赖ROS Humble、Livox消息、vikit_common、vikit_ros、Sophus、PCL、OpenCV等。
脚本只构建专用包，前提是依赖已安装/构建，不自动覆盖Orin现有工作空间。
Orin可单独拷贝本包到新的导航工作空间，source 已验证的 Livox/vikit 依赖环境后
执行 `colcon build --symlink-install --packages-select d1h_fast_lio`；
不要 source 已损坏的 `/workspace/fast_ws/install/setup.bash`，不要复制x86编译产物。

## 启动与检查

使用传感器已经在发布的同一ROS_DOMAIN_ID和RMW环境：

```bash
bash scripts/launch_real_lio_odometry.sh
ros2 topic echo /navigation/lio/status
ros2 topic echo /navigation/lio/odometry --once
ros2 topic hz /navigation/lio/odometry
```

也可以在独立导航工作空间source安装环境后直接运行：

```bash
ros2 launch d1h_fast_lio navigation_lio.launch.py
```

参数入口 `params_file:=/absolute/path/navigation_lio.yaml`。
初始雷达→IMU外参采用用户提供的 `[0.04165,0.02326,-0.0284]` 和单位旋转；
这是传感器间外参，不是本体外参。滤波/噪声与滑窗参数均需现场验收。
MID360 CustomMsg在此副本走AVIA=1解码分支，不能盲目改成PointCloud2的MID360=8。

原GLIM或其他里程计仍运行时，本包因无TF且使用独立话题不会抢占相同TF，
但计算负载仍会叠加；正式验收应由人工明确选择一个定位提供者。
禁止用pkill宽泛停止传感器；不要运行旧 start_mapping.py 自动清理/重启驱动。

## 尚待完成

传感器到本体的位姿/速度杆臂变换及协方差、固定odom初始方向约定、定位重启epoch管理、
现场运动精度与实时性测试、GPS＋人工初始航向全局约束。
空间滑窗不等于已测得总内存硬上限；长时间运行仍需评估体素子节点和峰值分配。
本地编译/合成输入测试不等于Orin实测或自主导航验收。

## 本地验证

```bash
bash scripts/check_navigation_lio.sh
```

包括C++速度坐标变换、协方差Jacobian/非法输入与退化点簇回归，
以及独立ROS域93中真实可执行文件的无相机合成CustomMsg/IMU流检查。
后者检查测量时间戳、传感器帧、非零协方差和输入停止后的进程退出。
这是静态无噪声几何测试，不是现场精度、动态运动或持续实时性验收。
