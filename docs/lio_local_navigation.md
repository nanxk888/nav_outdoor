# 无 GPS 小范围 LIO 导航：第一阶段

本模式不需要 GPS、OSM 配准或 Nav2 启动。复用 LIO、地面滤波、观察记忆，
新增独立本地路径跟踪器。选点仍是预览，只有点击“开始 LIO 导航（完整路径）”
才请求运动；不会自动起立、进入 loco 或打开 SDK。

## Orin 部署

先推送本地提交，再在 Orin 停止旧工作空间、拉取、构建。依赖源环境与原部署相同。

```bash
cd /workspace/nxk
bash scripts/stop_real_workspace.sh
git pull --ff-only origin main
source /opt/ros/humble/setup.bash
source /workspace/ws_livox/install/setup.bash
source /workspace/fast-livo2-deep-robotics/install/local_setup.bash
bash scripts/build_real_outdoor.sh
source install/local_setup.bash
bash scripts/launch_real_workspace.sh \
  sensor_height_m:=0.46 \
  gps_cyclonedds_xml:=/workspace/D1h/d1h-autonomy/.local/robot/perception/cyclonedds.xml \
  with_gps_driver:=false \
  with_nav:=false \
  control_config:=/root/.config/nxk/control.json
```

`with_nav:=false` 只关闭地理 Nav2 启动，不关闭本地执行器；`with_memory:=false`
则会关闭记忆和本地执行器。外部雷达驱动和 D1 接收器/过滤器须自行保持运行。
地理参考状态中的“等待GPS/方向登记”不阻塞此独立本地模式。
0.46m 是初始 IMU 实测高度，不是所有站立姿态都适用。机器人有线地址和密钥仍需有效。

## 实机测试顺序

1. 浏览器 Ctrl+F5，现场有人看护、硬件急停可用，在平整宽阔区域测试。
2. 人工站稳、打开 SDK 并进入 loco，查看网页当前机器人反馈有效。
3. 点击“无GPS：点选LIO相对目标”，选择约1～2米、已经观察为空闲的目标。
4. 等待完整候选路径，`partial:false`；本地执行状态应为 READY，或已停止但显示完整路径输入就绪。部分黄色前沿不能执行。
5. 点击独立的“开始 LIO 导航（完整路径）”，不要点击 OSM 卡片的“开始导航”。
6. 完成后显示 STOPPED / LIO local SUCCEEDED，发送停止、撤销接收端运动会话。
   “停止 / 取消任务”按钮对本模式同样有效。停止后不会自动恢复，必须重新显式启动。

位置目标距离限定0.15～2.5米，剩余路径长度不超过6米，单次任务最多60秒。
到达位置容差0.15米，不要求终点朝向。默认前进上限0.15m/s，转向上限0.35rad/s，禁止倒车。
机头采用已现场确认的180°安装修正；LIO屏幕初始朝上只是显示旋转，不参与控制。

真机候选规划、LIO执行校验和真机Nav2占地半径统一为实测0.25米（直径0.5米）。
地面滤波健康状态以10Hz发布（原2Hz与0.5秒超时冲突）；执行端仍要求状态及
测量有效年龄小于0.5秒，不用更长超时掩盖断流。错误分别报告状态过期、滤波拒绝
和测量过期及实际年龄。状态发布更快不代表点云处理频率变高。
室外navigation_reference也使用同一个require_ground检查，并把接收后经过时间
计入测量年龄，向网页传递实际失败原因。GPS/朝向登记仍是室外导航的独立前提。

前端“导航速度（LIO／室外共用）”通过speed_limits请求更新最终UDP输出限幅。
“参数与状态”为单项展开的二级分类：机器人状态、导航速度、GPS融合导航、无GPS导航、
机器人控制、loco姿态控制、数据明细。速度为第二项；无GPS导航集中点选、开始、
取消操作。取消调用通用stop，不自动落下、不清空目标；底部通用停止始终保留。
GPS融合导航合并原地图对齐与目标选择/开始导航，地图左下任务卡已删除。
机器人FSM和控制流程集中在机器人控制；地图工具左侧竖排，主视图标题缩小置右上
（小地图左侧避让，窄屏小地图下移）。名称调整不代表新增连续GPS融合算法。
默认前进0.15m/s、转向0.35rad/s，上限参数分别可在0.1～1m/s、0.1～0.5rad/s范围调整。
LIO启动及会话心跳携带速度上限，控制器直接按它生成速度并检查对应运动轨迹；
距离目标较近时按0.5m/s²假设制动减速，转弯也会减速，设置上限不是强制匀速。
UDP速度校验同步允许1m/s、0.5rad/s，必须更新D1H上的robot_wire.py并重启接收器。
接收器原有逐步加速、断流停止机制保留。1m/s尚未实机验证，不代表已验证制动能力。
两种导航最终下发均按同一比例限幅v和w，保持曲率。室外新增nav2_speed_sync节点，
接收相同设置，更新RPP的desired_linear_vel、rotate_to_heading_angular_vel，以及
velocity_smoother的max_velocity/min_velocity，等待参数服务确认并回读一致才报告生效。
室外导航启动前必须有当前设置的确认；运动中修改保留最终限幅，参数拒绝、同步
失联或更新超时则停止，不自动恢复。无GPS执行不依赖此节点。设置不启动任务，
仅当前进程有效，重启恢复默认0.15m/s和0.35rad/s。
栅格检查使用圆与栅格的实际相交关系，不再额外增加离散化扩张余量；圆内涉及的
未知格仍不视为空闲。有GPS真机Nav2也使用0.25米半径、零footprint_padding，
障碍膨胀代价层独立保留。OSM路网本身只有道路拓扑，不代表道路宽度已测量。
完整候选路径会保留，但每次更新用最新地图检查剩余路径；受阻或未知即失效。
执行时使用启动时接受的完整路径，不因候选重规划短暂失败而停止，仍持续检查
实际剩余路径、前视连线及指令运动轨迹。失败状态显示具体规划原因和起点未知/障碍格计数。
观察空间A*路径经过可通行直连简化和两轮Chaikin拐角平滑；只有整个圆形占地
仍处于已观察空闲区时才接受平滑，否则保留原绕障折线。起终点保持不变，路径加密
供前视跟踪使用。这不是切换为仿真的Nav2规划链路；无GPS仍可独立运行。
窄通道即使看起来有线也可能
因占用范围/未知格拒绝执行。射线空闲仅是平面可见性证据，不保证无坑、台阶或地面可通行。

## 执行链路和停止条件

`/navigation/memory/status` 的完整路径 + `/navigation/memory/grid` + 原生 LIO 姿态
→ `local_navigation` 水平路径跟踪和扫掠占用检查
→ `/navigation/local_execution/status` 的带 run_id 速度建议
→ 网页协调器的新接收会话 → 认证 UDP → D1 接收器 → UserCommand。

本地执行不消费 Nav2 速度。协调器按模式隔离输入，检查实时 SDK/loco、浏览器心跳、
执行器状态和指令新鲜度；接收器自身看门狗仍保留。本地执行器另有0.3秒会话租约，
协调器停止续租即终止任务。跟踪器校验整个剩余路径、前瞻连接段
和一秒指令运动弧线。转向同样要求圆形占用范围已知为空闲。

目标或记忆 epoch 改变、LIO失效/跳变、点云地面状态失效、地图过期、路径受阻、偏离路径、
SDK/FSM/接收连接失效、浏览器心跳丢失都会拒绝或停止。恢复输入不会恢复旧任务。
第一阶段固定执行点击时接受的完整路径；出现障碍停止，不自动绕行或沿部分前沿探索。

软件零速度不能保证机械瞬时停稳，不能代替独立硬件急停或厂家控制器自身失联保护。

## 本地验证，不连接机器人

```bash
python3 scripts/test_local_navigation.py
ROS_DOMAIN_ID=94 ROS_LOCALHOST_ONLY=1 python3 scripts/test_real_navigation_execution_ros.py --local
```

隔离 ROS 域、合成传感器和回环 UDP 接收器用于验证接线、限速及停止行为；没有验证
真实整机跟踪精度、转向符号、地形或制动距离。首次实机应先核对小幅前进/转向方向。
