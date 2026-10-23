# Orin → D1 网线控制接入（待实机验收）

## 边界

依据用户提供 D1_receiver 的接口结构独立实现：D1端发布 `/$ROBOT_NS/command/cmd_twist`（TwistStamped）与 `command/cmd_key`（String），读取 `rl_controller/fsm`（String）。没有复制原SDK源码，不修改 D:/D1_receiver。原压缩包二进制UDP只有六个float速度和CRC，不能传状态机或反馈；本实现使用专用配对的JSON/HMAC协议，不兼容原S1接收器。两种接收器不得同时占用控制权。

本地代码实现不代表硬件安全验收。站立、loco稳定反馈值、SDK模式、双轮足构型、命令话题兼容性均必须现场确认。不会自动切构型或开启SDK模式；不要在无人照看时启动。官方遥控/键盘/旧接收器与本接收器必须只有一个命令所有者。

## 数据路径

浏览器 → 同源HTTP＋Bearer令牌 → Orin控制队列 → 20Hz UDP → D1接收器 → 厂家ROS接口。Orin留在导航Domain 0，D1接收器在厂家实际ROS域（交付常见42，以实测为准）；无需将厂家TF或控制域桥接到导航域。HTTP仅回环监听，使用SSH隧道。网线、IP与DDS配置需现场配置，本程序不改网卡。

POST /api/control/start 请求：授权握手 → transform_up → 等待配置的实际“站立完成”反馈 → loco → 实际反馈loco → READY、零速度。只配置 transient transform_up 而无厂家完成语义不能视为站立完成。`stand_ready_states` 默认空，确认真实状态语义后才填写。转换10秒超时停止，不按固定时间猜测站立成功。

POST /api/control/navigate 使用后端保存的GPS目标，按当前OSM米制近似投影发送 Nav2 `/navigate_to_pose` action。只接受READY状态；拒绝/超时、结束/取消都归零，不自动恢复。UI需要单独确认导航，不会因误点地图立即运动。初始方向仍是预览，尚未修改融合定位。

## 部署前置检查（只读）

在D1加载厂家环境与namespace后，确认下列接口类型/状态与SDK实际行为一致：

```bash
source /opt/ros/humble/setup.bash
source /opt/d1_ros2/setup.bash
source /opt/d1_ros2/namespace.sh
printf '%s\n' "$ROBOT_NS" "$ROS_DOMAIN_ID"
ros2 topic type /$ROBOT_NS/command/cmd_twist
ros2 topic type /$ROBOT_NS/command/cmd_key
ros2 topic echo /$ROBOT_NS/rl_controller/fsm --once
```

检查接收端与Orin时钟偏差远小于250ms；本协议超过250ms拒绝。部署NTP/chrony局域网校时并验证，不能仅假设日期一致。D1与Orin经网线同网段；默认示例地址不是已验证现场地址。

## 文件与密钥

在D1放置 `scripts/robot_udp_receiver.py` 和同目录 `robot_wire.py`。在两端使用相同、至少32字节的随机私钥（例如 `openssl rand -hex 32`），通过受控渠道配置，权限600；私钥不入Git。网页令牌使用另外一个随机值，通过 `NXK_WEB_TOKEN` 设置，不放URL，不复用UDP私钥。

首次D1运行不带授权，确认UDP/FSM反馈但不会发布机器人命令：

```bash
python3 robot_udp_receiver.py --bind D1实际有线IP --peer Orin实际有线IP --key-file /private/udp.key
```

现场验收后才可增加 `--authorize-motion`。此时接收端会持续发布零速度，已经取得命令控制权；必须先排除其他发布者，并确认硬件急停与安全操作条件。不要将它配置为开机自动授权站立。

复制 map/robot_control.example.json 为私有本地配置，填写实际IP/密钥路径/完成状态和与规划器完全相同的地图原点。只有现场审核后才设置 hardware_authorized、interfaces_verified、survey_confirmed。不要修改后提交示例文件。

```bash
# Orin加载导航ROS环境后
export NXK_CONTROL_CONFIG=/absolute/private/control.json
# 在终端安全设置NXK_WEB_TOKEN，不把令牌提交Git或贴到日志
python3 scripts/navigation_web.py
```

网页输入令牌 → 启动 → 观察STANDING/ENTERING_LOCO/READY → 地图选目标 → 确认导航。停止取消任务，不发送transform_down；站立平衡由厂家维持。

## 仍缺失、必须接通的安全监督输入

导航非零速度要求 `/real_navigation/safety_status` String JSON：`navigation_ready:true, base_link_ready:true, map_alignment_verified:true, stamp:<ROS测量秒>`。消息和测量年龄均小于0.5s。此输入必须来自独立、真实的定位/感知监督器，检查融合、本体TF、障碍输入、路网配置。当前项目尚未实现这个完整监督器及GPS-LIO融合，因此真实导航默认被阻止。禁止手工循环发布true绕过。

这不是仅靠新增控制代码就已打通全部真机导航的声明。实际路径可达性由审核过的OSM规划器检查；网页预览不代表道路可达。当前最大前进0.3m/s、转向0.5rad/s，禁止倒车；规划器不得发布更大的候选速度，否则停止。

## 超时与限制

接收端200ms无新有效指令或FSM反馈超过1秒：归零、撤销会话，不自动恢复。发送端命令200ms过期、机器人反馈300ms过期、浏览器心跳2秒过期：取消任务并停止。HMAC校验来源、启动随机nonce和会话序号防旧会话/乱序重放；不提供加密，限可信有线网络及防火墙使用。上位ROS候选Twist无Header，无法检测消息生成前的未知延迟；需验收DDS延迟与命令唯一发布者。

接收器进程崩溃/断电时，软件看门狗也会消失，必须验证厂家控制器自身的指令超时停车或独立硬件保护。软件零速度不等于实测已停止，更不等于急停。

## 本地验证

`python scripts/test_robot_wire.py` 测试协议签名、时效、数值和速度边界；`python scripts/test_navigation_control.py` 使用假ROS/UDP对象验证默认禁用、零速保持、超时、缺融合安全输入、禁止自动恢复。不打开真实网络，不控制实机。还需双方完整假厂家接口状态机、断网、取消/晚到action回调测试，以及D1只读、零速度、低速现场验收。未经这些验收不得认定可用。

接收端以50ms周期限制非零输出变化，每次前进速度变化不超过0.015m/s、转向变化不超过0.04rad/s；安全停止直接发送零，不代表真实制动响应已标定。接收端崩溃时不能依赖这个软件限制。
