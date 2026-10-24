# Orin → D1 网线控制部署与分阶段验收

当前是待实机验收的接入代码，不是已完成安全认证的控制系统。默认不授权运动，不自动打开 SDK 模式，不自动起立。测试时须有人现场看护，确认急停有效、场地净空和厂家规定的支撑条件。

## 接口与数据链路

Orin（192.168.19.91）产生指令 → UDP 10099 → D1（192.168.19.90）接收器 → 本机 `/d13007925/command/user_command`，类型 `ddt_msgs/msg/UserCommand`。接收器读取实际 FSM、SDK 开关和双轮足构型。SSH 只用于部署文件，不承担运行时运动指令传输。

支持 `transform_up`、`loco`、`transform_down`、前进/转向和静止时的 roll/pitch 姿态控制。使用官方统一 UserCommand 接口，不使用旧版 cmd_key/cmd_twist。参考：<https://github.com/DDTRobot/D1-ROS2-SDK-Demo>。

参考了用户提供的 D1_receiver 通信结构，但未修改或复制其源文件。本接入使用配套 **protocol 2 JSON/HMAC**，不能与原二进制 UDP 接收器或旧版发送端混用。必须同时更新两端。Orin 保持导航域0，D1 接收器使用本机厂家域42，无需把控制域桥接给导航域。

## 1. Orin 准备文件与密钥

```bash
cd /workspace/nxk
git pull --ff-only
mkdir -p /root/.config/nxk
umask 077
# 仅第一次生成；已有配对密钥不要覆盖。
test -f /root/.config/nxk/udp.key || openssl rand -hex 32 -out /root/.config/nxk/udp.key
ssh robot@192.168.19.90 'mkdir -p /home/robot/nxk_receiver'
scp scripts/robot_udp_receiver.py scripts/robot_wire.py scripts/robot_policy.py robot@192.168.19.90:/home/robot/nxk_receiver/
scp /root/.config/nxk/udp.key robot@192.168.19.90:/home/robot/nxk_receiver/udp.key
ssh robot@192.168.19.90 'chmod 700 /home/robot/nxk_receiver; chmod 600 /home/robot/nxk_receiver/udp.key'
```

密钥不提交 Git、不贴到日志。协议校验两端主机时间，偏差必须远小于250ms；先配置并验证局域网 chrony/NTP，再启动 LIO。不要在运行中的 LIO 上跳变系统时间。UDP 仅限可信有线网络，并限制来源IP/端口；HMAC 不提供加密。

## 2. D1 先启动只读接收器

在 **D1 本机终端**加载已验证的厂家环境：

```bash
source /opt/ros/humble/setup.bash
source /opt/d1_ros2/setup.bash
source /opt/d1_ros2/namespace.sh
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
echo "$ROBOT_NS"  # 当前实测 d13007925
ros2 interface show ddt_msgs/msg/UserCommand
ros2 service call /$ROBOT_NS/command/get_controller_status std_srvs/srv/Trigger '{}'
ros2 param get /$ROBOT_NS/teleop_command use_sdk
ros2 topic echo /$ROBOT_NS/rl_controller/fsm --once
cd /home/robot/nxk_receiver
python3 robot_udp_receiver.py --bind 192.168.19.90 --peer 192.168.19.91 --key-file /home/robot/nxk_receiver/udp.key
```

此模式不发布控制命令。在 **Orin 另一个终端**：

```bash
cd /workspace/nxk
python3 scripts/robot_udp_test.py --key-file /root/.config/nxk/udp.key
```

应得到签名状态回复，包括 `fsm`、`sdk_enabled`、`biped`、`fsm_fresh`、`interfaces_fresh`。`authorized:false` 此时符合预期。若超时，先查时间偏差、IP、防火墙和接收器错误，不发送动作试探。

## 3. 现场授权与动作分项测试

先停止官方键盘控制、旧接收器等其他指令发布者。按厂家流程由操作员开启 SDK 模式；程序不会代开。确认构型为 biped、FSM反馈持续更新。接收器要求 FSM 新鲜度小于1秒、接口状态小于2秒，不满足会撤销会话。

在 D1 停止只读接收器，再启动：

```bash
python3 robot_udp_receiver.py --bind 192.168.19.90 --peer 192.168.19.91 --key-file /home/robot/nxk_receiver/udp.key --authorize-motion --allow-pose
```

**这一步即开始20Hz发布零速度/当前姿态命令，已占用控制通道。** 不要同时运行其他控制器。初始化姿态为零 roll/pitch；即使尚未发送起立请求，也须确认本机现有姿态和安全条件。不要配置无人看护自动启动。

以下命令在 Orin **逐条执行，每一步现场确认后再进行下一步**；并非一键连续动作脚本：

```bash
# idle → 请求起立；默认保持通信10秒，结束停止会话，不自动落下。
python3 scripts/robot_udp_test.py --key-file /root/.config/nxk/udp.key --action stand --authorize-motion

# 现场确认站立完成、稳定，再明确进入 loco。
python3 scripts/robot_udp_test.py --key-file /root/.config/nxk/udp.key --action loco --authorize-motion --confirm-stationary

# loco 且实际停稳：先小角度测试，例如俯仰5°。
python3 scripts/robot_udp_test.py --key-file /root/.config/nxk/udp.key --action pose --pitch-deg 5 --authorize-motion --confirm-stationary

# 恢复中立姿态，确认实际恢复后再落下。
python3 scripts/robot_udp_test.py --key-file /root/.config/nxk/udp.key --action pose --roll-deg 0 --pitch-deg 0 --authorize-motion --confirm-stationary
python3 scripts/robot_udp_test.py --key-file /root/.config/nxk/udp.key --action lower --authorize-motion --confirm-stationary

# 发送软件零速度/撤销会话，不等于硬件急停，不自动落下。
python3 scripts/robot_udp_test.py --key-file /root/.config/nxk/udp.key --action stop
```

FSM请求按操作ID只下发一次，持续心跳不会反复触发起立。CLI 不包含非零行走测试，需完成前述验收后另行进行低速闭环验证。命令已接收/FSM为loco，不代表实际姿态、站立或停车已安全完成。

姿态单位：CLI为度，协议为弧度，接收端转为四元数。限制 roll±0.2rad、pitch±0.4rad，50ms内角度命令变化不超过0.025rad。仅允许loco、零速度保持至少0.5秒时设置。停止会话保持最后姿态，不擅自回正；非中立姿态禁止开始导航或落下，须显式回正。

**高度默认禁止。** 厂家新版文档说明位置暂不可控，原接收器参数不能证明当前固件支持。只有厂家确认适用于本机、现场单独验收后，才可给接收器增加 `--allow-height`，发送器使用 `--height-m`（0.1–0.3m）。首次高度指令没有实际高度反馈，不能保证从实际高度平滑过渡；本轮不建议启用。状态中的 pose/pose_target 是输出/目标命令，不是机身实测姿态。

## 4. 接入网页（独立显式授权）

本次修改了统一launch，Orin上先加载原有依赖并重新安装该包（不必重新编译LIO）：

```bash
cd /workspace/nxk
source /opt/ros/humble/setup.bash
source install/local_setup.bash
colcon build --symlink-install --packages-select d1h_nav2
source install/local_setup.bash
```

先完成CLI分项验收。复制示例到私有文件，不修改仓库示例默认禁用项：

```bash
cp map/robot_control.example.json /root/.config/nxk/control.json
chmod 600 /root/.config/nxk/control.json
```

人工审核填写 key_file、实际IP后，只有已完成现场验收才将 hardware_authorized、interfaces_verified 设为 true。`stand_ready_states` 必须填写厂家确认的“起立完成”反馈，不能把瞬态 transform_up 猜作完成。默认空列表时，网页不会从idle自动起立；可通过前述CLI确认进入loco后，再点击启动接管零速度待命。

survey_confirmed 在仅测试站立/姿态时可保留 false；导航仍会被阻止。地图原点、GPS/LIO融合与安全监督验收后才能开启导航相关配置。

```bash
cd /workspace/nxk
# 单独生成网页令牌，不复用UDP密钥；输入网页控制抽屉，不放URL。
export NXK_WEB_TOKEN="$(openssl rand -hex 32)"
# 通过私有方式保存本终端令牌供浏览器输入，不发送给他人。
bash scripts/launch_real_workspace.sh control_config:=/root/.config/nxk/control.json
```

不传 control_config 时仍是默认只读。网页启动流程：握手 → 起立完成反馈 → loco实际反馈 → READY零速度待命。已在loco则直接接管待命。READY时可设置姿态或确认落下。导航任务中不允许姿态/落下：先停止导航，再启动接管loco待命、显式回正并确认停稳。网页排队成功不代表物理动作成功。

## 尚未完成的导航与安全前提

当前 GPS-LIO 融合、本体外参、真实地面过滤和完整独立安全监督尚未全部接通。非零导航速度要求 `/real_navigation/safety_status` 来自真实监督器，检查定位、TF、障碍输入和道路配置，不能手工发布 true 绕过。前进上限0.3m/s、转向0.5rad/s，当前禁止倒车，不采用未经实测的2m/s。

接收器200ms无新有效指令、SDK关闭或FSM过期即发零速度、撤销会话；不会自动恢复。模式切换、浏览器心跳、导航输入也有超时。软件零速度不是急停；接收器崩溃或断电时其看门狗同样失效，必须验证厂家控制器自身超时停车与独立硬件急停。落下前的零指令和人工确认不能替代实测速度传感器。

## 本地回归（不控制硬件）

```bash
python3 scripts/test_robot_wire.py
python3 scripts/test_robot_policy.py
python3 scripts/test_robot_udp_pair.py
python3 scripts/test_navigation_control.py
python3 scripts/test_navigation_web.py
```

UDP配对测试仅使用回环地址和模拟FSM，不证明厂家SDK实际接受动作。部署后还须分阶段验证只读、零速度、起落/姿态、断网、接收端崩溃、控制权冲突以及低速导航。
