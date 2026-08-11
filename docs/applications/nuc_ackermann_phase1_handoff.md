# NUC Ackermann 一期导航闭环实施交接

状态：软件闭环与正式 Wheeltec 后端候选已实现；物理部署验收仍未完成
日期：2026-08-10
目标分支：`feat/nuc_ackermann`

截至 2026-08-11，本文第 9 节所列的软件实现已经落地：公共合同、定位转换、规划、
Pure Pursuit、安全/车辆执行、fake VCU 闭环，以及同进程、无 `/cmd_vel` 绕行边界的
Wheeltec 串口后端均有实现和自动化测试。正式串口 launch 仍是独立的
`UNVERIFIED` 组合，七个物理/授权/readiness 门默认全部关闭，设备路径为空；因此
“后端代码已接入”不等于“实车后端已获准启用”。本修订还把共享串口工厂的物理写入
release gate 编译为关闭，任何 YAML、launch 或调用方 opt-in 都不能越过。原因是供应固件
的 11 字节 UART parser 会跨 Linux 进程重启保留半帧计数；在车载固件身份和断电状态下的
重同步流程得到证据之前，一次 host 完整零帧不能证明 MCU 已解析零。本文件后续内容同时
保留原始设计依据和尚未关闭的物理验收项。

## 1. 任务目标

在 Ubuntu 20.04、ROS 1 Noetic 上，基于 AUTO_ROVER 的既定边界，从头实现一套可维护的
Ackermann 小车已知航点导航闭环：

```text
FAST-LIVO2 位姿
  -> EgoState
  -> 航点 RoutePlan
  -> 可执行 Trajectory
  -> MotionReference
  -> 安全检查和车辆执行命令
  -> VCU 适配边界
  -> 底盘
```

一期的功能目标是：分别启动感知、规划/控制和车辆执行程序后，小车能按照用户编辑的航点
前向行驶。用户会在软件侧完成后协助架空轮/台架和低速实车测试。

本任务只做已知航点导航。不要加入在线障碍物检测、局部避障、探索、覆盖规划、运行时任务
管理器或 ROS 2 包。

## 2. 已确认的用户决策

1. 使用规范分支名 `feat/nuc_ackermann`。
2. 新功能包从头实现。厂家包和旧 `vdemo` 只作为协议、参数和行为证据，不复制为本项目的
   运行时依赖。
3. FAST-LIVO2 保持为外部部署，不搬入本仓库。
4. 一期不发布完整 TF 树；但定位适配器必须用显式、可配置的固定外参，把感知位姿数值转换
   到后轴中心控制参考点。不能把传感器坐标直接冒充车辆参考点。
5. `/cmd_vel` 只允许出现在选定 VCU 的适配边界，不作为规划、控制或安全模块的内部通用命令。
6. 不允许倒车；一期仅实现前向跟踪和停车。
7. 实车测试只分两阶段：架空轮/台架、受控场地低速实车。
8. 考虑到底盘存在死区，两个实车阶段都以 **`0.50 m/s` 作为首个前向测试目标值**。
   受加速度斜坡约束，实际发送的首个非零命令仍会低于该目标。`0.50 m/s` 是用户指定的
   commissioning 目标，不代表死区已经被测得，也不是允许绕过限速或安全检查。
9. VCU 串口的最终字段语义、时序和标定放在最后结合真机调整。先完成所有不依赖真机的
   接口、算法、保护、仿真和集成工作。

## 3. 外部证据和代码来源

### 3.1 感知源

- 已部署 FAST-LIVO2：`/home/nuc_x/fastlivo2/`
- 当前预期位姿话题：`/aft_mapped_to_init`
- 当前预期世界坐标系：`camera_init`

实施前必须从实际 launch、源码和 `rostopic` 类型确认话题、消息类型、时间戳、父/子参考点、
姿态约定和发布频率。不能仅凭上述预期冻结 `EgoState` 转换。

### 3.2 厂家代码

- 厂家原版 A：
  `/home/nuc_x/桌面/ws2/src/turn_on_wheeltec_robot/`
- 厂家原版 B：
  `/home/nuc_x/桌面/car/JP514_noetic_wheeltec_robot_src_241205/src/turn_on_wheeltec_robot/`

两份厂家目录已经只读比较，整树内容相同。后一个路径没有提供另一套更精确的 JP514 专用
协议或标定。

### 3.3 已有实车修改版

- `/home/nuc_x/vehicle_nav/vdemo/src/turn_on_wheeltec_robot/`
- 相关车辆参数：
  `/home/nuc_x/vehicle_nav/vdemo/src/vdemo_bringup/params/vdemo_params.yaml`
- 相关安全逻辑可作为负面和正面证据，但不能直接复制：
  `/home/nuc_x/vehicle_nav/vdemo/src/vdemo_safety/`

`vdemo` 的改进点包括串口完整写检查、单调时钟命令 watchdog、非阻塞接收和退出零帧；已知
问题包括订阅队列为 100、底层缺少非有限数/速度/曲率检查、车型参数写错成员，以及软件
急停未锁存。新实现应保留有证据支持的思路并修正这些问题。

## 4. 初始车辆配置

车辆配置必须作为版本化数据存在，不能硬编码在规划器或控制器中。初始控制参考点为后轴
中心，坐标约定为 `+x` 向前、`+y` 向左、`+z` 向上，长度使用米、角度使用弧度。

```yaml
schema_version: 1
vehicle_id: nuc_senior_akm
kinematic_model: ackermann_bicycle
reference_point: rear_axle_center

geometry:
  wheelbase_m: 0.3187             # 商品/CAD 一致，尚未手工实测
  rear_track_m: null              # CAD 估计 0.310232，未验证且一期核心不用
  front_track_m: null             # CAD 估计 0.307232，未验证且一期核心不用
  overall_length_m: 0.4447        # 商品/CAD 一致，尚未手工实测
  overall_width_m: 0.3620         # 商品标称；CAD 四轮包络约 0.3598
  wheel_diameter_m: 0.1250        # 商品标称；CAD 约 0.126
  wheel_tread_width_m: 0.0408     # 商品/CAD 一致，尚未手工实测
  ground_clearance_m: 0.0400      # 商品标称，尚未实测
  body_x_min_from_reference_m: -0.0630
  body_x_max_from_reference_m: 0.3817
  front_overhang_m: 0.0630        # 前轴到车头，按现有几何推导
  rear_overhang_m: 0.0630         # 车尾到后轴

footprints:
  catalog_body_envelope_xy_m:     # 保守商品包络，不是已测 physical truth
    - [-0.0630, -0.1810]
    - [ 0.3817, -0.1810]
    - [ 0.3817,  0.1810]
    - [-0.0630,  0.1810]
  initial_operational_xy_m:       # 厂家静态 footprint；安全 padding 未验证
    - [-0.0900, -0.1850]
    - [ 0.4000, -0.1850]
    - [ 0.4000,  0.1850]
    - [-0.0900,  0.1850]

limits:
  initial_max_forward_speed_mps: 0.50
  initial_max_longitudinal_accel_mps2: 0.20  # 厂家规划值，待台架验证
  operational_min_turning_radius_m: 0.95
  catalog_min_turning_radius_m: 0.80         # 定义/参考轨迹未知，UNVERIFIED
  verified_hardware_min_turning_radius_m: null
  reverse_supported: false

commissioning:
  bench_forward_target_mps: 0.50
  ground_forward_target_mps: 0.50            # 用户指定；仅台架门槛通过后使用
  historical_ground_fallback_mps: 0.35       # 旧 vdemo 的 P5 碰撞后回退记录
  measured_start_deadband_mps: null
  measured_stop_deadband_mps: null
```

数据依据和限制：

- 商品图给出总长 `444.7 mm`、总宽约 `362 mm`、轴距 `318.7 mm`、轮胎标称直径
  `125 mm`、胎面宽 `40.8 mm`、离地间隙 `40 mm`。
- 旧 `vdemo` 的实车参数给出轴距约 `0.319 m`，并把规格表 `0.80 m` 当作转弯半径。该半径
  是外轮轨迹、后轴中心轨迹还是其他口径并未确认，不能称作已测机械极限。
- 将原厂底盘/车轮 STL 与 `vdemo` 的详细 SolidWorks URDF 视为同一底盘装配是高置信推断，
  不是可由哈希证明的完整来源链。组合几何与商品图在轴距、总长和胎面宽上高度一致；CAD
  轮外径约 `126 mm`，应把 `125 mm` 视作商品标称值。
- 厂家通用参数另有轴距 `0.322 m`、最小转弯半径 `0.75 m`；简化 URDF 又使用
  `0.310 m` 轴距和 `0.100 m` 轮径。这些冲突值不能覆盖上述初始实车配置。
- 左右轮距尚未实测。CAD 推导的后/前轮中心距约为 `0.310232/0.307232 m`，一期后轴中心
  自行车模型不使用它们，因此运行配置先设为 null。
- `0.95 m` 是一期唯一批准的运行最小半径，规划器、控制输出检查和 guard 都不得突破它。
  `0.80 m` 只保留为定义未知的商品/旧配置证据；实测硬件能力保持 null。
- 商品标称外形、CAD 包络和厂家静态 footprint 都标记 `UNVERIFIED`。厂家 footprint 虽包住
  商品包络，但横向每侧只多约 `4 mm`，不能称为已验证的动态安全裕量。

### 4.1 `0.50 m/s` 与死区的实现规则

- 将 `0.50 m/s` 设为台架目标，以及台架门槛通过后的低速场地目标；初期车辆配置的前向
  上限也限制为 `0.50 m/s`。它是目标值，不是实际发送的第一个非零离散命令。
- 不要因为“可能存在死区”在 guard 或 VCU 适配器里悄悄把任意小正速度抬升为
  `0.50 m/s`。这种跳变会改变上层合同且影响停车安全。
- 规划器初始航点速度可使用 `0.50 m/s`，控制器仍必须生成受加速度约束的启动和停车过程；
  VCU 收到的过渡命令可能短暂低于死区，这是正常且需要记录的测试现象。
- 台架阶段按实际发送值做受控的单调缓升/步进和驻留，以有效轮速反馈定义“首次可持续运动”；
  再做缓降，分别记录启动与停止死区及滞回。然后填写两个 measured 字段，并通过新配置版本
  决定是否需要显式死区补偿策略。
- 未完成台架测量前，不把 `0.50 m/s` 写成已经验证的最小可控速度。
- 旧 `vdemo` 在一次 P5 碰撞后曾记录“下一次地面先回到 `0.35 m/s`”。本文保留该风险证据，
  同时记录用户当前改用 `0.50 m/s` 目标的决定；因此 `0.50 m/s` 地面测试必须先通过新的
  台架门槛，并在具体测试前显式 go/no-go，不能只靠配置文件自动获得授权。

## 5. 目标接口和依赖方向

算法核心必须无 ROS API；ROS 参数、消息、发布订阅和日志只存在于薄封装层。公共合同在第一
次使用前必须记录坐标系、单位、符号、时间戳、有效性、新鲜度、失效行为和版本兼容策略。

```text
RoutePlan -> Trajectory -> MotionReference
                           |
EgoState ------------------+
                           v
ChassisState ------> guard / vehicle motion manager
                           |
                           v
               VehicleExecutionCommand
                           |
                           v
                   selected VCU adapter
```

模块边界：

- `auto_rover_core`：ROS 无关领域类型、校验和公共数学。
- `auto_rover_interfaces`：ROS 1 Noetic 消息和服务。
- `auto_rover_ros1_conversions`：ROS 消息与核心类型的显式转换。
- `auto_rover_localization`：FAST-LIVO2 到 `EgoState` 的薄适配。
- `auto_rover_planning`：`RoutePlan` 校验、插值和曲率可行的 `Trajectory` 生成。
- `auto_rover_control`：以前轴/后轴参考约定明确的 Pure Pursuit 初版跟踪器，输出
  `MotionReference`，不得了解串口或 `/cmd_vel`。
- `auto_rover_vehicle`：车辆能力、方向/静止约束、执行状态机和
  `VehicleExecutionCommand`。
- `auto_rover_safety`：输入新鲜度、有效性、限值、watchdog、锁存软件急停和授权复位。
- `auto_rover_vcu_wheeltec_serial`：在当前观察格式已记录为暂定协议证据、并且能同时提交
  实质 codec/transport 实现和测试时才创建。真机证据补齐前整个包标记 `UNVERIFIED`，实际
  设备后端不得默认启用；该包只做协议和归一化反馈转换，不做轨迹跟踪。
- `auto_rover_known_map_bringup`：一期已知航点导航的配置和最小 launch 组合。

按需创建包，每个新包必须同时有实质实现和测试；不得先生成一批空包。

## 6. 定位、航点和规划约定

### 6.1 定位

- 一期不依赖运行时 TF 查询，也不发布一套虚构的 TF 树。
- 定位适配器从 FAST-LIVO2 源消息读取源时间戳和位姿，检查有限性、时间顺序和新鲜度。
- 固定外参作为版本化配置，以明确的乘法顺序将传感器/IMU 位姿转换为后轴中心位姿。
- 外参未知或无效时，`EgoState` 必须无效并禁止车辆运动，不能使用单位变换静默顶替。
- 初期航点和 `EgoState` 都使用 FAST-LIVO2 的会话坐标系 `camera_init`。没有重定位能力时，
  航点只对同一建图/启动基准有效，重启后不能宣称跨会话复用。

### 6.2 用户可编辑航点

固定编辑入口：

```text
src/apps/known_map_navigation/auto_rover_known_map_bringup/config/waypoints.yaml
```

建议最小格式：

```yaml
schema_version: 1
frame_id: camera_init
route_id: commissioning_route_01
loop: false
waypoints:
  - {x_m: 0.0, y_m: 0.0, yaw_rad: 0.0, speed_mps: 0.50}
  - {x_m: 2.0, y_m: 0.0, yaw_rad: 0.0, speed_mps: 0.50}
```

加载器要拒绝空列表、非有限数、重复/倒序版本、错误坐标系、负速度、超过车辆配置的速度和
不可实现的曲率。提供显式 reload 服务；reload 失败时保持安全停止，不继续执行旧路线。

### 6.3 初版规划和控制

- 规划只处理静态航点几何和车辆曲率约束，不建立环境模型。
- 轨迹包含路径位置、朝向、曲率、目标速度、方向、标识、时间/有效期和完成语义。
- 控制器初版使用 Pure Pursuit；前视距离、到点阈值、停车距离和发布频率均为配置，并由单元
  测试覆盖。
- `MotionReference` 用后轴中心速度 `m/s` 和曲率 `1/m` 表达；正速度向前，正曲率左转。
- 接近终点必须受控停车并保持零命令，不能靠轨迹话题消失实现停车。

## 7. 必须先实现的安全行为

默认启动不得驱动车辆。只有在显式 arm、有效路线和全部必需输入新鲜时才允许非零执行命令。

至少实现并测试：

- 所有必需浮点数的 NaN/Inf 拒绝；
- 定位、轨迹、控制输出和底盘反馈的独立新鲜度检查；
- 上限、加速度和一期批准的 `0.95 m` 运行最小转弯半径检查；
- Ackermann 横向速度恒为零；
- 禁止近零速下的纯横摆角速度命令；速度为零时，适配边界的横摆角速度也必须为零；
- 倒车能力为 false 时拒绝任何负速度；
- 两个独立的单调时钟 watchdog：command guard 检查上游消息 age/`valid_for`，VCU 适配器
  另行检查每个 `VehicleExecutionCommand` 的 age/本地 deadline，并维护输出 watchdog；
- 命令通道采用“最新值”语义，ROS 队列深度为 1，不能回放积压命令；
- 任一 watchdog 超时都拒绝旧命令；正常恢复需要满足配置数量的连续新鲜命令，不能收到一帧
  就立即恢复运动；
- 通讯/输入失效时软件持续生成显式零速安全命令；链路仍可写时由适配器按策略重试零帧并
  记录失效原因。链路已断时必须报告 `delivery_unconfirmed`，不能据此宣称物理车辆已经停车；
- 软件急停走独立路径并锁存，不自动恢复；条件清除后仍需授权 reset；
- 物理急停保持独立，软件不得替代或削弱它；
- 启动、重连、VCU 重启和退出时不重放旧命令；重连后保持 inhibit，要求连续新鲜命令并由
  操作员重新 arm，不能自动恢复之前的运动状态。

具体 timeout、停止时间和距离等验收数字不能为了通过测试凭空填写；在台架证据形成后固化到
版本化验收配置。

## 8. VCU 边界：静态证据、暂定实现和最后调整

### 8.1 静态源码证据和协议假设

当前三套代码都按相同方式组装下行运动帧；这只能证明源码行为，不能证明实际 VCU 行为：

```text
11 bytes:
0x7B
0x00, 0x00  # 现有发送代码固定写零；VCU 含义未知
intended signed 16-bit, high byte first: linear.x * 1000
intended signed 16-bit, high byte first: linear.y * 1000
intended signed 16-bit, high byte first: angular.z * 1000
XOR(frame[0..8])
0x7D
```

厂家源码用平台相关的 `short` 和隐式浮点转换。新 codec 必须使用定宽整数，明确量化规则，
并在转换前拒绝非有限数和越界值；不能复刻未定义的溢出行为。

现有配置表明 Ackermann 模式的 `angular.z` 是横摆角速度而不是舵角，控制板内部完成换算。
串口默认设备和波特率为 `/dev/wheeltec_controller`、`115200`。接收侧现有代码按 24 字节帧
解析速度、横摆角速度、IMU 和电压。该帧没有可见的 VCU 源时间戳或序号；真机协议确认前，
归一化反馈必须把 source time 标为 unavailable，只能使用本机单调 receipt time 计算接收
新鲜度，不能伪造采样时间。

```text
24-byte candidate feedback layout from source:
[0]       0x7B header
[1]       reserved / "Flag_Stop" (meaning unverified)
[2..3]    intended signed 16-bit forward speed, high byte first, /1000
[4..5]    intended signed 16-bit lateral speed, high byte first, /1000
[6..7]    intended signed 16-bit yaw rate, high byte first, /1000
[8..13]   three intended signed 16-bit accelerometer channels
[14..19]  three intended signed 16-bit gyroscope channels
[20..21]  intended voltage in millivolts
[22]      XOR(frame[0..21])
[23]      0x7D tail
```

> 2026-08-11 evidence addendum: the subsequently supplied C50C firmware source
> defines byte 1 as binary composite `FlagStop` (`0` current-cycle control
> allowed, `1` inhibited). It remains non-latched, is not a command ACK or a
> specific fault code, and the connected MCU has not been read back to bind it
> to the supplied HEX. See
> [the firmware source audit](../evidence/2026-08-11-wheeltec-c50c-firmware-source-audit.md).

只有完整、帧头/帧尾正确且校验通过的帧才能刷新本机 receipt freshness；坏帧不能刷新。若 ROS
消息需要 wall/ROS 时间戳，必须明确标记它是接收时间及其时钟域。修改版还解析超声和回充帧，
一期将它们明确排除，不因此扩大 `ChassisState`。

这些内容在本交接建立时是**待真机确认协议假设**，不是已经通过硬件验收的正式合同。交接
当时机器没有可用的 `/dev/wheeltec_controller`；之后的连接、被动反馈和架空台架证据另见
`docs/evidence/2026-08-11-*`，仍不构成真实闭环或落地验收。

> 2026-08-11 后续实机证据：车辆没有蘑菇式硬件急停按钮；后来确认有可由监护人立即断开并
> 保持断开的主电源断路器。它切断电池侧牵引供电，但 VCU 逻辑会继续由 NUC USB 供电。
> 该断路器只被条件性用于车辆架空、固定、轮下净空且有人持续值守的本次台架表征，不替代
> 地面运行/发布所需的独立安全链。只读数据观察到断路器断开时约 `4.46 V`、`FlagStop=1`，
> 合闸时约 `23.33 V`、`FlagStop=0`；由于回传无源时间戳且打开串口时有积压帧，这不能测量
> 断路器延迟或证明外部电机母线。合闸后的全零测试完成；之后两次正向直线斜坡都因异常的
> 轮编码器派生反馈而安全中止，最大下发分别仅为 `0.312 m/s` 和 `0.080 m/s`。因此死区、
> 持续起转时刻、正常减速和停止拖尾仍未测得，真实 VCU 闭环继续为 `UNVERIFIED` 且默认
> 禁用。后续 60 秒上电静止只读基线的 1,213 帧均未超过 `|forward|=0.001 m/s`、
> `|yaw|=0.011 rad/s`，说明第二次斜坡的 `-0.170/1.059` 不是常态静止随机噪声，但仍不能
> 唯一确定根因。本轮没有启动或测试任何感知组件。详见
> [主断路器架空台架后续证据](../evidence/2026-08-11-wheeltec-breaker-raised-bench-follow-up.md)
> 和[安全门槛更正时间线](../evidence/2026-08-11-wheeltec-estop-gate-correction.md)。

> 2026-08-11 最终架空矩阵补充：在独立、默认禁用且不安装的 ROS-free
> raw-profile 工具下，后来完成了 `52/52` 个最终请求工况：直线
> `0.50--6.0 m/s` 十二档，以及外轮命令档 `0.50--5.0 m/s`、左右两向、
> R=`2.0/0.95 m` 四十条。全部工况约保持 60 秒、正常归零、最终零帧主机
> 完整写入，`93,599` 个反馈帧均为 `FlagStop=0`，parser 汇总错误为零。
> 直线均值拟合为
> `feedback = 1.000313388 * command - 0.010680115 m/s`；二十对转弯的
> 左右镜像中心速度残差最大 `0.000629 m/s`，yaw 反对称残差最大
> `0.003553 rad/s`。预臂派生轮速噪声最大 `0.003703 m/s`，支持本架空
> 工具的逐轮 `0.005 m/s` 静止候选；反馈约 `20 Hz`、最大间隔
> `61.257 ms`，支持保留 `150 ms` receipt freshness。直线起转候选关联
> 命令为 `0.228--0.338 m/s`，但无 VCU ACK/source timestamp，仍不能把
> 该区间硬编码为死区或纯控制延迟。完整结果见
> [扩展 raw-profile 矩阵](../evidence/2026-08-11-wheeltec-expanded-raw-profile-matrix.md)。
> 这些数据不改变 Phase-1/生产 `0.50 m/s` 上限，也不授权落地运行。

### 8.2 在协议最终确认前可以完成的工作

- 先冻结协议无关的 `VehicleExecutionCommand` 和 `ChassisState` 语义。
- 设计很薄的传输/编解码边界，使用 fake/PTY/录制帧测试，不让串口字段进入上层模块。这里
  可以实现 `UNVERIFIED` 的纯 codec 和传输 harness，但不得默认打开真实设备或自称 ready。
- 暂定并测试内部命令映射：`linear.x = forward_speed`、`linear.y = 0`、
  `angular.z = forward_speed * curvature`；速度为零时强制 `angular.z = 0`。物理语义继续标记
  `UNVERIFIED`。
- 现在就实现并测试完整/短写处理、有界零帧重试、流式 parser 重同步、坏帧拒绝和断开报告；
  真机阶段只验证这些策略在实际串流和时序下的表现。
- fake VCU 必须支持正常反馈、丢包、过期、坏帧、断连、重连、停机和故障注入。
- 集成闭环先以 fake VCU 验证完整数据流和失效行为。
- 串口适配器只发布从真实帧有证据支持的反馈；没有 gear、实际舵角、autonomous enable、
  fault acknowledgement 等字段时标记 unavailable/invalid，不得伪造。

### 8.3 最后结合真机完成的调整

- 确认 `linear.x`、`angular.z` 的物理语义、符号、饱和范围、字节序和缩放；
- 确认保留字节、校验，以及已实现的完整/短写处理、帧同步和坏帧恢复在真机串流中的表现；
- 测量命令和反馈频率、VCU 自有 watchdog、断连安全状态和重连行为；
- 确认速度反馈来源、分辨率和静止判断阈值；
- 确认直行、左转、右转和停止，不先测试倒车；
- 测量从命令到持续轮动的真实死区，验证 `0.50 m/s` 起测选择；
- 确认停车/保持和物理急停效果。

架空轮运行属于 commissioning/证据采集，用来补齐 readiness gate，不是 Phase-1 acceptance。
只有这些证据补齐并通过 `docs/vehicles/vcu-adapter-readiness.md` 后，才可将真实 VCU 适配器
描述为 ready 或允许它成为显式选择的真实后端；默认启用还需应用级安全审查。

## 9. 建议实施顺序

每个阶段先写行为测试，再写实现；保持每个提交可以解释和审查。以下 1--10 的软件交付
现已完成；真机 readiness 仍受第 11 节和独立清单约束。

1. **证据与 ADR**：审计 FAST-LIVO2 的实际输出；记录公共合同、车辆参考点、安全状态机、
   一期无 TF 运行方式和 VCU 延后策略。公共合同或安全语义落地前提交 ADR。
2. **核心类型和接口**：实现最小 `EgoState`、`RoutePlan`、`Trajectory`、
   `MotionReference`、`ChassisState`、`VehicleProfile`、`SafetyState`、
   `EmergencyStop` 和 `VehicleExecutionCommand`，以及 ROS 转换测试。
3. **定位适配**：实现 FAST-LIVO2 源消息到后轴中心 `EgoState` 的转换、外参、时间和失效测试。
4. **航点与规划**：实现 YAML 编辑入口、严格校验、reload 服务和曲率可行轨迹生成。
5. **控制**：实现 ROS 无关 Pure Pursuit 核心和薄 ROS wrapper，覆盖直线、左/右弯、终点、
   过期和无效输入。
6. **车辆与安全**：实现 guard、执行状态机、锁存急停、授权复位、限值、最新命令和 watchdog。
7. **fake VCU 闭环**：实现可故障注入的 fake/PTY，完成规划—控制—安全—车辆反馈集成测试。
8. **最小 bringup**：提供彼此可独立启动的感知接入、规划/控制和车辆执行 launch；默认
   `actuation_enabled:=false`，真实串口不是默认后端。
9. **串口适配末段**：已根据静态源码和台架证据从头实现干净的 `UNVERIFIED` codec、
   transport/runtime、正式 `VehicleBackend`、默认禁用 ROS wrapper、harness 和编解码/
   PTY 测试；未复制厂家包，也不默认打开真实设备。可复用 codec 的最大前向速度由调用者
   显式配置，合法范围为有限且严格 `0 < v < 6.0 m/s`，没有可授权运动的默认值；一期应用
   配置仍明确固定为 `0.50 m/s`。
10. **软件验收与真机交接**：运行所有仓库、包、replay 和集成测试，输出台架/低速场地检查
    表，只把必须依赖用户真机的项目留为未验收。

## 10. 软件侧完成标准

在用户介入真机前，代码任务至少应交付：

- 可编译、非空且依赖方向正确的所需 ROS 1 包；
- 可编辑航点示例和严格校验；
- FAST-LIVO2 适配配置及可用录制/合成输入测试；
- 曲率可行轨迹和 Pure Pursuit 的确定性单元测试；
- guard、锁存急停、watchdog、失效停车和重连测试；
- fake VCU 下的完整闭环集成测试；
- 默认不动作的最小 launch 和分进程启动说明；
- 隔离的串口编解码单元测试和待真机确认项清单；
- 架空轮/台架与受控低速实车两阶段操作表、记录模板和回滚/停止条件；
- 对所有尚未真机验证的能力明确标注 `UNVERIFIED`，不使用“实车可用”措辞。

完成前运行：

```text
python3 -m unittest discover -s tests/repository -p "test_*.py" -v
python3 tools/ci/validate_repository.py --root .
python3 -m compileall -q tools/ci tests/repository
git diff --check
```

并补充所有相关 catkin、包级、replay 和集成测试。在 Ubuntu 20.04/Noetic 工具链之外的宿主
测试不能替代目标环境构建证据。

## 11. 真机阶段交接原则

代码任务可以使用伪终端 PTY，但不得自行打开真实 `/dev/wheeltec_controller`，也不得向物理
VCU 发出运动命令。用户到场后按以下顺序共同执行：

1. 静态检查、物理急停和轮下净空确认；
2. 架空轮状态下显式 arm，先保持零命令；
3. 以受加速度限制的斜坡发出首个 `0.50 m/s` 短时前向测试目标；实际首个非零命令低于
   `0.50 m/s`，据真实下发值和有效轮速测量启动/停止死区与滞回；
4. 架空轮验证小曲率左/右转，曲率不越过 `1/0.95 m^-1` 的规划值；
5. 验证方向、有效反馈、双 watchdog、零命令保持、停止、断连/重连和物理急停；记录真实
   死区、反馈频率、停止行为和串口异常，更新版本化车辆/协议配置并重跑软件测试；
6. 上述台架门槛全部通过并显式 go/no-go 后，才在受控场地以 `0.50 m/s` 目标/上限执行短
   直线、缓弯和少量航点；预留已确认的停止距离，并由独立操作员掌握物理急停；
7. 任一方向、时间戳、反馈、急停或停止行为不符合预期，立即停止，不通过放宽 watchdog、
   freshness、曲率或安全限制继续测试。

真机数据、结果和最终数值应作为独立证据集提交；不要回写历史验收条件来迎合结果。
