# Microduck 开发交接：电机、头颈控制与 Live Twin

交接日期：2026-09-15。面向接手本工作区的开发者和 agent。
本文件记录完成状态与下一步；协议机制由
[robotd 头颈控制与 Live Twin](../robot/robotd-head-control-and-live-twin.md) 说明，
具体调试台维护步骤由 [Live Twin README](../../scripts/live_twin/README.md) 维护。

## 先知道这五件事

1. 用户说机器人已经装配完成，当前 SSH 地址为 **`root@10.4.1.139`**。
   不再默认使用之前的 USB 地址 `192.168.77.1`。
2. 维护主仓库是 `/Users/homalozoax/micro_duck/microduck`。
   板级镜像、设备树和平台适配在相邻的 `microduck-orangepi-os`，不要混成一个仓库提交。
3. 当前正式服务运行 **0.10.0 / IPC API 16**；主仓库与 Live Twin 候选程序是另一条已开发到
   0.12.0 / IPC API 28 的代码路径。不要用源码版本代替运行版本。
4. 当前正式 `robotd` 虽然 active，但总线初始化失败，控制循环没有 tick，没有实时状态帧。
   本交接没有排除具体硬件/配置原因，也没有验证当前装配后的电机或 IMU。
5. 现有串口 Live Twin 是独立诊断和受监护姿态控制工具；新增的 IPC-only Twin 才是与正式
   `robotd` 并行的只读查看器。默认 `microduck-twin.service` 不打开电机 UART，也没有控制 POST 接口，
   由 `robotctl twin` 维护。

## 本次只读确认的运行状态

| 项目 | 2026-09-15 快照 |
|---|---|
| SSH / 平台 | 新地址连通，`orangepizero3w`，AArch64 |
| 正式 daemon | `/opt/robot/daemon/current` → `/opt/robot/daemon/releases/0.10.0` |
| 正式执行参数 | `/opt/robot/daemon/current/bin/robotd --socket /run/robotd.sock` |
| 服务 | `robotd` active/enabled，`padd` active/enabled，`microduck-twin` inactive/disabled |
| `hello` | API 16，daemon 0.10.0，revision 未提供 |
| `robot.health` | healthy=false，degraded=true；startup_failures=341，ticks=0，IMU 未 ready |
| 日志 | `combined imu+motor sync_read: Operation timed out`，尚未开始正常控制循环 |
| 订阅 | `robot.subscribe` accepted；随后 3 秒无 `robot.state` |
| 模型查询 | `robot.modelApi`=1；`robot.model` 尚不支持 |
| 总线配置 | `/etc/robot/robotd.toml` 的 bus 块仅含 `/dev/serial0`，未显式指向安装零位文件 |
| Live Twin 候选程序 | `/root/calibration/control-path/bin/robotd-ui-fast`，自报 0.12.0 |

`startup_failures` 会随时间增加，341 是采样值。无总线读数时，不能从历史 Torque OFF 记录
推断此刻逐电机扭矩寄存器，也不能从 `padd` active 推断机器人正在运动。
当前总线启动问题需要独立处理，不应拿 2026-09-14 的成功试验覆盖它。

本机只读证据保存在
`/Users/homalozoax/micro_duck/calibration/robotd-handoff/2026-09-15/board-readonly.json`。
该目录不在主仓库 Git 中；其他开发者应重新采集当前状态。

## 已完成的功能，以及完成到哪一层

| 功能 | 已有实现 | 验证与限制 |
|---|---|---|
| 15 路安装零位 | 工装零位转换为模型坐标；嘴部闭口显示 0 对应 runtime -5°；转换位于硬件 I/O 边界 | 历史工装读取已完成。重新装配后需核对适用性；当前正式 0.10.0 的标定接入未确认 |
| 编码器跨圈恢复 | 临时 Mode 4、受限恢复区间、捕获并恢复原模式和 RAM 设置 | 受支撑的慢速 HOME 已实机通过；不等于允许任意全圈运动 |
| UART RX DMA | 平台仓库增加 RX DMA 配置与恢复说明 | 2026-09-14 的 8,951 个普通读取周期、1,494 个宽读取周期无错误/重试；这是历史试验，不是当前总线已 ready 的证明 |
| Live Twin 遥测与三维显示 | 串口模式与 IPC-only 模式；后者订阅 `robot.state`、显示实测/目标并不打开 UART | 串口模式已实机验证；IPC-only 后端与只读服务已实现，当前 0.10.0 板上仍无健康状态帧 |
| HOME / 回零 / 卸力 | 固定姿态到位保持、同一持有会话切换、独立进程卸力与设置恢复 | 较慢版本的网页 HOME→保持约 21 秒→卸力→恢复在线已实机通过 |
| 网页会话和故障处理 | Host/Origin/网段检查、页面 capability、单个姿态所有者、浏览器心跳和原生进度双监护、pidfd 停止目标生产者 | 隔离测试覆盖网页失联、控制进程卡住/被杀；浏览器不是唯一卸力保障 |
| 串口交接显示与恢复 | 准备/恢复进度、连续空读或失败后重开 UART、取消后恢复原设置 | 实机完整准备和中途取消均验证；不再把有意的采样交接显示为全掉线 |
| 准备提速 | 完整应答后结束等待、完整占用扫描用 `os.scandir`、读回确认后省去 RAM 空等、正确 hold goal 不重复写入 | 无加力实机准备 **3.21 秒**；缺失/延迟回复仍保留原超时，模式切换等待和写入核对保留 |
| 交互姿态提速 | 最大 20°/s、40°/s² 的同步梯形轨迹，保留稳定到位检查 | 原生隔离测试通过；**加速后的真实 HOME/回零动作尚未验证** |
| 普通 `robotd` 头颈和嘴部意图 | `robot.head`、`robot.look`、`robot.mouth`、`robot.subscribe` 的源码接口已有；嘴部 ID 34、向量索引 9、runtime -5°..+30° | head/look/mouth 依赖允许驱动的控制循环；嘴部会让 theremin/chorale 优先占用；不是无策略的直接伺服控制，当前板上仅验证只读 RPC |
| 默认 IPC-only Twin | `microduck-twin.service`、`scripts/live_twin/ipc_server.py`、`robotctl twin status|enable|disable|restart` | 默认回环 HTTP 只读页面；不会打开 UART 或影响 robotd；随正式 daemon release 安装后开机启用 |

加速前网页 HOME 实机记录：到位最大误差约 1.88°，保持约 21.17 秒，单电机峰值输入电流
268 mA，最高 30°C，未发生读重试。不要把这些数字归给随后改成的 20°/s 轨迹。

### 主要提交

这是普通 Git 定位信息，后续按实际工作小步更新，不需要另设版本冻结文件。

| 主仓库提交 | 内容 |
|---|---|
| `ca104f0`、`57283a0`、`2aa8724` | 有支撑 HOME 稳定性、跨编码器边界恢复与说明 |
| `a938866`、`e9966d4` | 交互式 HOME/回零保持与 Live Twin 控制界面 |
| `d2d2bd0`、`154b733` | 设置读取与串口交接/恢复修正 |
| `3fbae1e`、`ed27589` | 轨迹和准备提速 |
| `c261bee` | Live Twin 维护说明与文档入口 |
| `819e015` | 默认只读 IPC-only Twin、systemd unit、`robotctl twin` 生命周期命令和 release 打包 |
| `919af95` | 默认 Twin 服务的协议、维护和交接文档 |

平台仓库的 `d4efe45`、`41d910b` 对应 UART RX DMA 和恢复文档。

## 找代码时从哪里开始

| 任务 | 入口 |
|---|---|
| 查 JSON-RPC 方法和字段 | [duck-ipc-proto/src/lib.rs](../../duck-ipc-proto/src/lib.rs)：`Call`、`HeadParams`、`LookParams`、`RobotState` |
| 查意图如何成为电机目标 | [robotd/src/main.rs](../../robotd/src/main.rs)：`apply_intent`、`dispatch`、`driving`、targets 分支；[control.rs](../../robotd/src/control.rs) |
| 查控制权与断联语义 | [intents.rs](../../robotd/src/intents.rs) 与 [safety.rs](../../duck-control/src/safety.rs)：head 最后写入者生效，deadman 只归零 twist |
| 查头颈顺序、HOME、嘴部 | [model.rs](../../duck-control/src/model.rs)、[obs.rs](../../duck-control/src/obs.rs)、[head.rs](../../kinematics/src/head.rs)；嘴部协议见 [robotd 头颈控制与 Live Twin](../robot/robotd-head-control-and-live-twin.md) §6 |
| 查安装零位 | [calibration.rs](../../duck-control/src/calibration.rs)、[export_joint_zero.py](../../scripts/export_joint_zero.py)、[标定文档](../robot/joint-calibration.md) |
| 查网页固定姿态控制 | [homing.rs](../../duck-control/src/bus/homing.rs)、[pose_session.rs](../../robotd/src/pose_session.rs) |
| 查网页后端和监护 | [Live Twin](../../scripts/live_twin/README.md) 中的模块职责表 |
| 查板级约束 | 相邻平台仓库的 [Orange Pi 部署说明](../../../microduck-orangepi-os/docs/deploy/orangepi-zero3w.md) 和 [UART RX DMA](../../../microduck-orangepi-os/docs/deploy/orangepi-uart-rx-dma.md) |

平台说明将 BMI088 定义为辅助采样/健康信息，原控制闭环仍使用 `imu_to_dxl`。
不要因为新装了 BMI088，就假设 `combined imu+motor sync_read` 不再需要原来的总线 IMU。
当前超时的具体原因仍需实查。

## 部署与本地资产边界

- 主仓库是维护源。`/root/calibration/digital-twin` 是已有板上部署副本，不能只改副本而漏回仓库。
- `/root/calibration/control-path/bin/robotd`、`robotd-ui`、`robotd-ui-fast` 是调试候选程序；
  放在那里不等于更新了 `/opt/robot/daemon/current` 或正式服务。
- `/root/calibration/digital-twin/calibration.json`、旧 STL/Three.js 资产、`servo_config.py`
  和 `/var/lib/robot/live-twin/runs/` 属于设备部署/证据。源码 README 已列出外部依赖。
  新开发者不能只 clone 仓库就假设这些设备特定文件自动存在。
- 在本工作区，历史实验主要位于 `calibration/home-probe/`、`calibration/live-twin-update/`
  和 `artifacts/uart-recovery/`。它们位于主仓库之外，不要把调试产物、机器零位或凭证混入源码提交。
- 当前新 IP 的 LAN 访问不自动符合旧网页的 USB 写控制 allowlist；查看与写控制的权限要分开处理。

## 接手后的工作顺序

### A. 先重新确认运行环境，定位当前启动失败

可以先执行以下只读操作：

```sh
ssh root@10.4.1.139 'systemctl show robotd microduck-twin padd --property=ActiveState --property=ExecStart'
ssh root@10.4.1.139 'readlink -f /opt/robot/daemon/current; robotctl --version'
ssh root@10.4.1.139 'journalctl -u robotd -n 30 --no-pager'
```

再用协议说明中的只读客户端检查 hello、health 和 state。把电源/连接、预期设备、所用 UART、
实际程序及其配置逐项对上。若需要独占串口检查，先明确停止控制的影响与机器人支撑状态，
通过既有维护路径操作；不要在正式 daemon 占用 UART 时另开一个读写程序。
当前任务只写文档，没有替接手者完成这项故障修复。

### B. 确认正式 runtime 与标定的集成

先确认是否要将维护中的标定与平台适配纳入正式服务，以及所选可执行文件是否包含这些功能。
更新要沿已有部署流程做可回溯的小步修改，保持平台仓库边界；
不要直接用调试候选文件覆盖 `current` 并顺手启动步态。
完整装配后，重新核对 motor ID、参考零位、模型空间解释与 IMU 适用条件。

### C. 维护可与 daemon 并行的只读 Live Twin 后端

IPC-only 后端已按协议说明第 5 节实现：只订阅 socket，不打开 UART、不启动姿态 worker、不修改控制模式。
默认服务由 `robotctl twin status|enable|disable|restart` 管理；它的 `disable` 只影响网页，
不停止 `robotd`、`padd` 或电机功能。
接受“旧版本缺少 robot.model / skeleton”和“订阅成功但无状态帧”这两类实际状态。
缺失电流、温度、扭矩等字段应显示未知；不能将 command.head 当成实测头部。
查看器退出不能卸力、停止 daemon 或切换策略。

最小验收：同一个 `robotd` 进程继续运行；UART 只有原控制者；
状态与模型正确对应，目标/实测可区分；断流显示过期；重连不产生电机动作。

### D. 根据需求决定是否补充“无策略头颈直接控制”

若策略条件下的 `robot.head` 足够，就沿现有接口开发，并明确与 `padd` 的控制权。
若要腿固定、只动头颈，则在 `robotd` 内增加有明确进入/退出条件的控制分支。
先定义断联后保持、回姿态或停止的具体行为，再实现；不能把网页诊断模式的断联全卸力
无条件复制给正在站立的机器人。该分支必须复用模型标定、行程/速度限制和总线所有权。

嘴部也遵循同一边界：`robot.mouth` 是模型空间的 `open` 意图，不能当成无策略的 ID 34
直接控制；普通策略、theremin、chorale 的优先级和退出行为必须先定义清楚。

### E. 完成加速版的实际动作验证

此前仅完成无加力准备和隔离轨迹测试。需要用户确认当前支撑与活动空间后，
分别记录冷启动准备时间、HOME 到位、回零到位、保持间切换、卸力和恢复时间，
同时记录追踪误差、电流、温度、通信错误和监护是否触发。
原来的“约 3 秒常用切换、约 10 秒大角度恢复”是轨迹估算，不能提前填成实测结果。

## 验证命令与完成记录

主仓库中相关检查：

```sh
cargo test --locked -p duck-control --lib
python3 -m unittest discover -s scripts/tests -p 'test_*.py'
python3 -m unittest discover -s scripts/live_twin/tests -p 'test_*.py'
node --test scripts/live_twin/tests/*.test.mjs
cargo fmt --all --check
git diff --check
```

使用 Python 3.11+。之前本机使用 Python 3.14。
最后一次加速代码验证为 Rust 96 passed、1 个依赖外部 ONNX Runtime 的测试 ignored；
维护脚本 Python 26 passed；网页/快速传输 Python 17 passed；板上隔离套件 17 项通过。
这些是既有记录，本次文档任务未重新运行全部测试。

Linux 原生隔离测试应通过 `scripts/run_calibrated_control_path.py` 启动。
它使用非 root 测试用户、私有 PTY 和设备隔离；不要绕开隔离保护直接在真串口上运行模拟测试。
交付时写清源码修改、实际运行程序、执行过的验证、未验证项以及最终硬件/服务状态。
提交使用小步的 Conventional Commits；不因文档说明而新增 hash、冻结文件或额外 gate。
