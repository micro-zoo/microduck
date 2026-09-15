# 通过 robotd 控制头颈，以及同时查看 Live Twin

更新：2026-09-15。接口说明以本仓库源码为依据；板上实际版本与源码的差异见下方实机快照。

## 结论

外部程序应向 **`/run/robotd.sock` 发送 JSON-RPC 2.0，以换行分帧的 JSON（NDJSON）**。
`robotd` 负责控制循环、策略、安全处理、标定转换和唯一的电机串口访问。
应用程序不应在它运行时另外打开 `/dev/serial0`、发送 Dynamixel 包或启动独立 `robotd init`。

头颈接口是 `robot.head`，注视点接口是 `robot.look`。但目前它们是**策略输入意图**，
不是一个“保持腿不动、无策略直接移动四个头颈舵机”的接口。

现有 Live Twin 的三维界面可以复用；**当前实现不能与正式 `robotd` 同时读取电机串口**。
正确的并行查看方式是增加只读 IPC 数据源，订阅 `robot.subscribe` 返回的 `robot.state`。
该 Live Twin IPC 后端尚未实现，不能把设计方案当成已有功能。

## 1. 当前连接与实机快照

用户已说明机器人装配完成，当前连接改为：

```sh
ssh root@10.4.1.139
```

此前的 `192.168.77.1` 是 USB 调试地址，不能继续默认把它当成当前连接方式。
装配完成不自动证明 IMU 朝向、标定、运动空间和控制软件都已验证。

2026-09-15 经新地址只读核对：

| 项目 | 实际结果 |
|---|---|
| 主机 | `orangepizero3w`，`aarch64` |
| 正式 `robotd` | active；`/opt/robot/daemon/current/bin/robotd --socket /run/robotd.sock` |
| `current` 指向 | `/opt/robot/daemon/releases/0.10.0` |
| 正式程序自报版本 | `robotd 0.10.0`、`robotctl 0.10.0` |
| `hello` | `api_version=16`、`daemon_version=0.10.0`、`revision=null` |
| 本仓库协议常量 | `API_VERSION=28`，与板上不同 |
| Live Twin 使用的候选程序 | `/root/calibration/control-path/bin/robotd-ui-fast`，自报 `0.12.0`；不是正式服务当前运行的程序 |
| 服务状态 | `padd` active；`microduck-twin` inactive、disabled |
| socket 权限 | `/run/robotd.sock` 为 `root:robot`、`0660` |
| 当前总线配置 | `/etc/robot/robotd.toml` 的 `[bus]` 只有 `port="/dev/serial0"`，未显式配置安装零位文件 |

**当前控制循环尚未就绪。** 采样时 `robot.health` 返回 `healthy=false`、
`degraded=true`、`startup_failures=341`、`control_loop.ticks=0`；
日志为 `combined imu+motor sync_read: Operation timed out`。
`robot.subscribe` 接受订阅，但随后 3 秒没有收到 `robot.state`。
`robot.modelApi` 返回 1；`robot.model` 返回 `-32601 unknown method`。

这些是一次有日期的观察，不是永久状态。它说明 socket 可连接与实际电机控制可用是两件事。
供电、连接、联合读取所需设备与版本配置的具体原因尚未定位；不能仅凭这一条错误认定某个器件损坏。
目前也不能宣称正式 `0.10.0` 已使用此前的 15 路工装标定。
本次没有发送运动、策略启用、卸力或服务启停指令。

## 2. 三层协议各自负责什么

| 层 | 使用方式 | 责任 |
|---|---|---|
| 远程连接 | SSH 到 `root@10.4.1.139`，在板上运行客户端 | 访问控制、远程执行；不要每个控制 tick 都新建 SSH 连接 |
| 应用到 daemon | Unix **stream** socket；JSON-RPC 2.0 + 每个对象一个 `\n` | 控制意图、离散操作、健康检查、订阅状态 |
| daemon 到电机 | Dynamixel Protocol 2.0，1 Mbps UART | 具体寄存器、位置、扭矩与传感器事务；只由总线所有者处理 |

`8765` 是 Live Twin 的 HTTP 端口，不是 `robotd` RPC 端口。
也不要把旧原型的 UDP `9872` 等端口套用到这里。
浏览器不能直接连接 Unix socket，需要板上的 HTTP/SSE 或 WebSocket 转发后端。

线格式示例，每个对象后都需要真正的换行：

```json
{"jsonrpc":"2.0","id":1,"method":"hello","params":{"api_version":28}}
{"jsonrpc":"2.0","id":2,"method":"robot.health","params":{}}
{"jsonrpc":"2.0","id":3,"method":"robot.subscribe","params":{"hz":10}}
```

请求带 `id`，用相同 `id` 对应响应。状态流与连续意图使用没有 `id` 的 notification。
客户端必须能处理响应与通知交错、拆包/粘包、断开重连，以及 JSON-RPC `error`。
本仓库请求行上限是 64 KiB。当前源码严格拒绝未知参数成员；API 数字差异本身不作全局拒绝，
但不代表两份程序的方法与字段相同。记录 `hello` 的版本，并处理具体方法不支持的响应。

## 3. 头部与颈部接口

### `robot.head`：四个关节的意图

```json
{"jsonrpc":"2.0","method":"robot.head","params":{"neck_pitch":0.3491,"head_pitch":0.3491,"head_yaw":0.1,"head_roll":0.0}}
```

这是**有潜在运动效果的消息示例**，本次没有执行。所有值都是模型空间的弧度，不是度数、编码器 tick、
PWM 或电流。四个字段构成完整头颈意图，建议每次全部给出：当前 `HeadParams` 的缺省字段为 0，
省略一个字段不表示“保留其上一次值”。未知字段会被当前源码拒绝。

| 字段 | 本仓库 motor ID | `joints` / `targets` 的零基索引 |
|---|---:|---:|
| `neck_pitch` | 30 | 5 |
| `head_pitch` | 31 | 6 |
| `head_yaw` | 32 | 7 |
| `head_roll` | 33 | 8 |

连续操作使用长连接，按 20–50 Hz 发送 notification，不必为每帧等待响应。
服务端也接受带 `id` 的同名请求，但 `accepted=true` 仅表示意图已写入，并非舵机到位。
停止变化的姿态不是必须靠网络不断刷新才能保持：当前实现的 deadman 只把底盘速度清零，
不会清除头部意图，也不会因这个客户端断线自动卸力。
这与 Live Twin 独立控制模式的“浏览器租约失效就卸力”不同。

头与底盘速度是两个独立的最后写入者生效槽。`padd` 的头控模式、其他客户端或行为可能同时写头部，
因此应先约定谁是当前头部意图的生产者。不要通过反复覆盖彼此的消息来实现控制权仲裁。
只读观察者无需争用这个控制权。

### `robot.look`：指定看向哪里

```json
{"jsonrpc":"2.0","id":10,"method":"robot.look","params":{"x":1.0,"y":0.2,"z":0.0,"neck_pitch":0.3491}}
```

`x/y/z` 是躯干坐标系下的米，X 向前、Y 向左、Z 向上；不是地面坐标系。
`neck_pitch` 是指定的颈部姿态，IK 保留它，主要求解头部俯仰和偏航。
响应的 `head` 是求得的四关节意图，`clamped` 表示受可达性或行程限制。
返回求解成功不代表实物相机已经对准目标。

同一接口也有 CLI 入口，例如下面的命令会设置注视意图，**不是只读查询**：

```sh
robotctl robot look 1 0.2 0 --neck-pitch 0.3491 --json
```

当前 `robotctl robot` 没有对应的四轴 `head` 子命令；四轴连续控制使用 JSON-RPC 客户端。
嘴部单独使用 `robot.mouth` 的 `open`（0–1）意图，不属于上述四个字段。

### 意图为什么可能被接收，却不产生运动

当前源码的路径为：`Intents.set_head` → 每 tick 取值和平滑 → `PolicyCommand.head`
→ 策略 observation 的四个头部 command 分量 → 策略产生最终关节 targets。
它没有把头部意图再次直接叠加到策略输出上。

策略需要启用、加载成功，且控制循环有可用传感器并处于允许驱动的状态，才会执行这条策略路径。
普通无策略保持分支使用已持有的 `hold`，不会因为 `robot.head` 被接受而独立移动头颈。
`robot.look` 也会落到相同的意图槽。

因此，若目标是**不运行步态策略、腿保持原姿态、仅精确操作头颈**，需要另行在 `robotd`
控制循环内实现明确的头颈直接控制分支，包括目标范围、速度/加速度、控制权与退出行为；
复用现有标定和安全 I/O 路径。该功能目前未实现。
不能为了让头动而隐式启动全身步态，也不能从外部绕开 daemon 直接写四个 motor ID。

### 容易混淆的操作

| 操作 | 含义 |
|---|---|
| `robot.init` / `robotctl robot init` | 请求正在运行的 daemon 给所有关节上力并回 HOME；不需要步态策略，但不是仅头颈操作 |
| `robot.enable` | 启用/禁用策略；可能引发全身行为，不是四轴舵机开关 |
| `robot.stop` | 速度意图归零，不是卸力 |
| `robot.relax` | 全关节卸力，支撑姿态会消失 |
| 独立命令 `robotd init` | 自己打开 UART，要求正式 daemon 已停止 |
| Live Twin 的 HOME/回零 | 独立受监护的固定姿态控制，使用临时 Mode 4；不是上表普通 daemon RPC 的别名 |

## 4. 如何只读查看 robotd

现有可用入口是 `robotctl monitor`，它向 daemon 订阅，不再打开电机串口：

```sh
ssh root@10.4.1.139 'robotctl monitor --json --hz 10'
```

当前板子的 CLI 已确认支持这个形式。不过本次运行状态没有有效传感器帧，
订阅接受后不出帧是已观察到的现象，应先看 `robot.health`，不能由客户端伪造读数。

下面是可在板上执行的最小**只读** RPC 观察示例；不会启用策略或电机：

```python
import json
import socket

with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as sock:
    sock.settimeout(3)
    sock.connect("/run/robotd.sock")
    with sock.makefile("rwb", buffering=0) as stream:
        def send(request_id, method, params):
            message = {"jsonrpc": "2.0", "id": request_id,
                       "method": method, "params": params}
            stream.write((json.dumps(message) + "\n").encode())

        send(1, "hello", {"api_version": 28})
        send(2, "robot.health", {})
        send(3, "robot.subscribe", {"hz": 10})
        try:
            while True:
                line = stream.readline(65537)
                if not line:
                    break
                if len(line) > 65536:
                    raise ValueError("oversized IPC frame")
                message = json.loads(line)
                if message.get("method") == "robot.state":
                    state = message["params"]
                    print("measured", state.get("joints"),
                          "targets", state.get("targets"), flush=True)
                else:
                    print(message, flush=True)  # 包括 hello、health、订阅确认与 error
        except TimeoutError:
            print("3 秒没有新的 IPC 消息；检查刚才的健康响应和 robotd 日志")
```

`robot.state` 中各数据不能互换：

| 字段 | 应如何使用 |
|---|---|
| `head` | 平滑后的头部 command/意图，**不是头部实测角度** |
| `joints[5:9]` | 四个头颈关节的实测模型角度 |
| `targets[5:9]` | 对应下发目标，用于观察追踪误差 |
| `policy`、`safety`、`loop` | 当前行为、限制与循环状态，不等价于逐电机 Torque Enable 寄存器 |
| `frames`、`skeleton`、`t_ns` | 新版可选信息；旧版可能没有，不能假设存在 |

完整 15 路顺序以协议 `JOINT_NAMES` 为准。支持 `robot.model` 的版本可以获取模型名称顺序；
当前板子不支持该方法，适配器应明确使用与运行版本核对过的顺序并检查向量长度，不能猜测索引。

## 5. Live Twin 并行查看需要怎样改

当前服务配置有 `Conflicts=robotd.service`；`server.py` 会检查 `robotd` 已停止，
然后通过 `ReadBus` 独占 `/dev/serial0`。它的控制 worker 也与正式 daemon 互斥。
**直接 `systemctl start microduck-twin` 可能停止当前 `robotd`，不是无影响的查看操作。**
`--offline` 只显示模型，不是 daemon 的实时遥测。

建议增加一个独立、默认只读的 IPC 模式，保留现有串口诊断模式：

1. 后端只连接 `/run/robotd.sock`，发送 `hello`、`robot.health` 和 `robot.subscribe`。
   初始以 10 Hz 查看，需要时再提高；使用 daemon 的服务端降采样。
2. 用 `joints` 驱动模型，用 `targets` 绘制目标/追踪误差，浏览器沿用 HTTP/SSE。
   该模式不创建 `ReadBus`、`Controller`、姿态 worker，也不修改电机模式。
3. 实测角度已是 daemon 的模型坐标，不能再次应用工装零位、编码器取模或拼造 raw tick。
   如果沿用现有闭嘴为 0 的显示约定，只有嘴部需显式做
   `q_display = q_model - MOUTH_CLOSED`，即模型 -5° 对应显示 0°。
   前提是实际 daemon 的标定和模型约定已经确认。
4. 状态流没有的逐电机电流、温度、原始编码器和扭矩位显示为未知。
   不可用 `gain`、`policy="held"` 或 socket 已连接来虚构这些测量值。
   单独标识连接状态、数据新鲜度与 daemon 健康状态；断流后保留最后姿态但明确标注过期。
5. 使用不含串口互斥和控制权限的单独 viewer 服务；不要直接删除现有串口模式的互斥保护。
   浏览器刷新、关闭或 viewer 崩溃只结束订阅，不能触发 `robot.relax` 或影响正式控制。
6. 验证查看器启动、关闭、重连时 `robotd` PID 不变、无新增 UART 持有者，控制循环和数据都正常；
   再在有支撑的运动中核对模型追踪。当前总线未就绪，因此这一项尚不能做实机通过结论。

新地址也不在旧 Live Twin 默认的 USB 控制 Host/网段列表中。不要为查看而放开所有写接口。
未来只读 viewer 可监听板上 loopback，再通过 SSH 转发 HTTP 到电脑；监听范围和控制授权分别处理。

## 源码依据与开发入口

- [协议类型、方法名与字段](../../duck-ipc-proto/src/lib.rs)：`HeadParams`、`LookParams`、`SubscribeParams`、`RobotState`。
- [daemon 请求与控制循环](../../robotd/src/main.rs)：`apply_intent`、`dispatch`、`driving`、状态发布。
- [意图槽和时间语义](../../robotd/src/intents.rs)：`set_head`、`snapshot`、`twist_age`。
- [策略 observation](../../duck-control/src/obs.rs) 与 [deadman](../../duck-control/src/safety.rs)。
- [模型关节与嘴部约定](../../duck-control/src/model.rs)、[注视 IK](../../kinematics/src/head.rs)。
- [现有 Live Twin 维护说明](../../scripts/live_twin/README.md)。
- [Agent 开发交接](../project/microduck-agent-handoff.md)：完成项、部署差异、后续工作与边界。
