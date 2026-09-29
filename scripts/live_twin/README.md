# Live Twin telemetry and gamepad pairing

`robotctl twin` manages a web viewer for the joint state published by
`robotd`. The Python bridge connects to `/run/robotd.sock`, requests `hello`,
`robot.health` once a second, and `robot.subscribe`. It also subscribes to
`head_imu.stream` on `/run/tofd/tof.sock`. The page shows the battery voltage
and percentage already calculated by `robotd` from its 6.6–8.2 V loaded-pack
range; it does not calculate a second percentage in the browser. The battery
reading is a voltage estimate, not a separate fuel gauge, and disappears when
the health sample is stale. The page also shows the SoC temperature,
all 15 servo case temperatures from `robotd`'s existing slow sample, the trunk
IMU attitude, and the head BMI088 attitude when enabled. The two IMUs retain
their own reference frames and arbitrary yaw origins; no mount correction is
applied in the viewer. The bridge serves the bundled Three.js model and
state events on port 8765 of the robot's network interfaces. The 3D trunk follows
`robot.state.imu.quat` after `robotd` applies the per-robot `[body_imu]` mount.
The viewer anchors the first yaw to its initial heading because game-rotation
yaw has no absolute north; subsequent relative yaw and measured tilt remain
live. No IMU mount transform is kept in the viewer, and trunk translation stays
fixed. It never opens the
motor serial port or sends a robot intent. The only HTTP write route is
`POST /api/pad/pair`, which invokes the existing `robotctl pad pair --json`
command. It accepts no MAC address or other parameters, and concurrent pairing
requests are refused. `GET /api/pad/status` reports the driver and pad state
without exposing Bluetooth addresses. Other HTTP write requests return 405.

```sh
robotctl twin
robotctl twin status
sudo robotctl twin enable
sudo robotctl twin restart
sudo robotctl twin disable
```

`robotctl twin` and `robotctl twin status` list the robot's current IPv4 URLs;
open one directly from the same network, such as `http://ROBOT_IP:8765/`.
The HTTP viewer has no login. Anyone who can open it on the LAN can see telemetry
and press **开始配对** after putting an Xbox pad in pairing mode. The write route
requires a same-origin browser request using the robot's LAN IP address; it
does not accept requests sent by unrelated web pages through a visitor's browser.
Use this page on a trusted network. Pairing uses `configd` and BlueZ; `padd`
reads the resulting input device when its service is active. The page shows the
current `padd` state, so pairing success is not presented as motor control when
the driver has been deliberately stopped. It refreshes pad status every three
seconds: an unpaired pad offers **开始配对**, a bonded but disconnected pad offers
**重新配对**, and a connected pad shows **已连接** without a pairing button. The
viewer can start while
`robotd` is stopped; it reports offline until `robotd` supplies state. Enabling
the viewer does not start `robotd` because its service has only `After=robotd`.

Joint values are the coordinates reported by `robotd`. They are not a fresh
calibration of a physical fixture at q=0. On a robot without installed joint
zeroes, the 3D pose may differ from its physical fixture pose. Load physical
zeroes through [`robotd`'s calibration setting](../../docs/robot/joint-calibration.md),
not in this viewer. The visual mouth hinge alone adds 5 degrees to match the
mesh's closed-mouth reference. The summary shows the largest visual deviation
from model q=0; the joint list retains raw `robotd` model angles, including the
closed mouth's −5°. Values
that `robotd` does not publish, including torque and raw encoder ticks, display
as unknown. A stale stream loses its live indication and holds its last pose.
The head IMU is off by default; enable `[head_imu] enabled = true` in
`/etc/robot/robotd.toml` and restart `tofd` to make its attitude live. When it
is off or unavailable, the card shows that state rather than a zero angle.
On the Orange Pi Zero 3W bench robot, BMI088 is on `/dev/i2c-0`, while the
generic `tofd` defaults look for the HAT bus at `/dev/i2c-pihat` or
`/dev/i2c-3`. A board-specific systemd drop-in sets `tofd --bus /dev/i2c-0
--imu-hz 25`; the experimental full HAT device-tree overlay is not needed.
Keep that bus override specific to boards whose physical I²C wiring has been
verified.

The robot telemetry and model remain read-only. The page has no HOME, zero,
relax, WBC, or motor control route. The old experimental control page remains
on the archived Git branch and is not part of this service.

The release package includes `ipc_server.py`, the complete `dist/` tree, and
the service wrapper. The wrapper resolves Python and assets through
`/opt/robot/daemon/current`, so both must come from the same installed build.
For an offline preview on a development machine, run
`python3 scripts/live_twin/ipc_server.py --port 8876` and open
`http://127.0.0.1:8876/`.
