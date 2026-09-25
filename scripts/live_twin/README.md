# Read-only Live Twin

`robotctl twin` manages a web viewer for the joint state published by
`robotd`. The Python bridge connects to `/run/robotd.sock`, requests `hello`,
`robot.health` once a second, and `robot.subscribe`. It also subscribes to
`head_imu.stream` on `/run/tofd/tof.sock`. The page shows the SoC temperature,
all 15 servo case temperatures from `robotd`'s existing slow sample, the trunk
IMU attitude, and the head BMI088 attitude when enabled. The two IMUs retain
their own reference frames and arbitrary yaw origins; no mount correction is
applied in the viewer. The bridge serves the bundled Three.js model and
state events on port 8765 of the robot's network interfaces. It never opens the
motor serial port or sends a robot intent. HTTP write requests return 405.

```sh
robotctl twin
robotctl twin status
sudo robotctl twin enable
sudo robotctl twin restart
sudo robotctl twin disable
```

`robotctl twin` and `robotctl twin status` list the robot's current IPv4 URLs;
open one directly from the same network, such as `http://ROBOT_IP:8765/`.
The HTTP viewer has no login and exposes read-only telemetry to hosts that can
reach that port, so use it on a trusted network. The viewer can start while
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

The page is limited to telemetry and model rotation. It has no HOME, zero,
relax, WBC, or other control route. The old experimental control page remains
on the archived Git branch and is not part of this service.

The release package includes `ipc_server.py`, the complete `dist/` tree, and
the service wrapper. The wrapper resolves Python and assets through
`/opt/robot/daemon/current`, so both must come from the same installed build.
For an offline preview on a development machine, run
`python3 scripts/live_twin/ipc_server.py --port 8876` and open
`http://127.0.0.1:8876/`.
