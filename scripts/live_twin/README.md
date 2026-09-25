# Read-only Live Twin

`robotctl twin` manages a local web viewer for the joint state published by
`robotd`. The Python bridge connects to `/run/robotd.sock`, requests `hello`,
`robot.health`, and `robot.subscribe`, and serves the bundled Three.js model and
state events at `127.0.0.1:8765`. It never opens the motor serial port or sends a
robot intent. HTTP write requests return 405.

```sh
robotctl twin status
sudo robotctl twin enable
sudo robotctl twin restart
sudo robotctl twin disable
ssh -L 8765:127.0.0.1:8765 USER@ROBOT
```

Open `http://127.0.0.1:8765/` through the SSH tunnel. The viewer can start while
`robotd` is stopped; it reports offline until `robotd` supplies state. Enabling
the viewer does not start `robotd` because its service has only `After=robotd`.

Joint values are the coordinates reported by `robotd`. They are **not** a fresh
calibration of a physical fixture at q=0. On a robot without installed joint
zeroes, the 3D pose may differ from its physical fixture pose. The visual mouth
hinge alone adds 5 degrees to match the mesh's closed-mouth reference. Values
that `robotd` does not publish, including torque and raw encoder ticks, display
as unknown. A stale stream loses its live indication and holds its last pose.

The page is limited to telemetry and model rotation. It has no HOME, zero,
relax, WBC, or other control route. The old experimental control page remains
on the archived Git branch and is not part of this service.

The release package includes `ipc_server.py`, the complete `dist/` tree, and
the service wrapper. The wrapper resolves Python and assets through
`/opt/robot/daemon/current`, so both must come from the same installed build.
For an offline preview on a development machine, run
`python3 scripts/live_twin/ipc_server.py --port 8876` and open
`http://127.0.0.1:8876/`.
