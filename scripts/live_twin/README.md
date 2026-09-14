# Microduck Live Twin

The existing joint telemetry and 3D view, with three supported controls:

- **HOME** moves to the model's default pose and holds it.
- **回零** moves to the captured fixture zero and holds it. The mouth stays closed;
  its runtime coordinate is -5 degrees, corresponding to zero on this page.
- **卸力** disables torque, restores the original operating modes and tuning, and
  returns the UART to the telemetry reader.

Support the trunk with the head, neck and legs free to move. These are supported
pose operations, without an IMU or walking policy. Preparation currently takes
less than 5 seconds on the tested board to change modes and verify the original
settings. Its progress
is shown as `准备电机 n/15`; UART handoff is not displayed as a motor disconnection.
Cancelling also waits for the original settings to be restored. An unloaded neck can fall again.
Arrival requires every joint within 2 degrees for one second, with no more than
0.5 degrees of encoder movement within that second. Interactive motion uses a
synchronized trapezoidal ramp bounded at 20 degrees/s and 40 degrees/s².
A typical HOME/zero switch has about 2 seconds of planned travel,
followed by the existing one-second settling check. A deeply slumped neck requires
a longer path. The standalone guarded diagnostic retains its original slow ramp.

Only the page that started the session can change its held pose. Other authorized
pages can still unload. Closing, reloading, disconnecting or suspending the owning
page ends its lease. A separate guardian requires a browser heartbeat within
2 seconds and actual Rust control-loop progress within 0.5 seconds. On expiry it
kills the goal-producing process through a Linux pidfd before writing torque-OFF
packets. This is also checked when the native process is stopped or killed.
The servo watchdog, load, voltage, temperature, tracking and motion checks remain
active while holding. A fault ends the session; it never resumes automatically.

## Runtime layout

Python 3.11+ and systemd run on the Linux controller. `robotd` must be built from
this source with `init --guarded --interactive` support. The web process requires
root access to the existing serial transport and local systemd manager. Pose
workers run in separate transient units, with `ExecStopPost` reconciliation, so a
web-server crash does not remove their independent guardian. Restarting the web
server stops and recovers the previous transaction before reading the UART.

`servo_config.py` is the installed hardware maintenance transport, selected with
`--protocol-dir`. It provides serial ownership, kernel exclusivity, CRC parsing and
journaling. The sibling `export_joint_zero.py`, `configure_extended_position.py`,
`run_guarded_home.py` and `probe_ankle_position.py` must accompany this directory.
The captured `calibration.json` belongs to the particular robot and stays outside
Git. Original settings and operation records are saved in `--control-runs`.

The existing STL assets and Three.js distribution are reused from the original
Live Twin deployment; they are not duplicated here. `--assets-dir` must contain
the existing `assets/` and `vendor/` directories. `dist/assets/model.json` and the
model license describe the view. Its mouth hinge remains a visual approximation.

For a local, unpowered preview:

```sh
python3 scripts/live_twin/server.py --offline --http-port 8876 \
  --assets-dir /absolute/path/to/existing/dist \
  --calibration /absolute/path/to/captured/calibration.json
```

The board launch additionally accepts `--robotd`, `--model-source`, `--model-xml`,
`--serial-port`, `--protocol-dir`, `--listen` and `--control-runs`. Control requests
are limited to loopback and the configured USB subnet, with exact Host/Origin
checks and a per-page capability. Add an explicit `--control-host` if accessing
through a different local hostname. Do not expose this debug service to the public
Internet. The API accepts only HOME, zero, unload and heartbeat; it accepts no
custom joint positions or output parameters.

## Verification

```sh
python3 -m unittest discover -s scripts/live_twin/tests -p 'test_*.py'
node --test scripts/live_twin/tests/*.test.mjs
```

`scripts/run_calibrated_control_path.py` runs native HOME/zero hold, retarget,
unload, browser-lease loss, control-loop stall and producer-death checks inside
the existing unprivileged systemd sandbox. Its private PTY bus has ideal emulated
motors; these checks establish control behavior, not physical tracking accuracy.
Real supported motion must be verified separately with the robot supported.
Three consecutive empty/failed telemetry reads close and reopen the UART, so a
failed read handoff cannot leave the page indefinitely showing old samples.

Unicast setup transactions finish after complete validated replies and a 2 ms
quiet interval, retaining the original timeout for delayed/missing replies. The
existing parser still rejects CRC errors and truncated frames; extra replies are
returned for the existing ID/count checks. Device ownership is still scanned in
full before every write, using `os.scandir` to avoid repeated Path allocations.
Mode writes retain their settling delay. RAM writes are read back immediately;
an already correct current-position goal is verified without rewriting it.

## Maintenance

| Change | Source |
|---|---|
| Page layout, controls and measured pose display | `dist/`, especially `controls.js`, `app.js` and `state.js` |
| HTTP authorization, session ownership and UART handoff | `server.py` and `control.py` |
| Preparation, original-setting recovery and process supervision | `pose_worker.py`, `guardian.py`, and the sibling calibration helpers |
| Unicast response completion and device-owner scanning | `reply_transport.py`; framing and device locks remain in the installed `servo_config.py` |
| Motion profiles, arrival checks and native stop handling | `duck-control/src/bus/homing.rs` and `robotd/src/pose_session.rs` |

The native paths above are relative to the repository root. Run relevant checks
from that root, using Python 3.11+:

```sh
cargo test --locked -p duck-control --lib bus::homing
python3 -m unittest discover -s scripts/tests -p 'test_*.py'
python3 -m unittest discover -s scripts/live_twin/tests -p 'test_*.py'
node --test scripts/live_twin/tests/*.test.mjs
```

For a board update:

1. Finish the active session and confirm all motors OFF. Record the service's
   current launch arguments and retain its previous source and native executable.
2. Stop the web service before replacing its Python/JavaScript files and required
   sibling helpers. Preserve that robot's calibration, existing mesh/vendor assets,
   and run records. Build and deploy a matching native executable when motion or
   local pipe handling changes; check the service's actual `--robotd` argument.
3. Restart the service and verify 15 fresh readings, unload, preparation progress,
   cancellation and restoration. Validate motion separately with physical support.
   Refresh open pages to load changed JavaScript. For rollback, stop the session
   and restore the previous service arguments and matching source/executable.

Operation evidence is in each `--control-runs` directory: `before.json`,
`preparation.json`, `telemetry.jsonl`, `robotd.log`, `guardian.json`, `after.json`
and `result.json` where produced. An accepted HTTP request is not proof of arrival;
use the measured arrival event and the final restoration result.

The acceleration update was checked on 2026-09-14: preparation measured **3.21 s**
on the real board without enabling torque, and the native isolated suite passed
17 checks. The earlier slower web HOME/hold/unload sequence was physically tested.
The new 20 degrees/s trajectory has **not yet been physically validated**; its
travel times above are planned durations, not measured hardware completion times.
