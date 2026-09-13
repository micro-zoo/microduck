# Motor installation zeroes

`robotd` reads a per-robot file once at startup and applies its offsets at the physical
Dynamixel I/O boundary. `robotctl`, policies, WBC, kinematics and the state stream continue
to use model radians. Neither `robotctl` nor the policy adds another compensation.
This feature changes no IMU bias, orientation, axes or mounting transform.

## Configuration

After deploying a daemon that supports this setting, configure the robot's existing
`/etc/robot/robotd.toml` through `sudo robotctl configure`, or edit its `[bus]` section:

```toml
[bus]
port = "/dev/serial0"
calibration = "/etc/robot/joint-zero.json"
```

The port above is an Orange Pi example; preserve the robot's actual port. In the editor,
`bus.calibration` is an optional path. Saving it offers a **robotd restart**, not a policy
reload. The JSON file must already exist on the robot. Changing its contents also needs
a restart. A missing or malformed named file refuses startup before opening the motor bus.

The file contains encoder counts at model angle zero, for example (illustrative values):

```json
{"joints":[{"name":"head_yaw","id":32,"zero_tick":2176.0}]}
```

Unlisted joints retain the original encoder-2048 zero. An absent setting or explicit `none`
disables compensation. A configured file with a wrong name/ID pair, duplicate joint, unknown
field or non-finite/out-of-range count is rejected. Per-robot measurements belong on that
robot and in its calibration records, not as defaults in shared source code.

`robotctl configure --list` shows the saved path. `journalctl -u robotd` reports the loaded
joint names. After a real hardware startup, `robotctl monitor` displays corrected joint
positions directly; it does not need a second switch. `--fake` and `--sim` already speak
model angles: the file is validated, but physical encoder offsets are not applied to them.

## Measurement and target conversion

```text
radians_per_tick = 2*pi / 4096
q_model = (present_tick - zero_tick) * radians_per_tick
raw_target = zero_tick + q_model_target / radians_per_tick
```

The same conversion covers startup position reads, periodic feedback, target writes and
standalone `robotd init`. Velocity and current are unchanged. There is no EEPROM Homing
Offset write. The runtime refuses corrected readings outside its model travel and refuses
shifted targets outside the single-turn encoder range; it never wraps a position command.

Configured motors must be XL330-M288, Drive Mode 0, Operating Mode 3, Homing Offset 0 and
position limits 0..4095. These are read and checked before startup EEPROM corrections.
Automatic replacement adoption is refused for a calibrated joint: a fresh motor's matching
ID and model do not prove it has the removed motor's installation zero. Configure and
recalibrate that replacement before starting the calibrated daemon again.

## Closed-mouth fixture reference

The current robotd mouth API is -5 degrees closed and +30 degrees fully open. A fixture
capture that defines **closed as zero** must be converted before use by this runtime:

```text
mouth_runtime_zero_tick = measured_closed_tick - (-5*pi/180) / radians_per_tick
```

This makes the existing closed command return to the measured physical closed position.
It does not validate the full-open mechanical limit. The read-only web twin can continue
to display closed=0, while robotctl's model angle at the same position is -5 degrees.
Use `joint-zero.runtime.json` from the fixture exporter, not its unconverted
`joint-zero.fixture.json`. ID swaps must migrate the original zeroes with the physical
motors before this export; do not recapture an arbitrary current pose as zero.

## Before movement

A repeatable fixture capture establishes an offset, not physical travel or IMU correctness.
Check the mechanical range and signed movement against the model before enabling control.
An encoder zero near the 0/4095 seam can make part of the desired model range unreachable;
software offsetting does not extend the position mode. Keep the robot supported and control
stopped until that working range is resolved. The independent read-only twin can continue
running during this work.
