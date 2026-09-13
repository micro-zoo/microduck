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
shifted targets outside the selected encoder range; it never wraps a position command.

Configured motors must be XL330-M288, Drive Mode 0 and Homing Offset 0. Their
Operating Mode must match the calibration file: 3 by default, or 4 for extended position.
Mode 3 also requires position limits 0..4095. These are read and checked before startup EEPROM corrections.
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
An encoder zero near the 0/4095 seam can make part of the desired model range unreachable
in Mode 3. Use the explicit extended-position configuration below to span that seam. Keep the robot supported and control
stopped until that working range is resolved. The independent read-only twin can continue
running during this work.

## Extended position for installation zeroes near the encoder seam

XL330 Mode 4 accepts signed goal counts from -1,048,575 to +1,048,575. Its hardware
Min/Max Position Limits are not applied, and power-on/reboot resets the position count
to a single-turn phase. See [ROBOTIS Operating Mode and Goal Position](https://emanual.robotis.com/docs/en/dxl/x/xl330-m288/).
Multi-turn motor control does not make a bounded robot joint safe to rotate repeatedly.

Export all 15 measured zeroes with the current model's joint limits:

```sh
python3 scripts/export_joint_zero.py /path/to/fixture/calibration.json /path/to/joint-zero.extended.json --extended
```

By default the exporter reads this repository's Alpha MJCF. `--model PATH` accepts another
MJCF or URDF with matching joint names and explicit position limits. The mouth uses the
runtime's existing -5..30 degree convention. Each robot supplies its own encoder zeroes;
the model limits and policy coordinates stay the same. A complete export has this shape
(one illustrative joint shown):

```json
{
  "position_mode": "extended_position",
  "joints": [
    {"name":"left_hip_yaw","id":20,"zero_tick":4071,
     "limits_rad":[-0.4363323129985824,0.5235987755982988]}
  ]
}
```

`bus.calibration` points at this file. The default mode remains `single_turn` for old files.
The daemon checks the actual motor mode but does not change it automatically at startup.
Only configured joints are affected; the complete fixture export configures all fifteen.

For Mode 4, each `limits_rad` interval must be finite, lie inside the runtime's -pi..pi
model domain and span less than one revolution, with room for encoder quantization.
On the first valid position read, find the unique integer `n` for which the observed
model angle lies in that interval:

```text
session_zero = zero_tick + 4096*n
q_model = (raw_position - session_zero) * 2*pi/4096
raw_goal = session_zero + q_model_goal * 4096/(2*pi)
```

That `session_zero` stays fixed during operation. Goals are continuous signed counts;
there is no per-command modulo. For example, zero 4071 with a +30 degree model target
produces a goal above 4095. After reboot at that same physical pose, the newly reported
phase can instead require session zero -25; the policy still sees the same +30 degrees.

The first read must establish a valid origin before any target is accepted. Torque enable
preloads the measured hold position before enabling any motor, so a retained old goal is
not chased. An explicit motor reboot invalidates its origin and waits for reboot settling
before reading it again. An unexpected count reset that puts the reading outside the model
interval is rejected; the running origin is never silently reinterpreted. Stop and restart
or explicitly reboot/re-read to establish it again.

Readings and targets are checked against model limits with one encoder-count tolerance.
True continuous joints, or a model interval as wide as a full turn, need homing or another
absolute reference after power loss; this phase-based recovery deliberately refuses them.
This conversion also assumes the same mechanism, joint direction and transmission ratio
as training. It does not compensate different dynamics, backlash, control latency or IMU
mounting, and is not itself a policy's hardware qualification.

## Torque-off firmware verification

The maintenance tool reuses the installed `servo_config.py` transport, including exclusive
UART access and durable operation logs. Stop both serial consumers first, keep every motor
torque OFF, and use the exported extended-position file:

```sh
python3 scripts/configure_extended_position.py /path/to/joint-zero.extended.json --protocol-dir /root/calibration
```

The default above is read-only. `--apply --restore-original-mode` temporarily switches each
motor to Mode 4, reads the mode back, restores mode-reset PID/profile/current/PWM settings,
then restores the original mode and tuning. The final goal is checked against the current
position. This verifies mode selection and restoration, not powered tracking or clearance.

`--probe-goals` additionally attempts strict goal-register readback at the model limits.
On the tested XL330 firmware 53, while torque was OFF, Goal Position followed Present
Position instead of retaining a different written goal. That probe was inconclusive and
aborted; it did not prove signed goal acceptance. Restoration now runs even on that failure.
A supported powered test is required to verify real tracking across the encoder seam.

`--apply` without restoration leaves Mode 4 installed. Use that only together with deployment
of this calibrated runtime and its matching JSON. Keep the old uncalibrated daemon stopped:
its encoder-2048 mapping does not become correct merely because the servo accepts multi-turn
goals. The utility never enables torque or starts a service; if it stops on an error, inspect
the journal and reconcile the actual modes before starting either controller.

Tests sweep all 4096 installation phases, cross 4095 continuously, reject an unexpected
counter reset, re-establish the correct origin after an explicit reset and reject ambiguous
or out-of-range model coordinates. IMU configuration remains outside this feature.
