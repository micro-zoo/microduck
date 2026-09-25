# Joint zeroes on a physical robot

`robotd` can load a per-robot joint-zero file through `[bus] calibration` in
`/etc/robot/robotd.toml`. `robotctl configure` exposes that setting and offers a
`robotd` restart when it changes. The file stays on the robot, outside Git.

## Capture while the robot is in its q=0 fixture

```sh
robotctl calibrate zero --fixture-q0 --output /root/joint-zero-candidate.json
```

The explicit fixture flag is a statement about the **physical pose**; software
cannot infer that from an encoder. `robotctl` asks the running `robotd` for the
zeroes it actually loaded and the motor setup it read at startup, then subscribes
to 40 fresh state frames at 10 Hz. It reverses the loaded coordinate shift before
calculating new encoder zeroes, so this also works when an older calibration is
already active. It checks that the policy is off, no HOME pose is powered, the
motors use supported single-turn position settings, and each joint stayed within
two encoder ticks during capture.

The command **only creates a new candidate file**. It refuses to overwrite an
existing file, never opens the motor serial port, and does not change
`robotd.toml`, torque, EEPROM or the currently loaded zeroes. Review the result
before installing it as `/etc/robot/joint-zero.json` and restarting `robotd`.

```toml
[bus]
calibration = "/etc/robot/joint-zero.json"
```

Each entry names the joint, its Dynamixel ID, and the encoder count that means
zero model radians. All 15 joints can be listed. For the mouth, the runtime's
closed position is **−5°**: if the fixture holds it closed at `closed_tick`, set
`zero_tick = closed_tick + 56.8888888889` so the closed pose reports −5°.
If a motor already has a nonzero Homing Offset in EEPROM, record its existing
count as `homing_offset_tick`. `robotd` checks that count at startup and does not
change it. `zero_tick` comes from the reported Present Position, which already
includes that hardware offset; do not add or subtract it a second time. A
changed hardware offset would otherwise make the saved zero wrong.

```json
{"joints":[{"name":"left_knee","id":23,"zero_tick":1976.0,"homing_offset_tick":-585}]}
```

Capture zeroes only while the physical robot is held in a known q=0 fixture.
Do not copy zeroes from another motor installation. A prior file can be wrong
after an ID swap, assembly change or encoder setup change.

The setting is read once at startup. A named missing or invalid file prevents
`robotd` from opening the bus. Check `journalctl -u robotd` for `loaded joint
zeroes`, then use `robotctl monitor` or `robotctl twin` to inspect corrected
positions. The twin remains a read-only consumer of `robotd` state and needs no
separate offset file or motor connection.

This establishes a position reference. It does not validate travel, torque,
IMU mounting, or a walking policy. Keep physical support and the existing
`--no-policy` commissioning configuration until those are checked separately.

## EEPROM and turn count

The XL330's [Homing Offset](https://emanual.robotis.com/docs/en/dxl/x/xl330-m288/)
is persistent EEPROM and shifts the reported position, but it does **not** save
the number of turns. The same manual says Present Position resets to a
single-turn absolute position on power-up, on a change to position mode, and
when torque is turned on in position mode. Writing the captured count to EEPROM
would also change the live position reference and require coordinated changes to
this file; it is deliberately outside the capture command. EEPROM writes require
torque OFF. Resolve a multi-turn discontinuity as a separate position-mode and
mechanical-range problem rather than hiding it in Homing Offset.
