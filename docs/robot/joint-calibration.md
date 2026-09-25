# Joint zeroes on a physical robot

`robotd` can load a per-robot joint-zero file through `[bus] calibration` in
`/etc/robot/robotd.toml`. `robotctl configure` exposes that setting and offers a
`robotd` restart when it changes. The file stays on the robot, outside Git.

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
With calibration **not yet enabled**, a `robotd` joint position `q` in radians
corresponds to `round(2048 + q × 4096 / (2π))` encoder ticks. Capture multiple
independent `robot.subscribe` frames and confirm that the values are stable
before creating the file. Do not use this inverse on an already calibrated
state stream or copy zeroes from another motor installation. A prior calibration
file can be wrong after an ID swap, assembly change, or encoder setup change.

The setting is read once at startup. A named missing or invalid file prevents
`robotd` from opening the bus. Check `journalctl -u robotd` for `loaded joint
zeroes`, then use `robotctl monitor` or `robotctl twin` to inspect corrected
positions. The twin remains a read-only consumer of `robotd` state and needs no
separate offset file or motor connection.

This establishes a position reference. It does not validate travel, torque,
IMU mounting, or a walking policy. Keep physical support and the existing
`--no-policy` commissioning configuration until those are checked separately.
