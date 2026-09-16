# IMU and servo reads

With `bus.imu_to_dxl_enabled = true` (the default), `DynamixelIo::read()` sends one
Dynamixel Protocol 2.0 `SYNC_READ` for the IMU and all fifteen servos. The IMU,
ID 200, is first in the request, followed by `JOINT_IDS`. All devices return twelve
bytes starting at address 124. The normal control rate remains 50 Hz.

Setting the option to `false` omits the IMU from that same request and preserves
motor telemetry. Orientation stays unready, so policy driving remains gated.
An **enabled** IMU that fails to answer is an error; the reader does not silently
substitute an IMU-free sample. Restart robotd after changing this option.

The IMU and servo blocks belong to one bus transaction. This is not a guarantee
that the sensors sampled at the same instant: each device has its own internal
updates, and replies are serialized in the request's ID order. Transaction
duration alone does not measure the difference between sampling times.

See the [ROBOTIS Sync Read specification](https://emanual.robotis.com/docs/en/dxl/protocol2/#sync-read-0x82).

## Read-only board checks

`check_sync_read` calls the same `DynamixelIo::read()` as robotd. It does not call
startup register correction, torque, gain, target writes, or policy inference.
Stop robotd, padd, and any other consumer of the selected UART before running it.
In particular, `robotd --no-policy` is **not** a read-only replacement: it still
has a motor write path.

Build the daemon and the checker with the board toolchain:

```sh
cargo zigbuild --release --target aarch64-unknown-linux-gnu.2.31 \
  -p robotd -p duck-control --bins --example check_sync_read
```

Copy `target/aarch64-unknown-linux-gnu/release/robotd` and
`target/aarch64-unknown-linux-gnu/release/examples/check_sync_read` to the board.
Run these checks sequentially, replacing the serial path with the board's UART:

```sh
./check_sync_read /dev/ttyS0 600
./check_sync_read /dev/ttyS0 30 --no-imu
./check_sync_read /dev/ttyS0 30 --reopen
```

The first measures combined reads at 50 Hz. The second checks the explicit
IMU-disabled path. The third closes and reopens the serial reader every two
seconds, including reacquiring fresh IMU samples before reporting ready.
The checker reports errors, achieved rate, skipped slots, read latency,
readiness, and stale runs. It returns an error for failed reads, non-finite
decoded values, lost readiness, or the existing 25-sample stale-run threshold.
Timing figures are reported separately, rather than interpreting successful
packets as proof of meeting the requested schedule.

## Software checks

```sh
cargo test -p duck-control -p robotd-params -p robotd
```

The bus test exercises the real serial SDK over a pseudo-terminal. It checks
the combined request, joint ordering, the disabled-IMU path, missing IMU and
motor replies, a corrupt CRC, and a short IMU block. Switching IMU participation
clears the old decoder's readiness; re-enabling it requires fresh samples.

These checks qualify the read path. They do not qualify walking, motion safety,
or simultaneous physical sampling across devices.

## Board verification, 2026-09-17

Tested on the assembled Orange Pi host with fifteen XL330s and the
CH32V203/LSM6DSV16X IMU firmware `0x0207`, sharing a 1 Mbps bus:

| Read-only check | Successful reads | Errors / skipped slots | Read P99 |
|---|---:|---:|---:|
| Combined, 600 seconds | 30,001 | 0 / 0 | 7.60 ms |
| Final build, combined, 120 seconds | 6,001 | 0 / 0 | 7.61 ms |
| Final build, IMU disabled, 30 seconds | 1,501 | 0 / 0 | 7.22 ms |
| Final build, 14 serial reopens in 30 seconds | 1,501 | 0 / 0 | 7.59 ms |

The final build includes the decoder reset on IMU participation changes.
All reads finished within 20 ms. The longest stale run was one read; readiness
was retained after convergence and was reacquired after reopening. With IMU
disabled, readiness stayed false.

529 host tests passed across `duck-control`, `robotd-params`, `robotd`, and
`robotctl`; 16 existing opt-in/runtime-dependent tests stayed ignored. The 16 bus
tests also passed natively on the ARM host. The deployed daemon separately
served 300 state frames and healthy status using `--fake --no-policy` on an
isolated socket; this is IPC validation, not physical control validation.

Formatting and diff checks passed. Clippy completed with three existing style
warnings in unchanged `duck-control/src/bus/homing.rs`; strict `-D warnings`
therefore does not pass. No motor commands were sent by the physical checks,
and the real control services remained stopped.
