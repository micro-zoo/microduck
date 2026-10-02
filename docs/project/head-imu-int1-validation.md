# BMI088 INT1 validation record

Measured on 2026-10-02 with an unmodified C1 HAT, Radxa Zero 3W, Linux 6.1.84-10-rk2410-nocsf. The mechanism and field semantics are owned by [robotd-design.md](../design/robotd-design.md#head-imu-acquisition-timing-api-v41).

## Direct nominal-100-Hz comparison

Polling used unmodified upstream `1fa84386`; INT1 used the candidate based on that revision. Both were ARM64 release builds with Rust 1.99 and a glibc 2.31 target. Each had one `head_imu.stream` subscriber, acc ODR 100 Hz, gyro ODR 100 Hz, the same ranges and filter settings. The original driver's incorrect power-configuration address was handled by waking the accelerometer before the capture window; polling source was unchanged and all captures had valid nonzero acceleration.

Three-second warmup followed by a 30-second window, in P/I, I/P, P/I order. Interval deviation is calculated around each window's own mean, without forcing timestamps onto a 10 ms grid. Summary values below are medians of the three window metrics.

| Program / timestamp | Interval standard deviation | P99 absolute interval deviation from mean | Effective frequency |
|---|---:|---:|---:|
| Polling `t_ns` | 184.3 µs | 595.7 µs | 98.74 Hz |
| INT1 `t_ns` | 186.6 µs | 616.5 µs | 100.93 Hz |
| INT1 `accel_data_ready_ns` | 37.4 µs | 105.1 µs | 100.93 Hz |

| Mode | Window | Frames | `t_ns` stddev, µs | `t_ns` P99 deviation, µs | Ready-time stddev, µs |
|---|---:|---:|---:|---:|---:|
| polling | 1 | 2963 | 181.9 | 586.4 | — |
| int1 | 1 | 3032 | 186.6 | 616.5 | 35.7 |
| int1 | 2 | 3037 | 182.9 | 607.6 | 37.4 |
| polling | 2 | 2967 | 188.9 | 595.7 | — |
| polling | 3 | 2967 | 184.3 | 650.6 | — |
| int1 | 3 | 3032 | 201.0 | 747.2 | 37.4 |

There were 8,897 polling frames and 9,101 INT1 frames, with no subscriber sequence gaps and no INT1 event-sequence gaps. The original completion field did not become more uniform. The new event field had about 80% less interval variation than polling's completion field; these are different time sources. This is not a measurement of absolute sampling-centre accuracy or gyro timing.

The governor stayed `ondemand`; maximum CPU frequency was 1.8 GHz and sampled CPU cooling state was zero in every window. SoC temperature ranged from 72.8 to 78.8 °C. Receiver-time statistics were excluded from the conclusion because the Python observer also sampled environmental sysfs files.

Raw records use one JSON frame per line. For any timestamp `t`, calculate `(t[i+1] - t[i]) / 1e6` in milliseconds within a window, then use population standard deviation and the 99th percentile of `abs(interval - mean(interval))`. Do not join across daemon restarts. Original raw-frame records and the capture scripts are retained in the local review bundle; this document records every window, not only the favourable ones.

## CPU comparison (separate experiment)

Same maintained-fork binary (`94fa96a`, Rust 1.96), selecting its unchanged polling path or INT1 path; matched ODR, one subscriber, three alternating 20-second windows per condition after warmup. Percentages are daemon user+system CPU, with one logical CPU fully busy equal to 100%. They exclude the client and do not capture every kernel interrupt cost.

| Rate | Polling tofd CPU | INT1 tofd CPU | Polling CPU per frame | INT1 CPU per frame |
|---|---:|---:|---:|---:|
| 25 Hz | 2.94% | 2.85% | 1182 µs | 1124 µs |
| 100 Hz | 9.35% | 9.95% | 946 µs | 983 µs |

At 100 Hz INT1 cost about 0.60 percentage points more CPU; per-frame CPU was about 3.9% higher. This feature is not presented as a CPU or power optimisation. The experiment above used a different toolchain and source version from the timestamp comparison, so the two experiments are kept separate.

## Failure handling and service checks

- A 0.8-second pause in a 1,500-frame nominal-100-Hz run exposed a GPIO sequence gap of 82. Published frames retained ordered event/read/completion times and passed the host association checks after resuming. This does not prove absence of all physical sample loss.
- Stock HAT mapping was zero before acquisition. Review found that retaining INT1 FIFO-full/watermark bits could mislabel an event. Initialization now selects DRDY alone on INT1, preserves INT2 routing, and restores the prior mapping on exit; a Linux unit test covers the earlier-FIFO mapping case. This does not change the stock-zero mapping used in the measurements above.
- Register restoration and GPIO release were checked after cooperative shutdown. The systemd service delivered 650 frames across two stop/start cycles. Existing socket permissions and service hardening were retained.
- The maintained deployment is now explicitly configured at 100 Hz, matching the upstream acquisition default; acc/gyro ODR readback and another 300 live frames passed. Boot enablement was not changed. No robotd restart or motor command was needed for these tests.
- GPIO capture time is not a calibrated physical sampling centre. Sensor group delay, GPIO capture latency, gyro data-ready time, camera alignment and orientation error were not qualified.
