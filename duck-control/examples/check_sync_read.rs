//! Read-only exercise of the exact bus reader used by robotd.
//! Usage: check_sync_read SERIAL SECONDS [--no-imu] [--reopen]
//! Stop robotd and every other UART consumer first. This never calls init,
//! register correction, torque, gain, position writes, or policy inference.
use duck_control::{RobotIo, bus::DynamixelIo};
use serde_json::json;
use std::time::{Duration, Instant};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut args = std::env::args().skip(1);
    let port = args.next().ok_or("serial path required")?;
    let seconds: u64 = args.next().ok_or("duration in seconds required")?.parse()?;
    if !(1..=3600).contains(&seconds) {
        return Err("duration must be between 1 and 3600 seconds".into());
    }
    let mut enabled = true;
    let mut reopen = false;
    for arg in args {
        match arg.as_str() {
            "--no-imu" => enabled = false,
            "--reopen" => reopen = true,
            _ => return Err(format!("unknown argument: {arg}").into()),
        }
    }
    let open = || DynamixelIo::open(&port).map(|io| io.with_imu_to_dxl_enabled(enabled));
    let mut bus = Some(open()?);
    let period = Duration::from_millis(20);
    let start = Instant::now();
    let mut deadline = start;
    let mut last_progress = start;
    let mut reopened_at = start;
    let mut reopen_count = 0;
    let mut errors = 0;
    let mut success = 0;
    let mut skipped_slots = 0;
    let mut ready_samples = 0;
    let mut max_stale_run = 0;
    let mut ready_lost = 0;
    let mut was_ready = false;
    let mut late_reads = 0;
    let mut timings = Vec::new();
    let mut last_sensors = None;
    while start.elapsed() < Duration::from_secs(seconds) {
        std::thread::sleep(deadline.saturating_duration_since(Instant::now()));
        if reopen && reopened_at.elapsed() >= Duration::from_secs(2) {
            drop(bus.take());
            bus = Some(open()?);
            reopened_at = Instant::now();
            was_ready = false;
            reopen_count += 1;
        }
        let io = bus.as_mut().unwrap();
        let read_start = Instant::now();
        let result = io.read();
        let duration = read_start.elapsed();
        timings.push(duration.as_secs_f64() * 1000.0);
        late_reads += u64::from(duration > period);
        match result {
            Ok(sensors) => {
                if sensors
                    .positions
                    .iter()
                    .chain(&sensors.velocities)
                    .chain(&sensors.currents_ma)
                    .chain(&sensors.imu.gyro)
                    .chain(&sensors.imu.gravity)
                    .chain(&sensors.imu.quat)
                    .any(|value| !value.is_finite())
                {
                    return Err("non-finite decoded sensor value".into());
                }
                success += 1;
                last_sensors = Some(sensors);
            }
            Err(error) => {
                errors += 1;
                eprintln!("read {errors} failed: {error}");
            }
        }
        let ready = io.imu_ready();
        ready_samples += u64::from(ready);
        ready_lost += u64::from(was_ready && !ready);
        was_ready |= ready;
        max_stale_run = max_stale_run.max(io.imu_stale().run);
        if last_progress.elapsed() >= Duration::from_secs(10) {
            println!(
                "{}",
                json!({"type":"progress", "seconds":start.elapsed().as_secs_f64(),
                "success":success, "errors":errors, "imu_ready":ready,
                "max_stale_run":max_stale_run, "reopens":reopen_count})
            );
            last_progress = Instant::now();
        }
        deadline += period;
        while deadline < Instant::now() {
            deadline += period;
            skipped_slots += 1;
        }
    }
    let elapsed = start.elapsed().as_secs_f64();
    let final_ready = bus.as_ref().unwrap().imu_ready();
    let stale = bus.as_ref().unwrap().imu_stale();
    timings.sort_by(f64::total_cmp);
    let percentile = |p: f64| timings[((timings.len() - 1) as f64 * p).round() as usize];
    let last = last_sensors.map(|s| {
        json!({"positions":s.positions, "velocities":s.velocities,
        "currents_ma":s.currents_ma, "gyro":s.imu.gyro, "gravity":s.imu.gravity, "quat":s.imu.quat})
    });
    println!(
        "{}",
        json!({"type":"summary", "imu_enabled":enabled, "read_only":true,
        "seconds":elapsed, "success":success, "errors":errors, "hz":success as f64 / elapsed,
        "skipped_slots":skipped_slots, "reads_over_20ms":late_reads,
        "mean_ms":timings.iter().sum::<f64>() / timings.len() as f64,
        "p50_ms":percentile(0.50), "p95_ms":percentile(0.95), "p99_ms":percentile(0.99),
        "max_ms":timings.last(), "imu_ready":final_ready, "ready_samples":ready_samples,
        "ready_lost":ready_lost, "max_stale_run":max_stale_run, "stale_last_session":stale.total,
        "reopens":reopen_count, "last":last})
    );
    if errors != 0 || ready_lost != 0 || final_ready != enabled || max_stale_run >= 25 {
        return Err("bus read or existing IMU readiness/staleness checks failed".into());
    }
    Ok(())
}
