//! Read-only check of the HOME telemetry transaction sizes at the real loop rate.
//! Stop all other UART consumers before running with the serial path as argument.
//! Add `--wide` to compare the original 83-byte burst with the normal smaller blocks.
use rustypot::servo::dynamixel::xl330::Xl330Controller;
use std::time::{Duration, Instant};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut args = std::env::args().skip(1);
    let path = args.next().ok_or("serial path required")?;
    let wide = match (args.next().as_deref(), args.next()) {
        (None, None) => false,
        (Some("--wide"), None) => true,
        _ => return Err("usage: check_home_bus SERIAL [--wide]".into()),
    };
    let blocks: &[(u8, u8)] = if wide {
        &[(64, 83)]
    } else {
        &[(124, 23), (64, 7), (98, 1)]
    };
    let serial = serialport::new(path, 1_000_000)
        .timeout(Duration::from_millis(30))
        .open()?;
    let mut bus = Xl330Controller::new()
        .with_protocol_v2()
        .with_serial_port(serial);
    let ids = duck_control::JOINT_IDS;
    let off = bus.sync_read_raw_data(&ids, 64, 1)?;
    if off.len() != ids.len() || off.iter().any(|b| b.as_slice() != [0]) {
        return Err("all motors must be OFF".into());
    }
    let began = Instant::now();
    let mut cycles = 0;
    let mut failures = 0;
    let mut recovered = 0;
    let mut longest = Duration::ZERO;
    while began.elapsed() < Duration::from_secs(10) {
        let tick = Instant::now();
        for &(a, n) in blocks {
            let first = bus.sync_read_raw_data(&ids, a, n);
            let value = if let Err(ref e) = first {
                eprintln!("discarded read at {a}/{n}: {e}");
                let retry = bus.sync_read_raw_data(&ids, a, n);
                if retry.is_ok() {
                    recovered += 1;
                }
                retry
            } else {
                first
            };
            match value {
                Ok(b) if b.len() == ids.len() && b.iter().all(|b| b.len() == n as usize) => (),
                other => {
                    failures += 1;
                    eprintln!("read at {a}/{n}: {other:?}");
                }
            }
        }
        longest = longest.max(tick.elapsed());
        cycles += 1;
        std::thread::sleep(Duration::from_millis(20).saturating_sub(tick.elapsed()));
    }
    println!(
        "cycles={cycles} recovered_reads={recovered} failures={failures} max_cycle_ms={:.2}",
        longest.as_secs_f64() * 1000.
    );
    if failures > 0 {
        return Err("telemetry was not clean".into());
    }
    Ok(())
}
