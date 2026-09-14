//! Bounded, hand-supported HOME diagnostic. No IMU or policy is involved.
//! The caller supplies an independent torque-off watchdog for process death.

use super::*;
use crate::model::DEFAULT_POSITION;
use serde::Serialize;
use std::collections::VecDeque;
use std::io::Write;

const PWM_CAP: u16 = 300;
const CURRENT_CAP_MA: i16 = 350;
fn output_limits(higher_effort: bool) -> (u16, i16, [u16; 3]) {
    if higher_effort {
        (600, 700, [100, 50, 1600])
    } else {
        (PWM_CAP, CURRENT_CAP_MA, [0, 0, 800])
    }
}
const WATCHDOG: u8 = 15;
const MAX_GAP: Duration = Duration::from_millis(150);
const SETTLE: Duration = Duration::from_secs(3);
const SAVE: &[(u8, u8)] = &[(80, 6), (88, 4), (98, 1), (100, 2), (108, 8)];

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SupportedPose {
    Home,
    Zero,
}
impl SupportedPose {
    pub fn name(self) -> &'static str {
        match self {
            Self::Home => "home",
            Self::Zero => "zero",
        }
    }
    pub fn positions(self) -> [f64; NUM_JOINTS] {
        match self {
            Self::Home => DEFAULT_POSITION,
            Self::Zero => std::array::from_fn(|j| {
                if JOINT_IDS[j] == 34 {
                    crate::model::MOUTH_CLOSED
                } else {
                    0.
                }
            }),
        }
    }
}
pub enum PoseCommand {
    Continue,
    Move(SupportedPose),
    Relax,
}
/// A supervised operator session. The external guardian requires both browser
/// heartbeats and progress from this actual control loop, independently.
pub trait PoseSession {
    fn poll(&mut self) -> Result<PoseCommand>;
    fn arm(&mut self) -> Result<()>;
    fn progress(&mut self) -> Result<()>;
    fn off(&mut self) -> Result<()>;
}

fn bad(message: impl Into<String>) -> IoError {
    IoError::Bus(message.into())
}
fn u16_at(b: &[u8], n: usize) -> u16 {
    u16::from_le_bytes([b[n], b[n + 1]])
}
fn i32_at(b: &[u8], n: usize) -> i32 {
    i32::from_le_bytes(b[n..n + 4].try_into().unwrap())
}

type ReadResult<T> = std::result::Result<T, Box<dyn std::error::Error>>;

fn retryable_read(error: &(dyn std::error::Error + 'static)) -> bool {
    use rustypot::CommunicationErrorKind as Kind;
    matches!(
        error.downcast_ref::<Kind>(),
        Some(
            Kind::ChecksumError | Kind::ParsingError | Kind::TimeoutError | Kind::IncorrectId(_, _)
        )
    ) || error
        .downcast_ref::<std::io::Error>()
        .is_some_and(|e| e.kind() == std::io::ErrorKind::TimedOut)
}

fn read_with_one_retry<T>(mut read: impl FnMut() -> ReadResult<T>) -> ReadResult<T> {
    match read() {
        Err(e) if retryable_read(e.as_ref()) => {
            // rustypot drains old RX bytes before issuing the new read. No goal or
            // setting write is repeated, and the caller still enforces MAX_GAP.
            tracing::warn!(error = %e, "HOME read frame discarded; retrying read once without advancing target");
            read()
        }
        result => result,
    }
}

#[derive(Clone, Serialize)]
struct Sample {
    elapsed_s: f64,
    positions: [f64; NUM_JOINTS],
    raw_ticks: [i32; NUM_JOINTS],
    currents_ma: [i16; NUM_JOINTS],
    pwm: [i16; NUM_JOINTS],
    velocities: [f64; NUM_JOINTS],
    volts: [f64; NUM_JOINTS],
    temperatures: [u8; NUM_JOINTS],
    torque: [u8; NUM_JOINTS],
    errors: [u8; NUM_JOINTS],
    watchdog: [u8; NUM_JOINTS],
}

#[derive(Default)]
struct HomeSettling {
    samples: VecDeque<(Duration, [f64; NUM_JOINTS])>,
}

impl HomeSettling {
    fn observe_target(
        &mut self,
        elapsed: Duration,
        sample: &Sample,
        destination: &[f64; NUM_JOINTS],
    ) -> bool {
        if !(0..NUM_JOINTS)
            .all(|j| (sample.positions[j] - destination[j]).abs() <= 2f64.to_radians())
        {
            self.samples.clear();
            return false;
        }
        self.samples.push_back((elapsed, sample.positions));
        // Keep the sample at the one-second boundary so discrete sampling covers
        // a full second instead of continually shrinking the window just below it.
        while self.samples.len() > 1
            && elapsed.saturating_sub(self.samples[1].0) >= Duration::from_secs(1)
        {
            self.samples.pop_front();
        }
        if elapsed.saturating_sub(self.samples[0].0) < Duration::from_secs(1) {
            return false;
        }
        (0..NUM_JOINTS).all(|j| {
            let low = self
                .samples
                .iter()
                .map(|(_, q)| q[j])
                .fold(f64::INFINITY, f64::min);
            let high = self
                .samples
                .iter()
                .map(|(_, q)| q[j])
                .fold(f64::NEG_INFINITY, f64::max);
            high - low <= 0.5f64.to_radians()
        })
    }
}

fn record(log: &mut impl Write, event: serde_json::Value) -> Result<()> {
    serde_json::to_writer(&mut *log, &event).map_err(|e| bad(e.to_string()))?;
    writeln!(log)
        .and_then(|_| log.flush())
        .map_err(|e| bad(e.to_string()))
}

fn pose_duration(from: &[f64; NUM_JOINTS], to: &[f64; NUM_JOINTS]) -> Duration {
    let degrees = (0..NUM_JOINTS)
        .map(|j| (to[j] - from[j]).abs().to_degrees())
        .fold(0., f64::max);
    Duration::from_secs_f64((degrees / 6. + 0.5).clamp(5., 30.))
}

#[cfg(test)]
fn validate_plan(from: &[f64; NUM_JOINTS], duration: Duration) -> Result<()> {
    validate_pose_plan(from, duration, &DEFAULT_POSITION)
}

fn validate_pose_plan(
    from: &[f64; NUM_JOINTS],
    duration: Duration,
    destination: &[f64; NUM_JOINTS],
) -> Result<()> {
    if !(5.0..=30.0).contains(&duration.as_secs_f64()) {
        return Err(bad("guarded HOME duration must be 5..30 seconds"));
    }
    for j in 0..NUM_JOINTS {
        let delta = (destination[j] - from[j]).abs();
        // An unpowered neck can hang past the policy's normal working interval.
        // Its supported recovery remains strictly shorter than a half-turn.
        let travel_limit = if JOINT_IDS[j] == 30 { 175f64 } else { 90f64 };
        if !from[j].is_finite()
            || delta > travel_limit.to_radians()
            || delta / duration.as_secs_f64() > 6f64.to_radians()
        {
            return Err(bad(format!(
                "{}: HOME exceeds {travel_limit} degrees or 6 degrees/s",
                JOINT_NAMES[j],
            )));
        }
    }
    Ok(())
}

#[cfg(test)]
fn check(
    sample: &Sample,
    from: &[f64; NUM_JOINTS],
    target: &[f64; NUM_JOINTS],
    initial_temp: &[u8; NUM_JOINTS],
    higher_effort: bool,
) -> Result<()> {
    check_pose(
        sample,
        from,
        target,
        &DEFAULT_POSITION,
        initial_temp,
        higher_effort,
    )
}

fn check_pose(
    sample: &Sample,
    from: &[f64; NUM_JOINTS],
    target: &[f64; NUM_JOINTS],
    destination: &[f64; NUM_JOINTS],
    initial_temp: &[u8; NUM_JOINTS],
    higher_effort: bool,
) -> Result<()> {
    let (pwm_cap, current_cap, _) = output_limits(higher_effort);
    let mut total_ma = 0;
    for j in 0..NUM_JOINTS {
        let q = sample.positions[j];
        total_ma += i32::from(sample.currents_ma[j]).abs();
        if sample.torque[j] != 1 || sample.errors[j] != 0 || sample.watchdog[j] != WATCHDOG {
            return Err(bad(format!(
                "{}: torque/error/watchdog changed",
                JOINT_NAMES[j]
            )));
        }
        if i32::from(sample.currents_ma[j]).abs() > i32::from(current_cap)
            || i32::from(sample.pwm[j]).abs() > i32::from(pwm_cap) + 3
            || !(4.5..=5.5).contains(&sample.volts[j])
            || sample.temperatures[j] >= 40
            || sample.temperatures[j].saturating_sub(initial_temp[j]) >= 3
        {
            return Err(bad(format!(
                "{}: current/PWM/voltage/temperature limit",
                JOINT_NAMES[j]
            )));
        }
        if !q.is_finite()
            || q < from[j].min(destination[j]) - 3f64.to_radians()
            || q > from[j].max(destination[j]) + 3f64.to_radians()
        {
            return Err(bad(format!(
                "{}: position {:.2} deg left HOME corridor {:.2}..{:.2} deg",
                JOINT_NAMES[j],
                q.to_degrees(),
                from[j].min(destination[j]).to_degrees(),
                from[j].max(destination[j]).to_degrees()
            )));
        }
        if (q - target[j]).abs() > 8f64.to_radians() {
            return Err(bad(format!(
                "{}: tracking error {:.2} deg exceeds 8 deg",
                JOINT_NAMES[j],
                (q - target[j]).to_degrees()
            )));
        }
        // A position servo catches up faster than its slowly changing command. The
        // 15 deg/s trial stopped during ordinary catch-up at low PWM/current. Keep
        // the command at <=6 deg/s and bound actual runaway separately at 60 deg/s.
        if sample.velocities[j].abs() > 60f64.to_radians() {
            return Err(bad(format!(
                "{}: speed {:.2} deg/s exceeds 60 deg/s",
                JOINT_NAMES[j],
                sample.velocities[j].to_degrees()
            )));
        }
    }
    if total_ma > 2000 {
        return Err(bad("total input current exceeded 2 A"));
    }
    Ok(())
}

impl DynamixelIo {
    fn home_blocks(&mut self, address: u8, length: u8) -> Result<Vec<Vec<u8>>> {
        let blocks = read_with_one_retry(|| {
            self.controller
                .sync_read_raw_data(&JOINT_IDS, address, length)
        })
        .map_err(|e| bad(format!("HOME telemetry at {address}, length {length}: {e}")))?;
        if blocks.len() != NUM_JOINTS || blocks.iter().any(|b| b.len() != length as usize) {
            return Err(bad("incomplete HOME telemetry"));
        }
        Ok(blocks)
    }

    fn home_sample(&mut self, start: Instant) -> Result<Sample> {
        // Read only live telemetry. A 64..146 burst also copies 50 unused bytes per
        // servo. That large burst suffered frame errors on hardware; narrower reads
        // reduce UART load while retaining every checked field.
        let blocks = self.home_blocks(124, 23)?;
        let state = self.home_blocks(64, 7)?;
        let watchdog = self.home_blocks(98, 1)?;
        let mut s = Sample {
            elapsed_s: start.elapsed().as_secs_f64(),
            positions: [0.; NUM_JOINTS],
            raw_ticks: [0; NUM_JOINTS],
            currents_ma: [0; NUM_JOINTS],
            pwm: [0; NUM_JOINTS],
            velocities: [0.; NUM_JOINTS],
            volts: [0.; NUM_JOINTS],
            temperatures: [0; NUM_JOINTS],
            torque: [0; NUM_JOINTS],
            errors: [0; NUM_JOINTS],
            watchdog: [0; NUM_JOINTS],
        };
        for (j, b) in blocks.iter().enumerate() {
            s.raw_ticks[j] = i32_at(b, 8);
            s.positions[j] = self.calibration.model_position(
                j,
                s.raw_ticks[j] as f64 * crate::calibration::RADIANS_PER_TICK - PI,
            )?;
            s.torque[j] = state[j][0];
            s.errors[j] = state[j][6];
            s.watchdog[j] = watchdog[j][0];
            s.pwm[j] = u16_at(b, 0) as i16;
            s.currents_ma[j] = u16_at(b, 2) as i16;
            s.velocities[j] = i32_at(b, 4) as f64 * RAD_PER_SEC_PER_COUNT;
            s.volts[j] = u16_at(b, 20) as f64 / 10.;
            s.temperatures[j] = b[22];
        }
        Ok(s)
    }

    fn home_setting(&mut self, address: u8, data: Vec<u8>) -> Result<()> {
        self.controller
            .sync_write_raw_data(&JOINT_IDS, address, &vec![data.clone(); NUM_JOINTS])
            .map_err(|e| bad(format!("HOME setting {address}: {e}")))?;
        if self
            .home_blocks(address, data.len() as u8)?
            .iter()
            .any(|b| b != &data)
        {
            return Err(bad(format!("HOME setting {address} readback mismatch")));
        }
        Ok(())
    }

    fn home_off(&mut self) -> Result<()> {
        // Attempt every motor even after one fails, then independently verify all 15.
        for _ in 0..3 {
            let _ = self.set_torque(false);
            if let Ok(blocks) = self.home_blocks(64, 1) {
                if blocks.iter().all(|b| b[0] == 0) {
                    return Ok(());
                }
            }
        }
        Err(bad("cannot confirm all motors OFF; disconnect servo power"))
    }

    /// Explicit hand-supported diagnostic through the same calibration and goal writer
    /// as the daemon. Always relaxes and restores RAM settings, including after failure.
    /// The independent caller watchdog must cover SIGKILL, process death and blocked I/O.
    pub fn guarded_home(
        &mut self,
        duration: Duration,
        log: &mut impl Write,
        higher_effort: bool,
    ) -> Result<()> {
        self.guarded_pose(SupportedPose::Home, duration, log, higher_effort, None)
    }

    pub fn guarded_pose(
        &mut self,
        initial_pose: SupportedPose,
        mut duration: Duration,
        log: &mut impl Write,
        higher_effort: bool,
        mut session: Option<&mut dyn PoseSession>,
    ) -> Result<()> {
        let started = Instant::now();
        let mut pose = initial_pose;
        let mut destination = pose.positions();
        let interactive = session.is_some();
        let (pwm_cap, current_cap, dip) = output_limits(higher_effort);
        if self.calibration.configured_names().len() != NUM_JOINTS {
            return Err(bad("guarded HOME requires all 15 calibrated joints"));
        }
        // Read-only validation: no factory adoption or EEPROM correction in a probe.
        let before = self.home_blocks(0, 147)?;
        for (j, b) in before.iter().enumerate() {
            if u16_at(b, 0) != 1200
                || b[7] != JOINT_IDS[j]
                || b[8] != 3
                || b[10] != 0
                || b[11] != self.calibration.operating_mode()
                || i32_at(b, 20) != 0
                || b[64] != 0
                || b[70] != 0
                || b[98] != 0
                || b[63] & 0x34 != 0x34
                || u16_at(b, 36) < pwm_cap
                || b[146] > 35
            {
                return Err(bad(format!(
                    "{}: unsupported HOME hardware configuration",
                    JOINT_NAMES[j]
                )));
            }
            if b[11] == 3 && (i32_at(b, 48) != 4095 || i32_at(b, 52) != 0) {
                return Err(bad("single-turn hardware limits must be 0..4095"));
            }
        }
        let initial = self.home_sample(started)?;
        if initial.volts.iter().any(|v| !(4.5..=5.5).contains(v)) {
            return Err(bad("guarded HOME requires a 4.5..5.5 V motor rail"));
        }
        if interactive {
            duration = pose_duration(&initial.positions, &destination);
        }
        validate_pose_plan(&initial.positions, duration, &destination)?;
        self.calibration.servo_targets(&destination)?;
        record(
            log,
            serde_json::json!({"event":"home_preflight", "ids":JOINT_IDS,
            "initial":initial, "target":destination, "pose":pose.name(), "duration_s":duration.as_secs_f64(),
            "pwm_cap":pwm_cap, "current_cap_ma":current_cap, "gain":dip[2], "position_dip":dip, "higher_effort":higher_effort}),
        )?;
        let mut fault_sample = None;
        let mut reached = false;
        let run = (|| -> Result<()> {
            self.home_setting(100, 0u16.to_le_bytes().to_vec())?;
            self.home_setting(80, dip.into_iter().flat_map(u16::to_le_bytes).collect())?;
            self.home_setting(88, vec![0; 4])?;
            self.home_setting(
                108,
                [0u32, 0].into_iter().flat_map(u32::to_le_bytes).collect(),
            )?;
            let hold = self.present_positions()?;
            self.write(&JointTargets::new(hold))?;
            self.home_setting(98, vec![WATCHDOG])?;
            if let Some(s) = session.as_deref_mut() {
                if matches!(s.poll()?, PoseCommand::Relax) {
                    return Ok(());
                }
                s.arm()?;
            }
            self.set_torque(true)?;
            let mut from = self.home_sample(started)?.positions;
            validate_pose_plan(&from, duration, &destination)?;
            self.write(&JointTargets::new(from))?;
            self.home_setting(100, pwm_cap.to_le_bytes().to_vec())?;
            let mut ramp = Instant::now();
            let mut last = Instant::now();
            let mut settling = HomeSettling::default();
            let mut last_target = from;
            let mut holding = false;
            let mut last_logged = Instant::now() - Duration::from_secs(1);
            record(
                log,
                serde_json::json!({"event":"pose_moving","pose":pose.name(),"duration_s":duration.as_secs_f64(),"target":destination}),
            )?;
            loop {
                let tick = Instant::now();
                let s = self.home_sample(started)?;
                if last.elapsed() > MAX_GAP {
                    return Err(bad("HOME telemetry gap exceeded 150 ms"));
                }
                last = Instant::now();
                if let Err(e) = check_pose(
                    &s,
                    &from,
                    &last_target,
                    &destination,
                    &initial.temperatures,
                    higher_effort,
                ) {
                    fault_sample = Some((s, last_target));
                    return Err(e);
                }
                if let Some(control) = session.as_deref_mut() {
                    control.progress()?;
                    match control.poll()? {
                        PoseCommand::Relax => break,
                        PoseCommand::Move(next) => {
                            if !holding {
                                return Err(bad("wait for the current pose to settle"));
                            }
                            pose = next;
                            destination = pose.positions();
                            from = s.positions;
                            duration = pose_duration(&from, &destination);
                            validate_pose_plan(&from, duration, &destination)?;
                            self.calibration.servo_targets(&destination)?;
                            ramp = Instant::now();
                            last_target = from;
                            settling = HomeSettling::default();
                            holding = false;
                            reached = false;
                            record(
                                log,
                                serde_json::json!({"event":"pose_moving","pose":pose.name(),"duration_s":duration.as_secs_f64(),"target":destination}),
                            )?;
                        }
                        PoseCommand::Continue => (),
                    }
                }
                if !holding || last_logged.elapsed() >= Duration::from_millis(100) {
                    record(
                        log,
                        serde_json::json!({"event":"home_sample", "sample":s, "target":last_target,"pose":pose.name(),"phase":if holding {"holding"} else {"moving"},"progress":(ramp.elapsed().as_secs_f64()/duration.as_secs_f64()).min(1.)}),
                    )?;
                    last_logged = Instant::now();
                }
                let age = ramp.elapsed();
                if !holding && age > duration + SETTLE {
                    return Err(bad("HOME did not settle within 2 degrees"));
                }
                if !holding && age >= duration && settling.observe_target(age, &s, &destination) {
                    // The speed register can report 2.75 deg/s while the encoder
                    // moves only one tick over a second. Position history establishes
                    // settling; the independent 60 deg/s motion check still applies.
                    record(
                        log,
                        serde_json::json!({"event":if pose==SupportedPose::Home {"home_reached"} else {"zero_reached"}, "sample":s,"pose":pose.name(),
                        "tolerance_degrees":2,"maximum_position_span_degrees":0.5,"hold_seconds":1}),
                    )?;
                    reached = true;
                    if !interactive {
                        break;
                    }
                    holding = true;
                }
                let t = (age.as_secs_f64() / duration.as_secs_f64()).min(1.);
                last_target = std::array::from_fn(|j| from[j] + (destination[j] - from[j]) * t);
                self.write(&JointTargets::new(last_target))?;
                std::thread::sleep(Duration::from_millis(20).saturating_sub(tick.elapsed()));
            }
            Ok(())
        })();
        // No log write or register restoration can delay confirmed torque-off.
        self.home_off()?;
        if let Some(s) = session.as_deref_mut() {
            s.off()?;
        }
        self.home_setting(98, vec![0])?;
        for &(a, n) in SAVE {
            let values: Vec<Vec<u8>> = before
                .iter()
                .map(|b| b[a as usize..(a + n) as usize].to_vec())
                .collect();
            self.controller
                .sync_write_raw_data(&JOINT_IDS, a, &values)
                .map_err(|e| bad(e.to_string()))?;
            if self.home_blocks(a, n)? != values {
                return Err(bad(format!("HOME restoration failed at {a}")));
            }
        }
        let final_blocks = self.home_blocks(0, 147)?;
        if (0..NUM_JOINTS)
            .any(|j| final_blocks[j][..64] != before[j][..64] || final_blocks[j][64] != 0)
        {
            return Err(bad("HOME EEPROM or torque restoration check failed"));
        }
        if let Some((s, target)) = fault_sample {
            record(
                log,
                serde_json::json!({"event":"home_fault_sample", "sample":s,"target":target}),
            )?;
        }
        record(
            log,
            serde_json::json!({"event":"home_cleanup", "all_off":true, "settings_restored":true,
            "reached":run.is_ok() && reached, "error":run.as_ref().err().map(ToString::to_string)}),
        )?;
        run
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn settling_accepts_one_tick_noise_but_not_real_oscillation() {
        let mut stable = HomeSettling::default();
        let mut moving = HomeSettling::default();
        for n in 0..=50 {
            let mut s = sample();
            s.positions[6] += (n % 2) as f64 * crate::calibration::RADIANS_PER_TICK;
            s.velocities[6] = 2.75f64.to_radians();
            assert_eq!(
                stable.observe_target(Duration::from_millis(n * 20), &s, &DEFAULT_POSITION),
                n == 50
            );
            s.positions[6] = DEFAULT_POSITION[6] + ((n % 2) as f64).to_radians();
            assert!(!moving.observe_target(Duration::from_millis(n * 20), &s, &DEFAULT_POSITION));
        }
        let mut out = sample();
        out.positions[6] += 3f64.to_radians();
        assert!(!stable.observe_target(Duration::from_millis(1020), &out, &DEFAULT_POSITION));
        assert!(!stable.observe_target(Duration::from_millis(1040), &sample(), &DEFAULT_POSITION));
    }
    #[test]
    fn read_recovery_is_one_bounded_retry_only_for_transport_faults() {
        let mut calls = 0;
        let value = read_with_one_retry(|| -> ReadResult<u8> {
            calls += 1;
            if calls == 1 {
                Err(Box::new(rustypot::CommunicationErrorKind::ParsingError))
            } else {
                Ok(42)
            }
        })
        .unwrap();
        assert_eq!((value, calls), (42, 2));
        calls = 0;
        let failed = read_with_one_retry(|| -> ReadResult<()> {
            calls += 1;
            Err(Box::new(rustypot::CommunicationErrorKind::ChecksumError))
        });
        assert!(failed.is_err());
        assert_eq!(calls, 2);
        calls = 0;
        let failed = read_with_one_retry(|| -> ReadResult<()> {
            calls += 1;
            Err(Box::new(rustypot::CommunicationErrorKind::Unsupported))
        });
        assert!(failed.is_err());
        assert_eq!(calls, 1);
    }
    fn sample() -> Sample {
        Sample {
            elapsed_s: 0.,
            positions: DEFAULT_POSITION,
            raw_ticks: [2048; NUM_JOINTS],
            currents_ma: [20; NUM_JOINTS],
            pwm: [30; NUM_JOINTS],
            velocities: [0.; NUM_JOINTS],
            volts: [5.1; NUM_JOINTS],
            temperatures: [29; NUM_JOINTS],
            torque: [1; NUM_JOINTS],
            errors: [0; NUM_JOINTS],
            watchdog: [WATCHDOG; NUM_JOINTS],
        }
    }
    #[test]
    fn refuses_fast_large_or_nonfinite_home() {
        for value in [f64::NAN, 2., -2.] {
            let mut from = DEFAULT_POSITION;
            from[0] = value;
            assert!(validate_plan(&from, Duration::from_secs(10)).is_err());
        }
        let mut from = DEFAULT_POSITION;
        from[0] = 30f64.to_radians();
        assert!(validate_plan(&from, Duration::from_secs(5)).is_ok());
        assert!(validate_plan(&from, Duration::from_secs(1)).is_err());
        from = DEFAULT_POSITION;
        from[5] = (-140f64).to_radians();
        assert!(validate_plan(&from, Duration::from_secs(30)).is_ok());
        assert!(validate_plan(&from, Duration::from_secs(10)).is_err());
        from[5] = DEFAULT_POSITION[5] - 176f64.to_radians();
        assert!(validate_plan(&from, Duration::from_secs(30)).is_err());
    }
    #[test]
    fn no_home_corridor_or_load_check_is_exempt() {
        let initial = sample();
        assert!(
            check(
                &initial,
                &DEFAULT_POSITION,
                &DEFAULT_POSITION,
                &initial.temperatures,
                false
            )
            .is_ok()
        );
        for fault in 0..8 {
            let mut s = initial.clone();
            match fault {
                0 => s.positions[0] += 4f64.to_radians(),
                1 => s.currents_ma[1] = 351,
                2 => s.pwm[2] = 304,
                3 => s.volts[3] = 4.4,
                4 => s.temperatures[4] = 32,
                5 => s.watchdog[5] = 255,
                6 => s.errors[6] = 32,
                _ => s.velocities[7] = 61f64.to_radians(),
            };
            assert!(
                check(
                    &s,
                    &DEFAULT_POSITION,
                    &DEFAULT_POSITION,
                    &initial.temperatures,
                    false
                )
                .is_err(),
                "fault {fault}"
            );
        }
    }
    #[test]
    fn higher_effort_still_bounds_load_and_preserves_motion_checks() {
        let mut s = sample();
        s.currents_ma[0] = 500;
        s.pwm[0] = 500;
        assert!(
            check(
                &s,
                &DEFAULT_POSITION,
                &DEFAULT_POSITION,
                &[29; NUM_JOINTS],
                true
            )
            .is_ok()
        );
        assert!(
            check(
                &s,
                &DEFAULT_POSITION,
                &DEFAULT_POSITION,
                &[29; NUM_JOINTS],
                false
            )
            .is_err()
        );
        s.currents_ma[0] = 701;
        assert!(
            check(
                &s,
                &DEFAULT_POSITION,
                &DEFAULT_POSITION,
                &[29; NUM_JOINTS],
                true
            )
            .is_err()
        );
        s.currents_ma[0] = 500;
        s.positions[0] += 4f64.to_radians();
        assert!(
            check(
                &s,
                &DEFAULT_POSITION,
                &DEFAULT_POSITION,
                &[29; NUM_JOINTS],
                true
            )
            .is_err()
        );
    }
}
