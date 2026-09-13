//! Per-robot encoder zeroes, applied only at the hardware I/O boundary.
//!
//! Policies, home poses, IPC and kinematics keep their model-space radians. A constant
//! zero shift changes measured and commanded positions in opposite directions; it does
//! not change velocity, gains, action scales or the policy's default pose.

use std::f64::consts::{PI, TAU};
use std::path::Path;

use serde::Deserialize;

use crate::io::{IoError, Result};
use crate::model::{JOINT_IDS, JOINT_NAMES, NUM_JOINTS, joint_index};

pub const RADIANS_PER_TICK: f64 = 2.0 * PI / 4096.0;
const POSITION_TOLERANCE: f64 = RADIANS_PER_TICK;
const EXTENDED_MAX_TICK: i32 = 1_048_575;

#[derive(Debug, Clone, Copy, Default, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
enum PositionMode {
    #[default]
    SingleTurn,
    ExtendedPosition,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct File {
    #[serde(default)]
    position_mode: PositionMode,
    joints: Vec<JointZero>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct JointZero {
    name: String,
    id: u8,
    /// Effective encoder count corresponding to zero model radians. Fractional counts
    /// allow calibration at a nonzero reference angle without throwing precision away.
    zero_tick: f64,
    #[serde(default)]
    limits_rad: Option<[f64; 2]>,
}

#[derive(Debug, thiserror::Error)]
pub enum CalibrationError {
    #[error("cannot read joint calibration {path}: {source}")]
    Read {
        path: String,
        #[source]
        source: std::io::Error,
    },
    #[error("invalid joint calibration JSON: {0}")]
    Json(#[from] serde_json::Error),
    #[error("invalid joint calibration: {0}")]
    Invalid(String),
}

#[derive(Debug, Clone, Copy, Default)]
pub struct JointCalibration {
    offsets: [f64; NUM_JOINTS],
    configured: [bool; NUM_JOINTS],
    mode: PositionMode,
    limits: [Option<[f64; 2]>; NUM_JOINTS],
    // Per-open hardware coordinates. Rebuilt from the first valid reading after boot/reboot.
    session_offsets: [Option<f64>; NUM_JOINTS],
}

impl JointCalibration {
    pub fn load(path: &Path) -> std::result::Result<Self, CalibrationError> {
        let text = std::fs::read_to_string(path).map_err(|source| CalibrationError::Read {
            path: path.display().to_string(),
            source,
        })?;
        Self::from_json(&text)
    }

    pub fn from_json(text: &str) -> std::result::Result<Self, CalibrationError> {
        let file: File = serde_json::from_str(text)?;
        let mut calibration = Self {
            mode: file.position_mode,
            ..Self::default()
        };
        for joint in file.joints {
            let index = joint_index(&joint.name).ok_or_else(|| {
                CalibrationError::Invalid(format!("unknown joint {}", joint.name))
            })?;
            if calibration.configured[index] {
                return Err(CalibrationError::Invalid(format!(
                    "duplicate joint {}",
                    joint.name
                )));
            }
            if joint.id != JOINT_IDS[index] {
                return Err(CalibrationError::Invalid(format!(
                    "{} requires ID {}, got {}",
                    joint.name, JOINT_IDS[index], joint.id
                )));
            }
            let valid_zero = match file.position_mode {
                PositionMode::SingleTurn => (0.0..=4095.0).contains(&joint.zero_tick),
                PositionMode::ExtendedPosition => (0.0..4096.0).contains(&joint.zero_tick),
            };
            if !joint.zero_tick.is_finite() || !valid_zero {
                return Err(CalibrationError::Invalid(format!(
                    "{} zero_tick must be finite and in 0..=4095",
                    joint.name
                )));
            }
            match joint.limits_rad {
                Some([min, max])
                    if min.is_finite()
                        && max.is_finite()
                        && min >= -PI
                        && max <= PI
                        && min < max
                        && max - min < TAU - 2.0 * POSITION_TOLERANCE => {}
                None if file.position_mode == PositionMode::SingleTurn => {}
                _ => {
                    return Err(CalibrationError::Invalid(format!(
                        "{} requires finite limits_rad inside -pi..pi spanning less than one revolution",
                        joint.name
                    )));
                }
            }
            calibration.limits[index] = joint.limits_rad;
            calibration.offsets[index] = (joint.zero_tick - 2048.0) * RADIANS_PER_TICK;
            calibration.configured[index] = true;
        }
        Ok(calibration)
    }

    pub fn configured_names(&self) -> Vec<&'static str> {
        JOINT_NAMES
            .iter()
            .enumerate()
            .filter_map(|(i, name)| self.configured[i].then_some(*name))
            .collect()
    }

    pub fn is_configured(&self, joint: usize) -> bool {
        self.configured[joint]
    }

    pub fn operating_mode(&self) -> u8 {
        match self.mode {
            PositionMode::SingleTurn => 3,
            PositionMode::ExtendedPosition => 4,
        }
    }

    pub fn has_extended_joints(&self) -> bool {
        self.mode == PositionMode::ExtendedPosition && self.configured.iter().any(|&v| v)
    }

    pub fn invalidate_origin(&mut self, id: u8) {
        if let Some(index) = JOINT_IDS.iter().position(|&candidate| candidate == id) {
            self.session_offsets[index] = None;
        }
    }

    fn inside_limits(&self, joint: usize, value: f64) -> bool {
        let [min, max] = self.limits[joint].unwrap_or([-PI, PI]);
        value.is_finite() && value >= min - POSITION_TOLERANCE && value <= max + POSITION_TOLERANCE
    }

    /// A replacement has a different installation zero even if its model and ID match.
    /// Refuse automatic adoption before any write; the new motor must be calibrated first.
    pub fn check_replacement(&self, id: u8) -> Result<()> {
        if let Some(joint) = JOINT_IDS.iter().position(|&candidate| candidate == id)
            && self.is_configured(joint)
        {
            return Err(IoError::Bus(format!(
                "ID {id} has a saved joint zero; configure and recalibrate its replacement before restarting robotd"
            )));
        }
        Ok(())
    }

    /// `servo_radians` follows rustypot: encoder 2048 means 0 radians.
    pub fn model_position(&mut self, joint: usize, servo_radians: f64) -> Result<f64> {
        if !self.configured[joint] {
            return Ok(servo_radians);
        }
        if self.mode == PositionMode::ExtendedPosition && self.session_offsets[joint].is_none() {
            let [min, max] = self.limits[joint].expect("extended limits validated at load");
            let delta = servo_radians - self.offsets[joint];
            // The mechanical interval is narrower than one revolution. At power-on the
            // servo reports a single-turn phase, so at most one integer turn fits it.
            let turns = ((delta - (min + max) * 0.5) / TAU).round();
            let offset = self.offsets[joint] + turns * TAU;
            let position = servo_radians - offset;
            if !self.inside_limits(joint, position) {
                return Err(IoError::Bus(format!(
                    "{} position cannot be located inside calibrated model limits; check pose/zero",
                    JOINT_NAMES[joint]
                )));
            }
            self.session_offsets[joint] = Some(offset);
        }
        let offset = if self.mode == PositionMode::ExtendedPosition {
            self.session_offsets[joint].expect("origin established above")
        } else {
            self.offsets[joint]
        };
        let model = servo_radians - offset;
        // Never silently rebase an active origin after a discontinuity. A reset that changes
        // the encoder's turn must be handled while stopped, via reboot or a new bus open.
        if !self.inside_limits(joint, model) {
            return Err(IoError::Bus(format!(
                "calibrated reading for {} is outside model travel; stop and check encoder reset/pose",
                JOINT_NAMES[joint]
            )));
        }
        Ok(model)
    }

    pub fn servo_targets(&self, model: &[f64; NUM_JOINTS]) -> Result<[f64; NUM_JOINTS]> {
        let mut servo = [0.0; NUM_JOINTS];
        for (joint, target) in model.iter().enumerate() {
            let offset = if self.configured[joint] && self.mode == PositionMode::ExtendedPosition {
                if !self.inside_limits(joint, *target) {
                    return Err(IoError::Bus(format!(
                        "target for {} is outside model limits",
                        JOINT_NAMES[joint]
                    )));
                }
                self.session_offsets[joint].ok_or_else(|| {
                    IoError::Bus(format!(
                        "{} needs a valid position read before any target/torque enable",
                        JOINT_NAMES[joint]
                    ))
                })?
            } else {
                self.offsets[joint]
            };
            servo[joint] = target + offset;
            if !servo[joint].is_finite() {
                return Err(IoError::Bus(format!(
                    "non-finite target for {}",
                    JOINT_NAMES[joint]
                )));
            }
            // rustypot truncates the inverse conversion. Floating cancellation can make
            // an exact calibrated count become the adjacent integer after truncation,
            // on either side of zero. Correct only roundoff next to an integer count;
            // preserve the existing truncation for genuinely fractional targets.
            if self.configured[joint] {
                let raw = 4096.0 * (PI + servo[joint]) / (2.0 * PI);
                let nearest = raw.round();
                if (nearest - raw).abs() < 1e-9 {
                    servo[joint] += nearest.signum() * RADIANS_PER_TICK * 1e-10;
                }
            }
            // Never wrap a shifted target: Mode 4 uses its continuous signed origin,
            // while legacy Mode 3 must still reject the single-turn seam. Check the
            // actual integer conversion rustypot sends, including endpoint roundoff.
            let tick = (4096.0 * (PI + servo[joint]) / (2.0 * PI)) as i32;
            let range = match self.mode {
                PositionMode::SingleTurn => 0..=4095,
                PositionMode::ExtendedPosition => -EXTENDED_MAX_TICK..=EXTENDED_MAX_TICK,
            };
            if self.configured[joint] && !range.contains(&tick) {
                return Err(IoError::Bus(format!(
                    "calibrated target for {} is outside configured encoder travel ({tick} ticks)",
                    JOINT_NAMES[joint]
                )));
            }
        }
        Ok(servo)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn calibration() -> JointCalibration {
        JointCalibration::from_json(
            r#"{"joints":[{"name":"left_hip_yaw","id":20,"zero_tick":2176.0}]}"#,
        )
        .unwrap()
    }

    #[test]
    fn hand_placed_reference_is_zero_and_inverse_command_returns_to_it() {
        let mut c = calibration();
        let servo = (2176.0 - 2048.0) * RADIANS_PER_TICK;
        assert!(c.model_position(0, servo).unwrap().abs() < 1e-12);
        assert!((c.servo_targets(&[0.0; NUM_JOINTS]).unwrap()[0] - servo).abs() < 1e-12);
        for model in [-0.7, 0.0, 0.5] {
            let mut targets = [0.0; NUM_JOINTS];
            targets[0] = model;
            assert!(
                (c.model_position(0, c.servo_targets(&targets).unwrap()[0])
                    .unwrap()
                    - model)
                    .abs()
                    < 1e-12
            );
        }
    }

    #[test]
    fn partial_files_leave_other_joints_unchanged() {
        let mut c = calibration();
        assert_eq!(c.configured_names(), ["left_hip_yaw"]);
        assert_eq!(c.model_position(1, 0.3).unwrap(), 0.3);
        assert_eq!(c.servo_targets(&[0.3; NUM_JOINTS]).unwrap()[1], 0.3);
    }

    #[test]
    fn nonzero_reference_angle_produces_the_same_model_coordinates() {
        // Measured 2304 at model angle pi/8 => zero at 2048, not 2304.
        let zero = 2304.0 - (PI / 8.0) / RADIANS_PER_TICK;
        let mut c = JointCalibration::from_json(&format!(
            r#"{{"joints":[{{"name":"head_yaw","id":32,"zero_tick":{zero}}}]}}"#
        ))
        .unwrap();
        assert!((c.model_position(7, PI / 8.0).unwrap() - PI / 8.0).abs() < 1e-12);
    }

    #[test]
    fn malformed_mismatched_duplicate_and_unknown_entries_fail() {
        for text in [
            r#"{"joints":[{"name":"head_yaw","id":0,"zero_tick":2048}]}"#,
            r#"{"joints":[{"name":"wrong","id":32,"zero_tick":2048}]}"#,
            r#"{"joints":[{"name":"head_yaw","id":32,"zero_tick":4096}]}"#,
            r#"{"joints":[{"name":"head_yaw","id":32,"zero_tick":-1}]}"#,
            r#"{"joints":[{"name":"head_yaw","id":32,"zero_tick":2048,"offset":0}]}"#,
            r#"{"joints":[{"name":"head_yaw","id":32,"zero_tick":2048},{"name":"head_yaw","id":32,"zero_tick":2049}]}"#,
            r#"{"joints":[],"joints":[]}"#,
            r#"{"joints":[{"name":"head_yaw","id":32,"zero_tick":NaN}]}"#,
        ] {
            assert!(
                JointCalibration::from_json(text).is_err(),
                "accepted {text}"
            );
        }
    }

    #[test]
    fn unsafe_shifted_target_is_rejected_instead_of_wrapped() {
        let c = calibration();
        let mut targets = [0.0; NUM_JOINTS];
        targets[0] = PI;
        assert!(c.servo_targets(&targets).is_err());
        targets[0] = f64::NAN;
        assert!(c.servo_targets(&targets).is_err());
    }

    #[test]
    fn exact_encoder_targets_survive_the_transport_integer_conversion() {
        for raw in [0, 532, 975, 1022, 1533, 2054, 3069, 4071, 4095] {
            let c = JointCalibration::from_json(&format!(
                r#"{{"joints":[{{"name":"left_ankle","id":24,"zero_tick":{raw}}}]}}"#
            ))
            .unwrap();
            let servo = c.servo_targets(&[0.0; NUM_JOINTS]).unwrap()[4];
            assert_eq!((4096.0 * (PI + servo) / (2.0 * PI)) as i32, raw);
        }
        let zero = 2054.0 - crate::model::MOUTH_CLOSED / RADIANS_PER_TICK;
        let c = JointCalibration::from_json(&format!(
            r#"{{"joints":[{{"name":"mouth","id":34,"zero_tick":{zero}}}]}}"#
        ))
        .unwrap();
        let mut pose = [0.0; NUM_JOINTS];
        pose[9] = crate::model::MOUTH_CLOSED;
        let servo = c.servo_targets(&pose).unwrap()[9];
        assert_eq!((4096.0 * (PI + servo) / (2.0 * PI)) as i32, 2054);
    }

    #[test]
    fn replacement_cannot_reuse_the_removed_motors_zero() {
        let c = calibration();
        assert!(c.check_replacement(20).is_err());
        assert!(c.check_replacement(21).is_ok());
        assert!(JointCalibration::default().check_replacement(20).is_ok());
    }

    #[test]
    fn explicit_missing_file_does_not_use_factory_zeroes() {
        assert!(JointCalibration::load(Path::new("/definitely-missing-calibration.json")).is_err());
    }

    #[test]
    fn holding_a_measured_endpoint_stays_inside_the_encoder_range() {
        for zero in [0.0, 100.5, 2176.0, 4095.0] {
            let mut c = JointCalibration::from_json(&format!(
                r#"{{"joints":[{{"name":"head_yaw","id":32,"zero_tick":{zero}}}]}}"#
            ))
            .unwrap();
            for tick in [0, 2048, 4095] {
                let mut model = [0.0; NUM_JOINTS];
                let servo = tick as f64 * RADIANS_PER_TICK - PI;
                let expected_model = (tick as f64 - zero) * RADIANS_PER_TICK;
                if !(-PI..=PI).contains(&expected_model) {
                    assert!(c.model_position(7, servo).is_err());
                    continue;
                }
                model[7] = c.model_position(7, servo).unwrap();
                assert!((c.servo_targets(&model).unwrap()[7] - servo).abs() < 1e-12);
            }
        }
    }
    #[test]
    fn closed_mouth_reference_and_swapped_head_zeroes_round_trip() {
        // UI uses closed=0; runtime retains its existing closed=-5 degree API.
        // Export an effective runtime zero instead of driving 5 degrees past closed.
        let mouth_zero = 2054.0 - crate::model::MOUTH_CLOSED / RADIANS_PER_TICK;
        let mut c = JointCalibration::from_json(&format!(
            r#"{{"joints":[{{"name":"head_yaw","id":32,"zero_tick":3069}},{{"name":"head_roll","id":33,"zero_tick":2051}},{{"name":"mouth","id":34,"zero_tick":{mouth_zero}}}]}}"#
        )).unwrap();
        let mut pose = [0.0; NUM_JOINTS];
        pose[crate::model::MOUTH_INDEX] = crate::model::mouth_target(0.0);
        let servo = c.servo_targets(&pose).unwrap();
        for (index, tick) in [(7, 3069.0), (8, 2051.0), (9, 2054.0)] {
            assert!((servo[index] - (tick - 2048.0) * RADIANS_PER_TICK).abs() < 1e-12);
            assert!((c.model_position(index, servo[index]).unwrap() - pose[index]).abs() < 1e-12);
        }
        pose[7] = 170.0f64.to_radians();
        assert!(c.servo_targets(&pose).is_err());
    }
    fn extended(zero: f64, limits: [f64; 2]) -> JointCalibration {
        JointCalibration::from_json(&format!(
            r#"{{"position_mode":"extended_position","joints":[{{"name":"left_hip_yaw","id":20,"zero_tick":{zero},"limits_rad":[{},{}]}}]}}"#,limits[0],limits[1]
        )).unwrap()
    }
    fn servo_angle(raw: i32) -> f64 {
        (raw as f64 - 2048.0) * RADIANS_PER_TICK
    }
    fn raw_target(servo: f64) -> i32 {
        (4096.0 * (PI + servo) / TAU) as i32
    }

    #[test]
    fn extended_crosses_the_encoder_seam_without_wrapping_commands() {
        let mut c = extended(4071.0, [-0.4363323129985824, 0.5235987755982988]);
        assert!(c.servo_targets(&[0.0; NUM_JOINTS]).is_err());
        c.model_position(0, servo_angle(4071)).unwrap();
        let mut last = f64::NEG_INFINITY;
        for raw in 4071..=4200 {
            let q = c.model_position(0, servo_angle(raw)).unwrap();
            assert!(q > last);
            last = q;
            let mut targets = [0.0; NUM_JOINTS];
            targets[0] = q;
            assert_eq!(raw_target(c.servo_targets(&targets).unwrap()[0]), raw);
        }
        // An unexpected reboot resets 4200 to 104; never silently rebase this live session.
        assert!(c.model_position(0, servo_angle(104)).is_err());
        c.invalidate_origin(20);
        assert!(c.servo_targets(&[0.0; NUM_JOINTS]).is_err());
        let q = c.model_position(0, servo_angle(104)).unwrap();
        assert!((q - 129.0 * RADIANS_PER_TICK).abs() < 1e-12);
        let mut targets = [0.0; NUM_JOINTS];
        targets[0] = 0.0;
        assert_eq!(raw_target(c.servo_targets(&targets).unwrap()[0]), -25);
    }

    #[test]
    fn every_installation_zero_maps_to_the_same_bounded_model_coordinates() {
        let limits = [-170f64.to_radians(), 170f64.to_radians()];
        for zero in 0..4096 {
            for pose in [-2.0, 0.0, 2.0] {
                let unwrapped = (zero as f64 + pose / RADIANS_PER_TICK).round() as i32;
                let boot = unwrapped.rem_euclid(4096);
                let mut c = extended(zero as f64, limits);
                let q = c.model_position(0, servo_angle(boot)).unwrap();
                assert!((q - pose).abs() <= RADIANS_PER_TICK / 2.0 + 1e-10);
                for goal in [-2.9, 0.0, 2.9] {
                    let mut targets = [0.0; NUM_JOINTS];
                    targets[0] = goal;
                    let raw = raw_target(c.servo_targets(&targets).unwrap()[0]);
                    // Continuous motor displacement equals model displacement for every zero.
                    let actual = (raw - boot) as f64 * RADIANS_PER_TICK;
                    assert!((actual - (goal - q)).abs() < RADIANS_PER_TICK + 1e-10);
                }
            }
        }
    }

    #[test]
    fn a_bus_reopen_recovers_origins_from_continuous_counts_too() {
        for turns in [-10, 0, 15] {
            let mut c = extended(4071.0, [-0.5, 0.5]);
            let raw = 4071 + turns * 4096;
            assert!(c.model_position(0, servo_angle(raw)).unwrap().abs() < 1e-10);
            assert_eq!(
                raw_target(c.servo_targets(&[0.0; NUM_JOINTS]).unwrap()[0]),
                raw
            );
        }
    }

    #[test]
    fn extended_requires_unambiguous_limits_and_rejects_outside_targets() {
        for extra in [
            "",
            r#", "limits_rad":[-3.141592653589793,3.141592653589793]"#,
            r#", "limits_rad":[1,-1]"#,
        ] {
            let json = format!(
                r#"{{"position_mode":"extended_position","joints":[{{"name":"left_hip_yaw","id":20,"zero_tick":4071{extra}}}]}}"#
            );
            assert!(JointCalibration::from_json(&json).is_err());
        }
        let mut c = extended(4071.0, [-0.5, 0.5]);
        assert!(c.model_position(0, servo_angle(2048)).is_err());
        c.model_position(0, servo_angle(4071)).unwrap();
        let mut targets = [0.0; NUM_JOINTS];
        targets[0] = 0.6;
        assert!(c.servo_targets(&targets).is_err());
    }
}
