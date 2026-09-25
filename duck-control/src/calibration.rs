//! Per-robot encoder zeroes, applied only at the hardware I/O boundary.
//!
//! Policies, home poses, IPC and kinematics keep their model-space radians. A constant
//! zero shift changes measured and commanded positions in opposite directions; it does
//! not change velocity, gains, action scales or the policy's default pose.

use std::f64::consts::PI;
use std::path::Path;

use serde::Deserialize;

use crate::io::{IoError, Result};
use crate::model::{JOINT_IDS, JOINT_NAMES, NUM_JOINTS, joint_index};

pub const RADIANS_PER_TICK: f64 = 2.0 * PI / 4096.0;

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct File {
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
    /// Existing servo EEPROM value. It is checked at startup, never written here.
    #[serde(default)]
    homing_offset_tick: i32,
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
    homing_offsets: [i32; NUM_JOINTS],
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
        let mut calibration = Self::default();
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
            if !joint.zero_tick.is_finite() || !(0.0..=4095.0).contains(&joint.zero_tick) {
                return Err(CalibrationError::Invalid(format!(
                    "{} zero_tick must be finite and in 0..=4095",
                    joint.name
                )));
            }
            calibration.offsets[index] = (joint.zero_tick - 2048.0) * RADIANS_PER_TICK;
            calibration.configured[index] = true;
            calibration.homing_offsets[index] = joint.homing_offset_tick;
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

    pub fn expected_homing_offset(&self, joint: usize) -> i32 {
        self.homing_offsets[joint]
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
    pub fn model_position(&self, joint: usize, servo_radians: f64) -> Result<f64> {
        let model = servo_radians - self.offsets[joint];
        // Safety clamps logical targets to +/-pi. Reject a corrected reading outside
        // that range before it can become a hold target and be clamped to a different
        // physical pose. Crossing the encoder seam needs a mounting/range check.
        if self.configured[joint] && (!model.is_finite() || !(-PI..=PI).contains(&model)) {
            return Err(IoError::Bus(format!(
                "calibrated reading for {} is outside model travel; check mounting/reference pose",
                JOINT_NAMES[joint]
            )));
        }
        Ok(model)
    }

    pub fn servo_targets(&self, model: &[f64; NUM_JOINTS]) -> Result<[f64; NUM_JOINTS]> {
        let mut servo = [0.0; NUM_JOINTS];
        for (joint, target) in model.iter().enumerate() {
            servo[joint] = target + self.offsets[joint];
            if !servo[joint].is_finite() {
                return Err(IoError::Bus(format!(
                    "non-finite target for {}",
                    JOINT_NAMES[joint]
                )));
            }
            // rustypot truncates the inverse conversion. Floating cancellation can make
            // an exact calibrated count (e.g. 1022) become 1021.9999999999999 and send
            // the preceding tick. Correct only roundoff next to an integer count;
            // preserve the existing truncation for genuinely fractional targets.
            if self.configured[joint] {
                let raw = 4096.0 * (PI + servo[joint]) / (2.0 * PI);
                let nearest = raw.round();
                if raw < nearest && nearest - raw < 1e-9 {
                    servo[joint] += RADIANS_PER_TICK * 1e-10;
                }
            }
            // Never wrap a shifted target through the single-turn encoder boundary:
            // the servo could travel almost a full revolution to a nearby logical angle.
            // Unconfigured joints retain the runtime's existing target range behavior.
            // Check the actual integer conversion rustypot sends. Checking a float
            // against 4095 would reject a valid endpoint at 4095 + roundoff.
            let tick = (4096.0 * (PI + servo[joint]) / (2.0 * PI)) as i32;
            if self.configured[joint] && !(0..=4095).contains(&tick) {
                return Err(IoError::Bus(format!(
                    "calibrated target for {} is outside single-turn travel ({tick} ticks)",
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
        let c = calibration();
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
        let c = calibration();
        assert_eq!(c.configured_names(), ["left_hip_yaw"]);
        assert_eq!(c.model_position(1, 0.3).unwrap(), 0.3);
        assert_eq!(c.servo_targets(&[0.3; NUM_JOINTS]).unwrap()[1], 0.3);
    }

    #[test]
    fn nonzero_reference_angle_produces_the_same_model_coordinates() {
        // Measured 2304 at model angle pi/8 => zero at 2048, not 2304.
        let zero = 2304.0 - (PI / 8.0) / RADIANS_PER_TICK;
        let c = JointCalibration::from_json(&format!(
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
    fn a_saved_zero_checks_the_existing_homing_offset() {
        let calibration = JointCalibration::from_json(
            r#"{"joints":[{"name":"left_knee","id":23,"zero_tick":1976,"homing_offset_tick":-585}]}"#,
        )
        .unwrap();
        assert_eq!(calibration.expected_homing_offset(3), -585);
        assert_eq!(calibration.expected_homing_offset(4), 0);
    }

    #[test]
    fn explicit_missing_file_does_not_use_factory_zeroes() {
        assert!(JointCalibration::load(Path::new("/definitely-missing-calibration.json")).is_err());
    }

    #[test]
    fn holding_a_measured_endpoint_stays_inside_the_encoder_range() {
        for zero in [0.0, 100.5, 2176.0, 4095.0] {
            let c = JointCalibration::from_json(&format!(
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
        let c = JointCalibration::from_json(&format!(
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
}
