//! The optional whole-body tracking controller.
//!
//! This is deliberately separate from [`crate::policy::Policy`]. The alpha family has one
//! fixed `61 -> 14` ABI and decodes actions around the home pose; the WBC graph is `72 -> 14`,
//! consumes a 24-float reference frame every tick, and decodes residuals around that frame's
//! joint pose. Combining those contracts behind the alpha constants would make a wrong model
//! look valid until it moved the robot.
//!
//! Like the established policy family, the ONNX path comes directly from `robotd`'s resolved
//! configuration. Its user-readable reference CSV is parsed once, before the realtime loop can
//! select WBC; each tick thereafter copies one already-parsed `[f32; 24]` frame.
//! `robotd` owns when this controller is active; this module owns the policy ABI, history,
//! reference clock and action decode.

use std::io::{BufRead, BufReader};
use std::path::{Path, PathBuf};

use ort::session::Session;
use ort::session::builder::GraphOptimizationLevel;
use ort::value::TensorRef;

use crate::io::Sensors;
use crate::model::{DEFAULT_POSITION, NUM_JOINTS};
use crate::obs::{ACTION_LEN, OBS_JOINTS, Observation, joint_of, policy_joints};
use crate::policy::{INTRA_THREADS, PolicyError, catching_ort_panics, check_width, ensure_runtime};

const WBC_REFERENCE_LEN: usize = 24;
const WBC_OBS_LEN: usize = 72;
const WBC_ACTION_LEN: usize = ACTION_LEN;
#[cfg(test)]
const WBC_HZ: f64 = 50.0;

#[derive(Debug, thiserror::Error)]
pub enum WbcError {
    #[error("reading WBC asset {path}: {source}")]
    Read {
        path: PathBuf,
        #[source]
        source: std::io::Error,
    },
    #[error("WBC asset {path}: {detail}")]
    Contract { path: PathBuf, detail: String },
    #[error(transparent)]
    Policy(#[from] PolicyError),
    #[error("WBC inference failed: {0}")]
    Inference(String),
}

/// One WBC tick. Targets still pass through `robotd`'s ordinary safety layer.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WbcStep {
    pub targets: [f64; NUM_JOINTS],
    pub gain: u16,
    /// This step consumed the final CSV row. The caller must leave WBC through its safe
    /// HOME transition before another control tick rather than holding or wrapping the clip.
    pub finished: bool,
}

/// One policy session plus the exact reference stream it tracks.
pub struct WbcController {
    session: Session,
    policy_path: PathBuf,
    reference: Vec<[f32; WBC_REFERENCE_LEN]>,
    frame: usize,
    last_action: [f32; WBC_ACTION_LEN],
    gain: u16,
}

impl WbcController {
    /// Load, shape-check and warm up the ONNX, and parse its reference CSV.
    pub fn load(policy_path: &Path, reference_path: &Path, gain: u16) -> Result<Self, WbcError> {
        let policy_path = policy_path.to_owned();
        let reference = load_reference_csv(reference_path)?;

        ensure_runtime()?;
        let policy_for_error = policy_path.clone();
        let mut session = catching_ort_panics(move || {
            let session = Session::builder()
                .and_then(|b| b.with_optimization_level(GraphOptimizationLevel::Level3))
                .and_then(|b| b.with_intra_threads(INTRA_THREADS))
                .and_then(|b| b.commit_from_file(&policy_for_error))
                .map_err(|source| PolicyError::Load {
                    path: policy_for_error.clone(),
                    source,
                })?;
            check_width(
                &policy_for_error,
                "observation width",
                session.inputs(),
                WBC_OBS_LEN,
            )?;
            check_width(
                &policy_for_error,
                "action count",
                session.outputs(),
                WBC_ACTION_LEN,
            )?;
            Ok(session)
        })?;
        // Pay ONNX Runtime's first-call cost before this controller can own a motor tick.
        let zero = [0.0f32; WBC_OBS_LEN];
        let _ = run(&mut session, &policy_path, &zero)?;

        Ok(Self {
            session,
            policy_path,
            reference,
            frame: 0,
            last_action: [0.0; WBC_ACTION_LEN],
            gain,
        })
    }

    pub fn reset(&mut self) {
        self.frame = 0;
        self.last_action = [0.0; WBC_ACTION_LEN];
    }

    #[cfg(test)]
    fn frame(&self) -> usize {
        self.frame
    }

    /// Run one 50 Hz step. One call consumes exactly one pre-parsed CSV row. The final row is
    /// marked on the returned step so `robotd` can hand ownership to its HOME transition; the
    /// reference is neither wrapped nor held indefinitely.
    pub fn step(&mut self, sensors: &Sensors) -> Result<WbcStep, WbcError> {
        let frame = self.frame;
        let Some(reference) = self.reference.get(frame).copied() else {
            return Err(WbcError::Inference(format!(
                "reference is exhausted after {} frames",
                self.reference.len()
            )));
        };
        let observation = build_observation(&reference, sensors, &self.last_action);
        let action = run(&mut self.session, &self.policy_path, &observation)?;
        if let Some((index, value)) = action.iter().enumerate().find(|(_, v)| !v.is_finite()) {
            return Err(WbcError::Inference(format!(
                "non-finite action at index {index}: {value}"
            )));
        }
        self.last_action = action;
        self.frame += 1;
        let finished = self.frame == self.reference.len();

        let targets = decode_targets(&reference, &action, sensors);
        Ok(WbcStep {
            targets,
            gain: self.gain,
            finished,
        })
    }
}

fn decode_targets(
    reference: &[f32; WBC_REFERENCE_LEN],
    action: &[f32; WBC_ACTION_LEN],
    sensors: &Sensors,
) -> [f64; NUM_JOINTS] {
    let residual = Observation::scatter_action(action);
    let mut targets = sensors.positions;
    for slot in 0..WBC_ACTION_LEN {
        let joint = joint_of(slot);
        // The graph already contains its normalizer and exports an unscaled joint residual.
        // This is deliberately not the alpha family's HOME + action_scale interpretation.
        targets[joint] = f64::from(reference[10 + slot]) + residual[joint];
    }
    targets
}

fn build_observation(
    reference: &[f32; WBC_REFERENCE_LEN],
    sensors: &Sensors,
    last_action: &[f32; WBC_ACTION_LEN],
) -> [f32; WBC_OBS_LEN] {
    let mut out = [0.0f32; WBC_OBS_LEN];
    out[..WBC_REFERENCE_LEN].copy_from_slice(reference);
    out[24..27].copy_from_slice(&sensors.imu.gyro.map(|v| v as f32));
    out[27..30].copy_from_slice(&sensors.imu.gravity.map(|v| v as f32));
    let positions = policy_joints(&sensors.positions);
    let home = policy_joints(&DEFAULT_POSITION);
    for i in 0..OBS_JOINTS {
        out[30 + i] = (positions[i] - home[i]) as f32;
    }
    out[44..58].copy_from_slice(&policy_joints(&sensors.velocities).map(|v| v as f32));
    out[58..72].copy_from_slice(last_action);
    out
}

fn load_reference_csv(path: &Path) -> Result<Vec<[f32; WBC_REFERENCE_LEN]>, WbcError> {
    let file = std::fs::File::open(path).map_err(|source| WbcError::Read {
        path: path.to_owned(),
        source,
    })?;
    parse_reference_csv(BufReader::new(file), path)
}

fn parse_reference_csv(
    reader: impl BufRead,
    path: &Path,
) -> Result<Vec<[f32; WBC_REFERENCE_LEN]>, WbcError> {
    let mut frames = Vec::new();
    for (frame_index, line) in reader.lines().enumerate() {
        let line_number = frame_index + 1;
        let line = line.map_err(|source| WbcError::Read {
            path: path.to_owned(),
            source,
        })?;
        if line.trim().is_empty() {
            return Err(WbcError::Contract {
                path: path.to_owned(),
                detail: format!("reference row {line_number} is empty"),
            });
        }
        let columns: Vec<&str> = line.split(',').collect();
        if columns.len() != WBC_REFERENCE_LEN {
            return Err(WbcError::Contract {
                path: path.to_owned(),
                detail: format!(
                    "reference row {line_number} has {} columns, expected {WBC_REFERENCE_LEN}",
                    columns.len()
                ),
            });
        }
        let mut frame = [0.0f32; WBC_REFERENCE_LEN];
        for (slot, encoded) in columns.iter().enumerate() {
            let value = encoded
                .trim()
                .parse::<f32>()
                .map_err(|e| WbcError::Contract {
                    path: path.to_owned(),
                    detail: format!(
                        "reference row {line_number} column {} is not f32: {e}",
                        slot + 1
                    ),
                })?;
            if !value.is_finite() {
                return Err(WbcError::Contract {
                    path: path.to_owned(),
                    detail: format!(
                        "reference row {line_number} column {} is not finite",
                        slot + 1
                    ),
                });
            }
            frame[slot] = value;
        }
        frames.push(frame);
    }
    if frames.is_empty() {
        return Err(WbcError::Contract {
            path: path.to_owned(),
            detail: "reference CSV is empty".into(),
        });
    }
    Ok(frames)
}

fn run(
    session: &mut Session,
    path: &Path,
    observation: &[f32; WBC_OBS_LEN],
) -> Result<[f32; WBC_ACTION_LEN], WbcError> {
    // Borrow the fixed observation buffer. The application performs no per-tick input Vec
    // allocation; ONNX Runtime owns whatever workspace its graph execution requires.
    let input = TensorRef::from_array_view(([1usize, WBC_OBS_LEN], &observation[..]))
        .map_err(|e| WbcError::Inference(format!("{}: building input: {e}", path.display())))?;
    let outputs = session
        .run(ort::inputs![input])
        .map_err(|e| WbcError::Inference(format!("{}: {e}", path.display())))?;
    let value = outputs
        .values()
        .next()
        .ok_or_else(|| WbcError::Inference(format!("{}: no output", path.display())))?;
    let (_, data) = value
        .try_extract_tensor::<f32>()
        .map_err(|e| WbcError::Inference(format!("{}: extracting output: {e}", path.display())))?;
    if data.len() != WBC_ACTION_LEN {
        return Err(WbcError::Inference(format!(
            "{}: {} actions, expected {WBC_ACTION_LEN}",
            path.display(),
            data.len()
        )));
    }
    let mut actions = [0.0f32; WBC_ACTION_LEN];
    actions.copy_from_slice(data);
    Ok(actions)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::imu::ImuData;
    use crate::model::MOUTH_INDEX;
    use std::io::Cursor;

    fn reference() -> [f32; WBC_REFERENCE_LEN] {
        std::array::from_fn(|i| (i + 1) as f32)
    }

    #[test]
    fn observation_blocks_match_the_exported_72_value_contract() {
        let sensors = Sensors {
            positions: std::array::from_fn(|i| DEFAULT_POSITION[i] + i as f64 * 0.01),
            velocities: std::array::from_fn(|i| i as f64 + 0.5),
            imu: ImuData {
                gyro: [101.0, 102.0, 103.0],
                gravity: [201.0, 202.0, 203.0],
                ..ImuData::default()
            },
            ..Sensors::default()
        };
        let last = std::array::from_fn(|i| 300.0 + i as f32);
        let obs = build_observation(&reference(), &sensors, &last);
        assert_eq!(&obs[..24], &reference());
        assert_eq!(&obs[24..30], &[101.0, 102.0, 103.0, 201.0, 202.0, 203.0]);
        assert_eq!(&obs[58..72], &last);
        assert_eq!(obs[30], 0.0);
        assert!(
            (obs[39] - 0.1).abs() < 1e-6,
            "mouth is skipped, so slot 9 is joint 10"
        );
        assert_eq!(obs[44], 0.5);
        assert_eq!(obs[53], 10.5, "mouth velocity is skipped too");
    }

    #[test]
    fn action_decode_is_reference_residual_and_leaves_the_mouth_alone() {
        let reference = reference();
        let action = [0.25f32; WBC_ACTION_LEN];
        let sensors = Sensors {
            positions: [7.0; NUM_JOINTS],
            ..Sensors::default()
        };
        let targets = decode_targets(&reference, &action, &sensors);
        assert_eq!(targets[MOUTH_INDEX], 7.0);
        assert_eq!(targets[0], 11.25);
        assert_eq!(targets[10], 20.25);
    }

    #[test]
    fn the_reference_and_observation_widths_are_exact() {
        assert_eq!(WBC_REFERENCE_LEN, 24);
        assert_eq!(WBC_REFERENCE_LEN + 3 + 3 + OBS_JOINTS * 3, WBC_OBS_LEN);
        assert_eq!(WBC_ACTION_LEN, OBS_JOINTS);
    }

    #[test]
    fn the_shipped_reference_is_the_expected_complete_clip() {
        let reference = Path::new(env!("CARGO_MANIFEST_DIR")).join("../policies/wbc_happy.csv");
        let frames = load_reference_csv(&reference).expect("reference stream");
        assert_eq!(frames.len(), 989);
        assert_eq!(
            frames[0], frames[988],
            "the deploy clip starts and ends at HOME"
        );
    }

    fn row() -> String {
        (0..WBC_REFERENCE_LEN)
            .map(|value| value.to_string())
            .collect::<Vec<_>>()
            .join(",")
    }

    #[test]
    fn csv_reference_is_parsed_once_into_fixed_width_finite_frames() {
        let row = row();
        let frames = parse_reference_csv(
            Cursor::new(format!("{row}\n{row}\n")),
            Path::new("test.csv"),
        )
        .unwrap();
        assert_eq!(frames.len(), 2);
        assert_eq!(frames[0][0], 0.0);
        assert_eq!(frames[0][23], 23.0);
    }

    #[test]
    fn csv_reference_rejects_empty_wrong_width_header_and_nonfinite_values() {
        assert!(
            parse_reference_csv(Cursor::new(""), Path::new("empty.csv"))
                .unwrap_err()
                .to_string()
                .contains("empty")
        );

        let error = parse_reference_csv(Cursor::new("0,1,2\n"), Path::new("short.csv"))
            .unwrap_err()
            .to_string();
        assert!(error.contains("row 1 has 3 columns"), "{error}");

        let mut header = vec!["0"; WBC_REFERENCE_LEN];
        header[0] = "ref_base_height";
        let error = parse_reference_csv(
            Cursor::new(header.join(",") + "\n"),
            Path::new("header.csv"),
        )
        .unwrap_err()
        .to_string();
        assert!(error.contains("row 1 column 1 is not f32"), "{error}");

        let mut nonfinite = vec!["0"; WBC_REFERENCE_LEN];
        nonfinite[7] = "NaN";
        let error = parse_reference_csv(
            Cursor::new(nonfinite.join(",") + "\n"),
            Path::new("nonfinite.csv"),
        )
        .unwrap_err()
        .to_string();
        assert!(error.contains("row 1 column 8 is not finite"), "{error}");
    }

    /// This is ignored in ordinary CI because it needs a compatible `libonnxruntime.so`.
    /// Run it explicitly on a host/board with `ORT_DYLIB_PATH` to execute the exact shipped
    /// graph for all 989 reference rows and check the previous-action and frame-clock contract.
    #[test]
    #[ignore = "requires ONNX Runtime >= 1.23 via ORT_DYLIB_PATH"]
    fn shipped_onnx_runs_every_csv_frame_once_with_finite_raw_actions() {
        let directory = Path::new(env!("CARGO_MANIFEST_DIR")).join("../policies");
        let policy = directory.join("wbc_v1.onnx");
        let reference = directory.join("wbc_happy.csv");
        let mut controller =
            WbcController::load(&policy, &reference, 200).expect("load shipped WBC assets");
        assert_eq!(controller.reference.len(), 989);
        assert_eq!(controller.last_action, [0.0; WBC_ACTION_LEN]);

        let mut sensors = Sensors::default();
        let mut previous_joint_reference = controller.reference[0][10..24].to_vec();
        for tick in 0..controller.reference.len() {
            let reference = controller.reference[tick];
            sensors.imu.gyro = std::array::from_fn(|i| f64::from(reference[4 + i]));
            sensors.imu.gravity = std::array::from_fn(|i| f64::from(reference[7 + i]));
            for slot in 0..WBC_ACTION_LEN {
                let joint = joint_of(slot);
                sensors.positions[joint] = f64::from(reference[10 + slot]);
                sensors.velocities[joint] =
                    f64::from(reference[10 + slot] - previous_joint_reference[slot]) * WBC_HZ;
            }
            previous_joint_reference.copy_from_slice(&reference[10..24]);

            let previous_action = controller.last_action;
            let observation = build_observation(&reference, &sensors, &previous_action);
            assert_eq!(&observation[..24], &reference);
            assert_eq!(&observation[58..72], &previous_action);
            assert!(observation.iter().all(|value| value.is_finite()));

            let step = controller.step(&sensors).expect("ONNX inference");
            assert_eq!(controller.frame(), tick + 1, "one CSV row per 20 ms tick");
            assert_eq!(step.finished, tick == 988);
            assert!(controller.last_action.iter().all(|value| value.is_finite()));
            assert!(step.targets.iter().all(|value| value.is_finite()));
            for slot in 0..WBC_ACTION_LEN {
                let joint = joint_of(slot);
                let expected = f64::from(reference[10 + slot] + controller.last_action[slot]);
                assert!((step.targets[joint] - expected).abs() < 1e-6);
            }
        }
    }
}
