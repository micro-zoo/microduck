//! Capture a fixture zero through robotd's existing state stream.
//! This client never opens the motor UART, sends an intent or writes EEPROM.

use std::io::Write;
use std::path::Path;
use std::time::{Duration, Instant};

use duck_ipc_proto as proto;
use serde::Serialize;

use crate::{Client, Failure, decode, exit, result_of};

const FRAMES: usize = 40;
const HZ: u32 = 10;
const MAX_SPREAD_TICKS: i32 = 2;
const TICKS_PER_RADIAN: f64 = 4096.0 / (2.0 * std::f64::consts::PI);
const MOUTH_CLOSED_RAD: f64 = -5.0 * std::f64::consts::PI / 180.0;

#[derive(Serialize)]
struct ZeroFile {
    joints: Vec<JointZero>,
}

#[derive(Serialize)]
struct JointZero {
    name: &'static str,
    id: u8,
    zero_tick: f64,
    #[serde(skip_serializing_if = "Option::is_none")]
    homing_offset_tick: Option<i32>,
}

struct Frame {
    t_ns: u64,
    policy: String,
    joints: [f64; proto::JOINT_NAMES.len()],
}

impl TryFrom<proto::RobotState> for Frame {
    type Error = Failure;

    fn try_from(state: proto::RobotState) -> Result<Self, Failure> {
        let joints = state.joints.try_into().map_err(|joints: Vec<f64>| {
            Failure::new(
                exit::FAILED,
                format!("robotd reported {} joints, expected 15", joints.len()),
            )
        })?;
        Ok(Self {
            t_ns: state.t_ns,
            policy: state.policy,
            joints,
        })
    }
}

fn candidate(info: &proto::CalibrationInfo, frames: &[Frame]) -> Result<(ZeroFile, i32), Failure> {
    if frames.len() != FRAMES {
        return Err(Failure::new(
            exit::FAILED,
            format!("need {FRAMES} fresh state frames, got {}", frames.len()),
        ));
    }
    if info.policy_enabled || info.homed {
        return Err(Failure::new(
            exit::REFUSED,
            "fixture capture needs robotd --no-policy and no powered HOME pose".into(),
        ));
    }
    if let Some(index) = info.single_turn_compatible.iter().position(|ready| !ready) {
        return Err(Failure::new(
            exit::REFUSED,
            format!(
                "{} is not an XL330 in single-turn mode with its expected drive mode, limits and homing offset range",
                proto::JOINT_NAMES[index]
            ),
        ));
    }

    let mut last_t_ns = 0;
    let mut samples: [Vec<i32>; proto::JOINT_NAMES.len()] = std::array::from_fn(|_| Vec::new());
    for frame in frames {
        if frame.t_ns <= last_t_ns || frame.policy != "held" {
            return Err(Failure::new(
                exit::REFUSED,
                "state stream was stale or the policy was driving during fixture capture".into(),
            ));
        }
        last_t_ns = frame.t_ns;
        for (joint, model_angle) in frame.joints.iter().enumerate() {
            let raw = info.zero_ticks[joint] + model_angle * TICKS_PER_RADIAN;
            let rounded = raw.round();
            if !raw.is_finite()
                || !(0.0..=4095.0).contains(&rounded)
                || (raw - rounded).abs() > 1e-6
            {
                return Err(Failure::new(
                    exit::REFUSED,
                    format!(
                        "{} has no unambiguous single-turn encoder count; stop and inspect its origin",
                        proto::JOINT_NAMES[joint]
                    ),
                ));
            }
            samples[joint].push(rounded as i32);
        }
    }

    let mut largest_spread = 0;
    let mut joints = Vec::with_capacity(proto::JOINT_NAMES.len());
    for (joint, values) in samples.iter_mut().enumerate() {
        values.sort_unstable();
        let spread = values[FRAMES - 1] - values[0];
        largest_spread = largest_spread.max(spread);
        if spread > MAX_SPREAD_TICKS {
            return Err(Failure::new(
                exit::REFUSED,
                format!(
                    "{} moved {spread} encoder ticks during capture; keep it on the q=0 fixture and retry",
                    proto::JOINT_NAMES[joint]
                ),
            ));
        }
        let median = (f64::from(values[FRAMES / 2 - 1]) + f64::from(values[FRAMES / 2])) / 2.0;
        let reference = if proto::JOINT_NAMES[joint] == "mouth" {
            MOUTH_CLOSED_RAD
        } else {
            0.0
        };
        let zero_tick = median - reference * TICKS_PER_RADIAN;
        if !(0.0..=4095.0).contains(&zero_tick) {
            return Err(Failure::new(
                exit::REFUSED,
                format!(
                    "{} zero falls outside single-turn travel",
                    proto::JOINT_NAMES[joint]
                ),
            ));
        }
        joints.push(JointZero {
            name: proto::JOINT_NAMES[joint],
            id: proto::JOINT_IDS[joint],
            zero_tick,
            homing_offset_tick: (info.homing_offset_ticks[joint] != 0)
                .then_some(info.homing_offset_ticks[joint]),
        });
    }
    Ok((ZeroFile { joints }, largest_spread))
}

pub fn capture_zero(socket: &Path, output: &Path, fixture_q0: bool) -> Result<(), Failure> {
    if !fixture_q0 {
        return Err(Failure::new(
            exit::USAGE,
            "place and support the robot in its q=0 fixture, then pass --fixture-q0".into(),
        ));
    }
    let parent = output
        .parent()
        .filter(|parent| !parent.as_os_str().is_empty())
        .unwrap_or(Path::new("."));
    if !parent.is_dir() || output.file_name().is_none() {
        return Err(Failure::new(
            exit::USAGE,
            "--output needs an existing directory and file name".into(),
        ));
    }
    if output.exists() {
        return Err(Failure::new(
            exit::REFUSED,
            format!(
                "{} already exists; choose a new candidate path",
                output.display()
            ),
        ));
    }

    let mut client = Client::connect_to("robotd", socket)?;
    client.set_read_timeout(Duration::from_secs(2))?;
    client.hello()?;
    let info: proto::CalibrationInfo = decode(&result_of(
        client.call(&proto::Call::RobotCalibrationInfo)?,
    )?)?;
    if info.policy_enabled || info.homed {
        return Err(Failure::new(
            exit::REFUSED,
            "fixture capture needs robotd --no-policy and no powered HOME pose".into(),
        ));
    }
    let subscribed: proto::SubscribeResult = decode(&result_of(client.call(
        &proto::Call::RobotSubscribe(proto::SubscribeParams { hz: Some(HZ) }),
    )?)?)?;
    if !subscribed.accepted {
        return Err(Failure::new(
            exit::REFUSED,
            "robotd refused the state subscription".into(),
        ));
    }

    let deadline = Instant::now() + Duration::from_secs(12);
    let mut frames = Vec::with_capacity(FRAMES);
    while frames.len() < FRAMES {
        if Instant::now() >= deadline {
            return Err(Failure::new(
                exit::UNREACHABLE,
                "timed out waiting for 40 fresh robotd frames".into(),
            ));
        }
        let frame = Frame::try_from(client.next_robot_state()?)?;
        if frames
            .last()
            .is_some_and(|last: &Frame| frame.t_ns <= last.t_ns)
        {
            continue; // A coasted read repeats its sensor timestamp.
        }
        frames.push(frame);
    }
    let after: proto::CalibrationInfo = decode(&result_of(
        client.call(&proto::Call::RobotCalibrationInfo)?,
    )?)?;
    if after != info {
        return Err(Failure::new(
            exit::REFUSED,
            "robot calibration or power state changed during capture".into(),
        ));
    }
    let (candidate, spread) = candidate(&info, &frames)?;
    let mut data = serde_json::to_vec_pretty(&candidate)
        .map_err(|e| Failure::new(exit::FAILED, format!("could not encode calibration: {e}")))?;
    data.push(b'\n');

    let mut temporary = tempfile::NamedTempFile::new_in(parent)
        .map_err(|e| Failure::new(exit::FAILED, format!("could not create candidate: {e}")))?;
    temporary
        .write_all(&data)
        .and_then(|()| temporary.as_file().sync_all())
        .map_err(|e| Failure::new(exit::FAILED, format!("could not write candidate: {e}")))?;
    temporary.persist_noclobber(output).map_err(|e| {
        Failure::new(
            exit::REFUSED,
            format!(
                "could not save {} without overwriting: {}",
                output.display(),
                e.error
            ),
        )
    })?;
    println!(
        "captured {} joint zeroes from {FRAMES} fresh robotd frames (max spread {spread} ticks): {}",
        candidate.joints.len(),
        output.display()
    );
    println!(
        "candidate only; set bus.calibration with robotctl configure and restart robotd to apply it"
    );
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn loaded_offsets_are_inverted_before_recapturing_fixture_zero() {
        let mut info = proto::CalibrationInfo {
            zero_ticks: [2048.0; 15],
            homing_offset_ticks: [0; 15],
            single_turn_compatible: [true; 15],
            policy_enabled: false,
            homed: false,
        };
        info.zero_ticks[3] = 1976.0;
        info.homing_offset_ticks[3] = -585;
        info.zero_ticks[9] = 2100.888888888889;
        let frames = (1..=FRAMES)
            .map(|i| {
                let mut joints = [0.0; 15];
                joints[3] = (1980.0 - info.zero_ticks[3]) / TICKS_PER_RADIAN;
                joints[9] = (2044.0 - info.zero_ticks[9]) / TICKS_PER_RADIAN;
                Frame {
                    t_ns: i as u64,
                    policy: "held".into(),
                    joints,
                }
            })
            .collect::<Vec<_>>();
        let (file, spread) = candidate(&info, &frames).unwrap_or_else(|e| panic!("{}", e.message));
        assert_eq!(spread, 0);
        assert_eq!(file.joints[3].zero_tick, 1980.0);
        assert_eq!(file.joints[3].homing_offset_tick, Some(-585));
        assert!((file.joints[9].zero_tick - 2100.888888888889).abs() < 1e-9);
    }

    #[test]
    fn moving_or_ambiguous_encoder_samples_are_refused() {
        let info = proto::CalibrationInfo {
            zero_ticks: [2048.0; 15],
            homing_offset_ticks: [0; 15],
            single_turn_compatible: [true; 15],
            policy_enabled: false,
            homed: false,
        };
        let mut frames = (1..=FRAMES)
            .map(|i| Frame {
                t_ns: i as u64,
                policy: "held".into(),
                joints: [0.0; 15],
            })
            .collect::<Vec<_>>();
        frames[FRAMES - 1].joints[0] = 4.0 / TICKS_PER_RADIAN;
        assert!(candidate(&info, &frames).is_err());
        frames[FRAMES - 1].joints[0] = 0.0;
        frames[0].joints[0] = 4096.0 / TICKS_PER_RADIAN;
        assert!(candidate(&info, &frames).is_err());
    }
}
