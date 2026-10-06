//! Capture this robot's hardware calibration through robotd's existing state stream: the
//! fixture joint zeroes, and the body IMU's mount. Both land in one per-robot file.
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
    /// Carried over from the calibration robotd loaded, so recapturing the zeroes does not lose
    /// the IMU mount measured before.
    #[serde(skip_serializing_if = "Option::is_none")]
    body_imu: Option<BodyImu>,
}

#[derive(Serialize)]
struct BodyImu {
    mount: [f64; 4],
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
    let body_imu = info
        .body_imu_mount
        .filter(|_| info.body_imu_mount_calibrated)
        .map(|mount| BodyImu { mount });
    Ok((ZeroFile { joints, body_imu }, largest_spread))
}

/// Where a candidate goes: a new file in an existing directory. Returns the directory.
fn candidate_destination(output: &Path) -> Result<&Path, Failure> {
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
    Ok(parent)
}

/// Write a candidate without ever replacing a file, through a synced temporary.
fn persist_candidate(parent: &Path, output: &Path, data: &[u8]) -> Result<(), Failure> {
    let mut temporary = tempfile::NamedTempFile::new_in(parent)
        .map_err(|e| Failure::new(exit::FAILED, format!("could not create candidate: {e}")))?;
    temporary
        .write_all(data)
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
    Ok(())
}

pub fn capture_zero(socket: &Path, output: &Path, fixture_q0: bool) -> Result<(), Failure> {
    if !fixture_q0 {
        return Err(Failure::new(
            exit::USAGE,
            "place and support the robot in its q=0 fixture, then pass --fixture-q0".into(),
        ));
    }
    let parent = candidate_destination(output)?;

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
    persist_candidate(parent, output, &data)?;
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

/// Frames averaged per pose: two seconds at [`HZ`].
const IMU_FRAMES: usize = 20;
/// Rotation rate above which a frame is not "still", rad/s.
const IMU_STILL_RAD_S: f64 = 0.05;
/// How far the gravity direction may wander within one pose, degrees.
const IMU_MAX_SPREAD_DEG: f64 = 1.0;

/// Average gravity, in the sensor's own frame, over a still stretch of fresh frames.
///
/// A fresh connection per pose, so nothing buffered while the operator was moving the robot is
/// counted as the pose they then held.
fn still_sensor_gravity(socket: &Path, mount: [f64; 4]) -> Result<[f64; 3], Failure> {
    let mut client = Client::connect_to("robotd", socket)?;
    client.set_read_timeout(Duration::from_secs(2))?;
    client.hello()?;
    let subscribed: proto::SubscribeResult = decode(&result_of(client.call(
        &proto::Call::RobotSubscribe(proto::SubscribeParams { hz: Some(HZ) }),
    )?)?)?;
    if !subscribed.accepted {
        return Err(Failure::new(
            exit::REFUSED,
            "robotd refused the state subscription".into(),
        ));
    }
    let deadline = Instant::now() + Duration::from_secs(8);
    let mut samples: Vec<[f64; 3]> = Vec::with_capacity(IMU_FRAMES);
    let mut last_t_ns = 0;
    while samples.len() < IMU_FRAMES {
        if Instant::now() >= deadline {
            return Err(Failure::new(
                exit::UNREACHABLE,
                format!("timed out waiting for {IMU_FRAMES} fresh robotd frames"),
            ));
        }
        let state = client.next_robot_state()?;
        if state.t_ns <= last_t_ns {
            continue;
        }
        last_t_ns = state.t_ns;
        if let Some(imu) = &state.imu {
            let rate = imu.gyro.iter().map(|v| v * v).sum::<f64>().sqrt();
            if rate > IMU_STILL_RAD_S {
                return Err(Failure::new(
                    exit::REFUSED,
                    format!("the robot turned at {rate:.2} rad/s; hold it still and retry"),
                ));
            }
        }
        samples.push(proto::mount::sensor_gravity(mount, state.safety.gravity));
    }
    let mean = average_direction(&samples).ok_or_else(|| {
        Failure::new(
            exit::FAILED,
            "gravity samples do not average to a direction".into(),
        )
    })?;
    let spread = samples
        .iter()
        .map(|g| angle_deg(*g, mean))
        .fold(0.0f64, f64::max);
    if spread > IMU_MAX_SPREAD_DEG {
        return Err(Failure::new(
            exit::REFUSED,
            format!("gravity wandered {spread:.1}° while held; hold the robot still and retry"),
        ));
    }
    Ok(mean)
}

fn average_direction(samples: &[[f64; 3]]) -> Option<[f64; 3]> {
    let sum = samples.iter().fold([0.0; 3], |acc, g| {
        let n = g.iter().map(|v| v * v).sum::<f64>().sqrt();
        [0, 1, 2].map(|i| acc[i] + g[i] / n)
    });
    let n = sum.iter().map(|v| v * v).sum::<f64>().sqrt();
    (n > 1e-9).then(|| sum.map(|v| v / n))
}

fn angle_deg(a: [f64; 3], b: [f64; 3]) -> f64 {
    let na = a.iter().map(|v| v * v).sum::<f64>().sqrt();
    let nb = b.iter().map(|v| v * v).sum::<f64>().sqrt();
    let cos = (a[0] * b[0] + a[1] * b[1] + a[2] * b[2]) / (na * nb);
    cos.clamp(-1.0, 1.0).acos().to_degrees()
}

/// How far apart two mounts are, as one rotation angle in degrees.
fn mount_change_deg(a: [f64; 4], b: [f64; 4]) -> f64 {
    let d: f64 = a.iter().zip(b).map(|(x, y)| x * y).sum();
    (2.0 * d.abs().clamp(0.0, 1.0).acos()).to_degrees()
}

/// The calibration file with `body_imu.mount` set, everything else in it untouched.
fn with_mount(mut base: serde_json::Value, mount: [f64; 4]) -> Result<serde_json::Value, Failure> {
    let object = base.as_object_mut().ok_or_else(|| {
        Failure::new(
            exit::REFUSED,
            "the existing calibration is not a JSON object".into(),
        )
    })?;
    object
        .entry("joints")
        .or_insert_with(|| serde_json::json!([]));
    object.insert("body_imu".into(), serde_json::json!({ "mount": mount }));
    Ok(base)
}

fn wait_for_enter(prompt: &str) -> Result<(), Failure> {
    print!("{prompt} — press Enter when it is still: ");
    std::io::stdout()
        .flush()
        .map_err(|e| Failure::new(exit::FAILED, format!("stdout: {e}")))?;
    let mut line = String::new();
    std::io::stdin()
        .read_line(&mut line)
        .map_err(|e| Failure::new(exit::FAILED, format!("stdin: {e}")))?;
    Ok(())
}

/// Measure the body IMU's mount from two held poses and write it into a new calibration candidate
/// alongside the joint zeroes this robot already has.
pub fn capture_imu(
    socket: &Path,
    output: &Path,
    from: Option<&Path>,
    config: &Path,
) -> Result<(), Failure> {
    let parent = candidate_destination(output)?;
    // The joint zeroes carry over: from the file named, else the one robotd.toml points at.
    let base_path = match from {
        Some(path) => Some(path.to_path_buf()),
        None => robotd_params::Params::load(config, false)
            .ok()
            .and_then(|params| params.bus.calibration_path().map(Path::to_path_buf)),
    };
    let base = match &base_path {
        Some(path) => {
            let text = std::fs::read_to_string(path).map_err(|e| {
                Failure::new(exit::FAILED, format!("cannot read {}: {e}", path.display()))
            })?;
            serde_json::from_str(&text).map_err(|e| {
                Failure::new(
                    exit::REFUSED,
                    format!("{} is not JSON: {e}", path.display()),
                )
            })?
        }
        None => serde_json::json!({ "joints": [] }),
    };

    let mut client = Client::connect_to("robotd", socket)?;
    client.set_read_timeout(Duration::from_secs(2))?;
    client.hello()?;
    let info: proto::CalibrationInfo = decode(&result_of(
        client.call(&proto::Call::RobotCalibrationInfo)?,
    )?)?;
    let current = info.body_imu_mount.ok_or_else(|| {
        Failure::new(
            exit::REFUSED,
            "this robotd does not report its IMU mount (it predates API v42); update it first"
                .into(),
        )
    })?;
    drop(client);

    println!("Two poses, two seconds each. The legs can be limp; only the trunk matters.");
    wait_for_enter("1/2  Hold the trunk upright, as it stands")?;
    let upright = still_sensor_gravity(socket, current)?;
    wait_for_enter("2/2  Pitch it nose-down 20–40°, straight forward with no roll")?;
    let nose_down = still_sensor_gravity(socket, current)?;
    let mount = proto::mount::mount_from_gravity(upright, nose_down)
        .map_err(|e| Failure::new(exit::REFUSED, e.to_string()))?;

    let candidate = with_mount(base, mount)?;
    let mut data = serde_json::to_vec_pretty(&candidate)
        .map_err(|e| Failure::new(exit::FAILED, format!("could not encode calibration: {e}")))?;
    data.push(b'\n');
    persist_candidate(parent, output, &data)?;
    println!(
        "measured body IMU mount {mount:?} (tilt {:.1}°, {:.1}° from the mount in effect): {}",
        angle_deg(upright, nose_down),
        mount_change_deg(mount, current),
        output.display()
    );
    match &base_path {
        Some(path) => println!("joint zeroes carried over from {}", path.display()),
        None => {
            println!("no calibration to carry joint zeroes from; this file holds the mount only")
        }
    }
    println!(
        "candidate only; set bus.calibration with robotctl configure and restart robotd to apply it"
    );
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::{BufRead, BufReader};
    use std::os::unix::fs::PermissionsExt;
    use std::os::unix::net::UnixListener;

    #[test]
    fn loaded_offsets_are_inverted_before_recapturing_fixture_zero() {
        let mut info = proto::CalibrationInfo {
            zero_ticks: [2048.0; 15],
            homing_offset_ticks: [0; 15],
            single_turn_compatible: [true; 15],
            policy_enabled: false,
            homed: false,
            body_imu_mount: None,
            body_imu_mount_calibrated: false,
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
    fn recapturing_zeroes_keeps_a_calibrated_imu_mount_and_only_that() {
        let frames = (1..=FRAMES)
            .map(|i| Frame {
                t_ns: i as u64,
                policy: "held".into(),
                joints: [0.0; 15],
            })
            .collect::<Vec<_>>();
        let mut info = proto::CalibrationInfo {
            zero_ticks: [2048.0; 15],
            homing_offset_ticks: [0; 15],
            single_turn_compatible: [true; 15],
            policy_enabled: false,
            homed: false,
            body_imu_mount: Some([0.5, -0.5, 0.5, -0.5]),
            body_imu_mount_calibrated: true,
        };
        let (file, _) = candidate(&info, &frames).unwrap_or_else(|e| panic!("{}", e.message));
        assert_eq!(file.body_imu.map(|b| b.mount), Some([0.5, -0.5, 0.5, -0.5]));
        // A mount from robotd.toml or the default is not this robot's measurement.
        info.body_imu_mount_calibrated = false;
        let (file, _) = candidate(&info, &frames).unwrap_or_else(|e| panic!("{}", e.message));
        assert!(file.body_imu.is_none());
    }

    #[test]
    fn the_measured_mount_joins_the_existing_calibration_untouched() {
        let base = serde_json::json!({
            "joints": [{"name": "left_knee", "id": 23, "zero_tick": 1976.0, "homing_offset_tick": -585}],
            "body_imu": {"mount": [1.0, 0.0, 0.0, 0.0]},
        });
        let out = with_mount(base.clone(), [0.5, -0.5, 0.5, -0.5])
            .unwrap_or_else(|e| panic!("{}", e.message));
        assert_eq!(out["joints"], base["joints"]);
        assert_eq!(
            out["body_imu"]["mount"],
            serde_json::json!([0.5, -0.5, 0.5, -0.5])
        );
        let empty = with_mount(serde_json::json!({}), [1.0, 0.0, 0.0, 0.0])
            .unwrap_or_else(|e| panic!("{}", e.message));
        assert_eq!(empty["joints"], serde_json::json!([]));
        assert!(with_mount(serde_json::json!([]), [1.0, 0.0, 0.0, 0.0]).is_err());
    }

    #[test]
    fn mount_change_and_spread_are_angles() {
        let h = std::f64::consts::FRAC_1_SQRT_2;
        assert!(mount_change_deg([1.0, 0.0, 0.0, 0.0], [-1.0, 0.0, 0.0, 0.0]).abs() < 1e-6);
        assert!((mount_change_deg([1.0, 0.0, 0.0, 0.0], [h, 0.0, h, 0.0]) - 90.0).abs() < 1e-6);
        assert!((angle_deg([0.0, 0.0, -1.0], [1.0, 0.0, 0.0]) - 90.0).abs() < 1e-9);
        let mean = average_direction(&[[0.0, 0.0, -2.0], [0.0, 0.0, -1.0]]).unwrap();
        assert!((mean[2] + 1.0).abs() < 1e-12);
    }

    #[test]
    fn moving_or_ambiguous_encoder_samples_are_refused() {
        let info = proto::CalibrationInfo {
            zero_ticks: [2048.0; 15],
            homing_offset_ticks: [0; 15],
            single_turn_compatible: [true; 15],
            policy_enabled: false,
            homed: false,
            body_imu_mount: None,
            body_imu_mount_calibrated: false,
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

    #[test]
    fn capture_uses_the_robotd_socket_and_never_overwrites_a_candidate() {
        let dir = tempfile::tempdir().unwrap();
        let socket = dir.path().join("robotd.sock");
        let output = dir.path().join("candidate.json");
        let listener = UnixListener::bind(&socket).unwrap();
        let server = std::thread::spawn(move || {
            fn answer<T: serde::Serialize>(
                reader: &mut BufReader<std::os::unix::net::UnixStream>,
                stream: &mut std::os::unix::net::UnixStream,
                expected: &str,
                result: &T,
            ) {
                let mut line = String::new();
                reader.read_line(&mut line).unwrap();
                let request: proto::Request = serde_json::from_str(&line).unwrap();
                assert_eq!(request.method, expected);
                let response = proto::Response::ok(request.id, result);
                writeln!(stream, "{}", serde_json::to_string(&response).unwrap()).unwrap();
            }
            let (mut stream, _) = listener.accept().unwrap();
            let mut reader = BufReader::new(stream.try_clone().unwrap());
            answer(
                &mut reader,
                &mut stream,
                proto::method::HELLO,
                &proto::HelloResult {
                    api_version: proto::API_VERSION,
                    daemon_version: None,
                    revision: None,
                },
            );
            answer(
                &mut reader,
                &mut stream,
                proto::method::ROBOT_CALIBRATION_INFO,
                &proto::CalibrationInfo {
                    zero_ticks: [2048.0; 15],
                    homing_offset_ticks: [0; 15],
                    single_turn_compatible: [true; 15],
                    policy_enabled: false,
                    homed: false,
                    body_imu_mount: None,
                    body_imu_mount_calibrated: false,
                },
            );
            answer(
                &mut reader,
                &mut stream,
                proto::method::ROBOT_SUBSCRIBE,
                &proto::SubscribeResult {
                    accepted: true,
                    ..Default::default()
                },
            );
            for tick in 1..=FRAMES {
                let state = proto::RobotState {
                    t: tick as f64 / 10.0,
                    movement: proto::MoveState {
                        requested: [0.0; 3],
                        applied: [0.0; 3],
                        limited_by: Vec::new(),
                    },
                    head: [0.0; 4],
                    policy: "held".into(),
                    safety: proto::SafetyState {
                        fallen: false,
                        limp: false,
                        gravity: [0.0, 0.0, -1.0],
                        gain: None,
                        picked_up: false,
                    },
                    control_loop: proto::LoopState {
                        hz: 50.0,
                        missed: 0,
                    },
                    joints: vec![0.0; 15],
                    targets: vec![0.0; 15],
                    velocities: Vec::new(),
                    currents_ma: Vec::new(),
                    odom: proto::OdomState::default(),
                    theremin: None,
                    chorale: None,
                    t_ns: tick as u64,
                    imu: None,
                    frames: None,
                    skeleton: Vec::new(),
                };
                writeln!(
                    stream,
                    "{}",
                    serde_json::to_string(&proto::Request::notify_state(&state)).unwrap()
                )
                .unwrap();
            }
            answer(
                &mut reader,
                &mut stream,
                proto::method::ROBOT_CALIBRATION_INFO,
                &proto::CalibrationInfo {
                    zero_ticks: [2048.0; 15],
                    homing_offset_ticks: [0; 15],
                    single_turn_compatible: [true; 15],
                    policy_enabled: false,
                    homed: false,
                    body_imu_mount: None,
                    body_imu_mount_calibrated: false,
                },
            );
        });
        capture_zero(&socket, &output, true).unwrap_or_else(|e| panic!("{}", e.message));
        server.join().unwrap();
        let saved: serde_json::Value =
            serde_json::from_slice(&std::fs::read(&output).unwrap()).unwrap();
        assert_eq!(saved["joints"].as_array().unwrap().len(), 15);
        assert!(
            (saved["joints"][9]["zero_tick"].as_f64().unwrap() - 2104.888888888889).abs() < 1e-9
        );
        assert_eq!(
            std::fs::metadata(&output).unwrap().permissions().mode() & 0o777,
            0o600
        );
        assert_eq!(
            capture_zero(&socket, &output, true).unwrap_err().code,
            exit::REFUSED
        );
    }
}
