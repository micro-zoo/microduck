//! Exercise the configuration path against the daemon binary, without opening hardware.
use std::path::PathBuf;
use std::process::{Child, Command, Stdio};
use std::time::{Duration, Instant};
use updater::robot::{Health, RobotClient, SocketRobotClient};

struct Daemon {
    child: Child,
    socket: PathBuf,
    log: PathBuf,
    _dir: tempfile::TempDir,
}
impl Daemon {
    fn start(calibration: Option<&str>) -> Self {
        let dir = tempfile::tempdir().unwrap();
        let socket = dir.path().join("robotd.sock");
        let json = dir.path().join("zero.json");
        if let Some(contents) = calibration {
            std::fs::write(&json, contents).unwrap();
        }
        let params = dir.path().join("robotd.toml");
        std::fs::write(
            &params,
            format!(
                "[bus]\ncalibration = {:?}\n[audio]\nenabled = false\n",
                json.to_str().unwrap()
            ),
        )
        .unwrap();
        let log = dir.path().join("robotd.log");
        let child = Command::new(env!("CARGO_BIN_EXE_robotd"))
            .args(["--fake", "--no-policy", "--params"])
            .arg(params)
            .arg("--socket")
            .arg(&socket)
            .env("RUST_LOG", "info")
            .stdout(Stdio::null())
            .stderr(std::fs::File::create(&log).unwrap())
            .spawn()
            .unwrap();
        Self {
            child,
            socket,
            log,
            _dir: dir,
        }
    }
}
impl Drop for Daemon {
    fn drop(&mut self) {
        let _ = self.child.kill();
        let _ = self.child.wait();
    }
}

#[tokio::test]
async fn a_configured_zero_file_is_loaded_before_serving_fake_io() {
    let mut daemon = Daemon::start(Some(
        r#"{"joints":[{"name":"head_yaw","id":32,"zero_tick":2176.0}]}"#,
    ));
    let client = SocketRobotClient::new(daemon.socket.clone());
    let deadline = Instant::now() + Duration::from_secs(5);
    loop {
        assert!(
            daemon.child.try_wait().unwrap().is_none(),
            "{}",
            std::fs::read_to_string(&daemon.log).unwrap()
        );
        if matches!(
            client.health(Duration::from_millis(200)).await,
            Health::Healthy
        ) {
            break;
        }
        assert!(Instant::now() < deadline, "daemon never became healthy");
        tokio::time::sleep(Duration::from_millis(25)).await;
    }
    let log = std::fs::read_to_string(&daemon.log).unwrap();
    assert!(log.contains("loaded joint zeroes"), "{log}");
    assert!(log.contains("head_yaw"), "{log}");
}

#[tokio::test]
async fn a_missing_invalid_or_mismatched_file_prevents_control_startup() {
    for document in [
        None,
        Some("{"),
        Some(r#"{"joints":[{"name":"head_yaw","id":33,"zero_tick":2048}]}"#),
    ] {
        let mut daemon = Daemon::start(document);
        let deadline = Instant::now() + Duration::from_secs(5);
        let status = loop {
            if let Some(status) = daemon.child.try_wait().unwrap() {
                break status;
            }
            assert!(
                Instant::now() < deadline,
                "daemon accepted invalid calibration"
            );
            tokio::time::sleep(Duration::from_millis(25)).await;
        };
        assert_eq!(status.code(), Some(1));
        let log = std::fs::read_to_string(&daemon.log).unwrap();
        assert!(
            log.contains("bad joint calibration; bus not opened"),
            "{log}"
        );
        assert!(
            !log.contains("--fake: no bus, no robot"),
            "control thread must not start"
        );
        assert!(!daemon.socket.exists());
    }
}
