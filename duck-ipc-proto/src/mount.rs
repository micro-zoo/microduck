//! The body IMU's mounting: the sensor-to-trunk rotation `robotd` decodes the board with, and how
//! to measure it.
//!
//! Here rather than beside the decoder because it is on the wire — `robot.calibrationInfo`
//! carries the mount in effect — and because the client that measures it, `robotctl calibrate
//! imu`, links this crate and nothing heavier. Quaternions are scalar-first `[w, x, y, z]`, and a
//! mount maps a sensor-frame vector into the trunk frame (x forward, y left, z up).

use std::fmt;

/// Least tilt between the two poses [`mount_from_gravity`] accepts. Below it the forward axis is
/// the difference of two nearly equal vectors, and the sensor's noise sets its direction.
pub const MIN_TILT_DEG: f64 = 15.0;

/// Why two gravity samples do not make a mount.
#[derive(Debug, Clone, PartialEq)]
pub enum MountError {
    /// A sample whose length is far from one is not gravity: the robot was moving, or the
    /// sample is not what it claims to be.
    NotADirection(f64),
    /// The second pose is too close to the first to find forward from.
    TooLittleTilt { degrees: f64 },
}

impl fmt::Display for MountError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NotADirection(n) => {
                write!(f, "a gravity sample is not a direction (|g| = {n:.3})")
            }
            Self::TooLittleTilt { degrees } => write!(
                f,
                "the second pose is tilted {degrees:.1}° from the first; tilt the nose down at least \
                 {MIN_TILT_DEG}° so the forward axis is measured rather than guessed"
            ),
        }
    }
}

impl std::error::Error for MountError {}

/// Gravity as the sensor itself measured it, from gravity in the trunk frame and the mount that
/// turned the one into the other.
pub fn sensor_gravity(mount: [f64; 4], trunk: [f64; 3]) -> [f64; 3] {
    rotate(conjugate(mount), trunk)
}

/// The sensor-to-trunk mount, from gravity measured in the sensor's own frame in two still poses:
/// trunk upright, then the same trunk pitched nose-down.
///
/// Upright gives the trunk's up axis (opposite gravity). Pitching nose-down adds a component
/// along the trunk's forward axis — a pitched trunk sees gravity as `[sin θ, 0, −cos θ]` — so the
/// part of the second sample not along the first is forward. Left completes the frame. A tilt with
/// some roll in it leans the forward axis by that roll, so the pitch should be straight forward.
pub fn mount_from_gravity(upright: [f64; 3], nose_down: [f64; 3]) -> Result<[f64; 4], MountError> {
    let down = unit(upright)?;
    let tilted = unit(nose_down)?;
    let along = dot(down, tilted);
    let degrees = along.clamp(-1.0, 1.0).acos().to_degrees();
    if degrees < MIN_TILT_DEG {
        return Err(MountError::TooLittleTilt { degrees });
    }
    let up = down.map(|c| -c);
    let forward =
        unit([0, 1, 2].map(|i| (tilted[i] - along * down[i]) / (1.0 - along * along).sqrt()))?;
    let left = cross(up, forward);
    // Rows are the trunk axes in sensor coordinates, so this matrix maps a sensor vector into
    // the trunk frame — which is what a mount is.
    Ok(quat_from_matrix([forward, left, up]))
}

fn unit(v: [f64; 3]) -> Result<[f64; 3], MountError> {
    let n = dot(v, v).sqrt();
    if !(0.5..=1.5).contains(&n) {
        return Err(MountError::NotADirection(n));
    }
    Ok(v.map(|c| c / n))
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn conjugate(q: [f64; 4]) -> [f64; 4] {
    [q[0], -q[1], -q[2], -q[3]]
}

/// `q · v · q⁻¹`
fn rotate(q: [f64; 4], v: [f64; 3]) -> [f64; 3] {
    let u = [q[1], q[2], q[3]];
    let t = cross(u, v).map(|c| 2.0 * c);
    let c = cross(u, t);
    [0, 1, 2].map(|i| v[i] + q[0] * t[i] + c[i])
}

/// Scalar-first unit quaternion of a rotation matrix (Shepperd's method), with `w >= 0`.
fn quat_from_matrix(r: [[f64; 3]; 3]) -> [f64; 4] {
    let trace = r[0][0] + r[1][1] + r[2][2];
    let q = if trace > 0.0 {
        let s = (trace + 1.0).sqrt() * 2.0;
        [
            0.25 * s,
            (r[2][1] - r[1][2]) / s,
            (r[0][2] - r[2][0]) / s,
            (r[1][0] - r[0][1]) / s,
        ]
    } else if r[0][0] > r[1][1] && r[0][0] > r[2][2] {
        let s = (1.0 + r[0][0] - r[1][1] - r[2][2]).sqrt() * 2.0;
        [
            (r[2][1] - r[1][2]) / s,
            0.25 * s,
            (r[0][1] + r[1][0]) / s,
            (r[0][2] + r[2][0]) / s,
        ]
    } else if r[1][1] > r[2][2] {
        let s = (1.0 + r[1][1] - r[0][0] - r[2][2]).sqrt() * 2.0;
        [
            (r[0][2] - r[2][0]) / s,
            (r[0][1] + r[1][0]) / s,
            0.25 * s,
            (r[1][2] + r[2][1]) / s,
        ]
    } else {
        let s = (1.0 + r[2][2] - r[0][0] - r[1][1]).sqrt() * 2.0;
        [
            (r[1][0] - r[0][1]) / s,
            (r[0][2] + r[2][0]) / s,
            (r[1][2] + r[2][1]) / s,
            0.25 * s,
        ]
    };
    let n = q.iter().map(|v| v * v).sum::<f64>().sqrt();
    let sign = if q[0] < 0.0 { -1.0 } else { 1.0 };
    q.map(|v| sign * v / n)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The original board's mount: +90° about Y, trunk = [+raw_z, +raw_y, −raw_x].
    const DEFAULT_MOUNT: [f64; 4] = [
        std::f64::consts::FRAC_1_SQRT_2,
        0.0,
        std::f64::consts::FRAC_1_SQRT_2,
        0.0,
    ];

    /// A mount and its negation are one rotation.
    fn same_rotation(a: [f64; 4], b: [f64; 4]) -> bool {
        let d: f64 = a.iter().zip(b).map(|(x, y)| x * y).sum();
        (d.abs() - 1.0).abs() < 1e-9
    }

    /// What the sensor reads with the trunk upright, and pitched nose-down by `degrees`.
    fn poses(mount: [f64; 4], degrees: f64) -> ([f64; 3], [f64; 3]) {
        let t = degrees.to_radians();
        (
            sensor_gravity(mount, [0.0, 0.0, -1.0]),
            sensor_gravity(mount, [t.sin(), 0.0, -t.cos()]),
        )
    }

    #[test]
    fn the_default_mount_reads_the_documented_axes() {
        // trunk = [+raw_z, +raw_y, −raw_x]: the board's z is trunk forward, its x trunk down.
        assert!(
            rotate(DEFAULT_MOUNT, [0.0, 0.0, 1.0])
                .iter()
                .zip([1.0, 0.0, 0.0])
                .all(|(a, b)| (a - b).abs() < 1e-12)
        );
        assert!(
            rotate(DEFAULT_MOUNT, [1.0, 0.0, 0.0])
                .iter()
                .zip([0.0, 0.0, -1.0])
                .all(|(a, b)| (a - b).abs() < 1e-12)
        );
    }

    #[test]
    fn two_still_poses_give_back_the_mount_they_were_taken_with() {
        let h = std::f64::consts::FRAC_1_SQRT_2;
        let odd = {
            let q: [f64; 4] = [0.9, 0.2, -0.3, 0.25];
            let n = q.iter().map(|v| v * v).sum::<f64>().sqrt();
            q.map(|v| v / n)
        };
        for mount in [
            DEFAULT_MOUNT,
            [0.5, -0.5, 0.5, -0.5],
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
            [h, h, 0.0, 0.0],
            odd,
        ] {
            for degrees in [20.0, 35.0, 60.0] {
                let (upright, nose_down) = poses(mount, degrees);
                let got = mount_from_gravity(upright, nose_down).unwrap();
                assert!(
                    same_rotation(got, mount),
                    "{mount:?} at {degrees}°: got {got:?}"
                );
                let g = rotate(got, upright);
                assert!((g[2] + 1.0).abs() < 1e-9 && g[0].abs() < 1e-9 && g[1].abs() < 1e-9);
            }
        }
    }

    #[test]
    fn a_mount_needs_a_real_tilt_and_real_gravity() {
        let (upright, nose_down) = poses(DEFAULT_MOUNT, 5.0);
        assert!(matches!(
            mount_from_gravity(upright, nose_down),
            Err(MountError::TooLittleTilt { .. })
        ));
        assert!(matches!(
            mount_from_gravity([0.0, 0.0, 0.0], nose_down),
            Err(MountError::NotADirection(_))
        ));
    }
}
