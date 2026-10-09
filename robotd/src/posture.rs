//! Is the robot standing, sitting, or lying down — judged from where it actually is.
//!
//! Asked once each time the policy is about to take over a robot at its home pose: the ramp there
//! is open-loop, and a robot that started folded can end it standing or sat back on its seat. The
//! two want different starts — a gait handed a seated robot walks it over backwards, and a rise
//! handed a standing one throws it — so a seated verdict starts with the sitstand rise. Every other
//! verdict starts the gait: this picks how the policy starts, never whether.
//!
//! Two signals, both available on a stiff robot standing still:
//!
//! - **Trunk height above the feet**, along gravity: the feet sites from the joint angles through
//!   the kinematic model, turned into the world by the IMU's attitude, as a fraction of the model's
//!   standing height. Measured on the robot, three quite different seated poses — hips folded,
//!   knees folded, everything folded — all landed between 15 and 39 % of standing, where the joint
//!   angles alone had nothing in common. The home pose itself reads about 97 %.
//! - **Trunk tilt** from the IMU: the angle between the trunk's up and the world's. The seated
//!   poses sat 6–11° back; a robot on its back read 86°. Past [`LYING_TILT_DEG`] the height means
//!   nothing — a robot lying down has its feet level with its trunk whatever its legs do.
//!
//! Between the two height thresholds the verdict is [`Posture::Unsure`], on purpose: the robot is
//! neither clearly up nor clearly down, so it is not handed to the rise, which is only safe from
//! a real seat.

use duck_ipc_proto as proto;
use kinematics::Quat;

/// Trunk tilt beyond which the robot is lying down, degrees.
pub const LYING_TILT_DEG: f64 = 45.0;

/// Below this fraction of the standing trunk height, the robot is sitting.
pub const SEATED_BELOW: f64 = 0.5;

/// Above this fraction of the standing trunk height, the robot is standing.
pub const STANDING_ABOVE: f64 = 0.8;

/// The verdict.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Posture {
    Standing,
    Seated,
    /// On its back, front or side: neither of the two the robot can start from.
    Lying,
    /// Between sitting and standing height.
    Unsure,
}

/// The numbers the verdict was made from, for the journal line that reports it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Reading {
    pub posture: Posture,
    /// Trunk height above the feet over the model's standing trunk height.
    pub height_ratio: f64,
    /// Angle between the trunk's up and the world's, degrees.
    pub tilt_deg: f64,
}

/// Judge a pose: `positions` in wire order ([`proto::JOINT_NAMES`]), `quat` the IMU's trunk →
/// world attitude, scalar first.
pub fn classify(positions: &[f64], quat: [f64; 4]) -> Reading {
    let model = kinematics::Model::alpha();
    // The model's joint order is the MJCF's, not the wire's: gather the angles by name.
    let angles: Vec<f64> = model
        .joint_names()
        .map(|name| {
            proto::JOINT_NAMES
                .iter()
                .position(|wire| *wire == name)
                .and_then(|i| positions.get(i).copied())
                .unwrap_or(0.0)
        })
        .collect();
    let foot = |name: &str| {
        model
            .site(name)
            .map_or([0.0; 3], |site| model.site_pose(site, &angles).pos)
    };
    let (left, right) = (foot("left_foot"), foot("right_foot"));
    let feet = [
        (left[0] + right[0]) / 2.0,
        (left[1] + right[1]) / 2.0,
        (left[2] + right[2]) / 2.0,
    ];

    let [w, x, y, z] = quat;
    let attitude = Quat::new(w, x, y, z).normalized();
    let height = -attitude.rotate(feet)[2];
    let up = attitude.rotate([0.0, 0.0, 1.0]);
    let tilt_deg = up[2].clamp(-1.0, 1.0).acos().to_degrees();
    let height_ratio = height / model.trunk_height_m();

    let posture = if tilt_deg > LYING_TILT_DEG {
        Posture::Lying
    } else if height_ratio < SEATED_BELOW {
        Posture::Seated
    } else if height_ratio > STANDING_ABOVE {
        Posture::Standing
    } else {
        Posture::Unsure
    };
    Reading {
        posture,
        height_ratio,
        tilt_deg,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Recorded on an alpha robot, torque off, 2026-10-05: `robot.state`'s joints and IMU quat.
    const LYING: ([f64; 15], [f64; 4]) = (
        [
            0.0951, -0.4418, -0.4525, -0.0476, 0.4541, 0.1058, -0.7440, 0.3942, 0.0936, 0.0015,
            -0.0583, 0.1381, 0.3712, -0.3651, -0.4633,
        ],
        [0.1588, 0.6902, -0.2091, 0.6742],
    );
    const SEATED_HIPS: ([f64; 15], [f64; 4]) = (
        [
            -0.0644, 0.5783, -1.5524, 0.2086, 0.4571, 1.0017, 1.4788, -0.0261, -0.0491, 0.0031,
            -0.2761, -0.4479, 1.0677, -0.8207, -0.6811,
        ],
        [0.7972, 0.0264, -0.0984, 0.5951],
    );
    const SEATED_KNEES: ([f64; 15], [f64; 4]) = (
        [
            -0.0092, -0.0460, -0.5722, 1.0968, -0.0966, 1.0630, 1.4772, -0.1565, 0.1212, 0.0031,
            0.0552, -0.1611, 0.7164, -0.8851, 0.3421,
        ],
        [0.7136, 0.0561, -0.0230, 0.6980],
    );
    const SEATED_FOLDED: ([f64; 15], [f64; 4]) = (
        [
            -0.0276, 0.0123, 1.6720, 1.5248, -0.0660, 1.1167, 1.4711, -0.0813, 0.0414, 0.0046,
            -0.1396, -0.0261, -1.7656, -1.6015, 0.0598,
        ],
        [0.7429, 0.0613, -0.0527, 0.6645],
    );

    /// The home pose, level, is standing — the pose the ramp aims every bring-up at.
    #[test]
    fn the_home_pose_is_standing() {
        let reading = classify(&duck_control::model::DEFAULT_POSITION, [1.0, 0.0, 0.0, 0.0]);
        assert_eq!(reading.posture, Posture::Standing, "{reading:?}");
        assert!(reading.height_ratio > 0.9, "{reading:?}");
    }

    /// The seat the ramp brings a sitting robot to, level, reads seated — or the second Start
    /// would hand a robot it had just sat down to the gait.
    #[test]
    fn the_seat_pose_is_seated() {
        let reading = classify(&crate::SEAT_POSITION, [1.0, 0.0, 0.0, 0.0]);
        assert_eq!(reading.posture, Posture::Seated, "{reading:?}");
    }

    /// **Three ways of sitting, one verdict.** The joint angles of these have little in common —
    /// hips folded, knees folded, both — and the old mean-deviation criterion would have scored
    /// them anywhere from 0.4 to 0.9 rad. The trunk's height above the feet puts all three low.
    #[test]
    fn every_recorded_seat_is_seated() {
        for (name, (joints, quat)) in [
            ("hips", SEATED_HIPS),
            ("knees", SEATED_KNEES),
            ("folded", SEATED_FOLDED),
        ] {
            let reading = classify(&joints, quat);
            assert_eq!(reading.posture, Posture::Seated, "{name}: {reading:?}");
        }
    }

    /// On its back, legs nearly straight: by the joints alone this is a standing robot, which is
    /// exactly the mistake the tilt is there to catch.
    #[test]
    fn a_robot_on_its_back_is_lying() {
        let (joints, quat) = LYING;
        let reading = classify(&joints, quat);
        assert_eq!(reading.posture, Posture::Lying, "{reading:?}");
        assert!(reading.tilt_deg > 80.0, "{reading:?}");
    }

    /// The home pose tipped well past the lying threshold is lying, whatever its legs say — on its
    /// side as much as on its back.
    #[test]
    fn a_robot_on_its_side_is_lying() {
        let half = 70.0f64.to_radians() / 2.0;
        let on_side = [half.cos(), half.sin(), 0.0, 0.0];
        let reading = classify(&duck_control::model::DEFAULT_POSITION, on_side);
        assert_eq!(reading.posture, Posture::Lying, "{reading:?}");
    }
}
