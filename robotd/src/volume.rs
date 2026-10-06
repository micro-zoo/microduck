//! Speaker loudness: `[audio] volume`, applied to the codec's PCM control.
//!
//! The setting lives in the config file and nowhere else — no second state file, no RPC to keep in
//! step with it. `robotctl volume 70` writes the file; `robotd` sees the mtime move within a
//! second and re-sets the mixer, the way `padd` takes back its own sections. A reboot
//! needs nothing more, because `aic3104-init.service` runs `Before=robotd` and leaves the card at
//! full, and `robotd` then puts the configured level over it. One layer brings the hardware up,
//! the next holds the preference.
//!
//! **Why not a restart, like the rest of `[audio]`.** Everything else in that section is read once.
//! A volume change that restarted `robotd` would stop driving the motors for being quieter, and a
//! standing robot falls. So this key is classified `Live` in `robotctl configure`.
//!
//! **`amixer`, off the control loop.** The mixer is reached the way playback already is, by running
//! the ALSA tool (`sound.rs` spawns `aplay`), but never from the 50 Hz thread: changes go down a
//! channel to one worker that runs them in order, so two quick changes cannot land reversed and a
//! slow card never costs a tick.
//!
//! **Percent on the perceptual scale** (`amixer -M`): a raw-linear 50 % of a dB-scaled control is
//! barely quieter than full, and the first few percent above zero would be the whole range.

use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use std::sync::mpsc;
use std::time::SystemTime;

use crate::params::Params;

/// The codec's playback control, by the simple-control name `amixer sset` takes. The boot script
/// leaves the output stages (`Line DAC`, `Line`) at their maximum; this is the user-facing knob.
const CONTROL: &str = "PCM";

/// Watches the config file for `audio.volume` and keeps the mixer at it.
pub struct Volume {
    path: PathBuf,
    /// The file's mtime when it was last read. A change is the only thing that triggers a re-read.
    stamp: Option<SystemTime>,
    applied: u8,
    set: Box<dyn FnMut(u8) + Send>,
}

impl Volume {
    /// Start at `initial`, setting the mixer to it now so the greet is already at the right level.
    ///
    /// `None` when `device` names no card (`default`, `sysdefault`): there is nothing to address,
    /// and guessing one would turn somebody else's volume.
    pub fn new(path: &Path, device: &str, initial: u8) -> Option<Self> {
        let Some(card) = card_of(device) else {
            tracing::debug!(
                device,
                "audio.volume: the device names no card; the mixer is left alone"
            );
            return None;
        };
        Some(Self::with_setter(path, initial, worker(card)))
    }

    fn with_setter(path: &Path, initial: u8, mut set: Box<dyn FnMut(u8) + Send>) -> Self {
        set(initial);
        Self {
            path: path.to_owned(),
            stamp: modified(path),
            applied: initial,
            set,
        }
    }

    /// Called once a second. A `stat` when nothing changed; a parse and possibly a mixer write when
    /// something did.
    pub fn poll(&mut self) {
        let stamp = modified(&self.path);
        if stamp == self.stamp {
            return;
        }
        // Whatever happens next, this version of the file has been looked at. A half-written file
        // is picked up again when its mtime moves, not retried — and logged — every second.
        self.stamp = stamp;
        match Params::load(&self.path, false) {
            Ok(params) => {
                let volume = params.audio.volume;
                if volume != self.applied {
                    tracing::info!(volume, "speaker volume set");
                    (self.set)(volume);
                    self.applied = volume;
                }
            }
            Err(e) => {
                tracing::warn!(error = %e, "config unreadable; the speaker volume is left as it was")
            }
        }
    }
}

fn modified(path: &Path) -> Option<SystemTime> {
    std::fs::metadata(path).and_then(|m| m.modified()).ok()
}

/// The card an ALSA device string names: `plughw:aic3104`, `hw:aic3104,0` and
/// `plughw:CARD=aic3104,DEV=0` all say `aic3104`.
fn card_of(device: &str) -> Option<String> {
    let (_, rest) = device.split_once(':')?;
    let first = rest.split(',').next()?;
    let card = first.strip_prefix("CARD=").unwrap_or(first);
    (!card.is_empty()).then(|| card.to_owned())
}

/// The `amixer` arguments for one change.
fn mixer_args(card: &str, percent: u8) -> Vec<String> {
    [
        "-M",
        "-q",
        "-c",
        card,
        "sset",
        CONTROL,
        &format!("{percent}%"),
    ]
    .map(str::to_owned)
    .to_vec()
}

/// One worker thread running the changes in the order they were asked for.
fn worker(card: String) -> Box<dyn FnMut(u8) + Send> {
    let (tx, rx) = mpsc::channel::<u8>();
    std::thread::spawn(move || {
        for percent in rx {
            let status = Command::new("amixer")
                .args(mixer_args(&card, percent))
                .stdin(Stdio::null())
                .stdout(Stdio::null())
                .stderr(Stdio::null())
                .status();
            match status {
                Ok(s) if s.success() => {}
                // Debug, like a failed `aplay`: a board without the codec walks identically.
                Ok(s) => tracing::debug!(card, percent, %s, "amixer refused the volume"),
                Err(e) => tracing::debug!(card, percent, error = %e, "amixer could not run"),
            }
        }
    });
    Box::new(move |percent| {
        let _ = tx.send(percent);
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::{Arc, Mutex};

    /// A setter that writes down what it was asked for, and the list it writes to.
    type Recorder = (Box<dyn FnMut(u8) + Send>, Arc<Mutex<Vec<u8>>>);

    fn recorder() -> Recorder {
        let calls = Arc::new(Mutex::new(Vec::new()));
        let seen = Arc::clone(&calls);
        (Box::new(move |v| seen.lock().unwrap().push(v)), calls)
    }

    /// Writes the file and gives it an mtime later than any earlier write's: two writes in one
    /// test must not share a timestamp, which a real edit a second apart never does.
    fn write(path: &Path, body: &str) {
        use std::sync::atomic::{AtomicU64, Ordering};
        static TICK: AtomicU64 = AtomicU64::new(0);
        std::fs::write(path, body).unwrap();
        let when = SystemTime::UNIX_EPOCH
            + std::time::Duration::from_secs(1_800_000_000 + TICK.fetch_add(1, Ordering::Relaxed));
        std::fs::File::options()
            .write(true)
            .open(path)
            .unwrap()
            .set_modified(when)
            .unwrap();
    }

    #[test]
    fn card_names_come_out_of_every_alsa_spelling() {
        assert_eq!(card_of("plughw:aic3104").as_deref(), Some("aic3104"));
        assert_eq!(card_of("hw:aic3104,0").as_deref(), Some("aic3104"));
        assert_eq!(
            card_of("plughw:CARD=aic3104,DEV=0").as_deref(),
            Some("aic3104")
        );
        assert_eq!(card_of("hw:1").as_deref(), Some("1"));
        assert_eq!(card_of("default"), None, "no colon, no card to address");
        assert_eq!(card_of("plughw:"), None);
    }

    #[test]
    fn the_mixer_is_driven_on_the_perceptual_scale() {
        assert_eq!(
            mixer_args("aic3104", 70),
            ["-M", "-q", "-c", "aic3104", "sset", "PCM", "70%"]
        );
    }

    #[test]
    fn it_sets_the_starting_level_at_once_and_only_calls_again_on_a_real_change() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("robotd.toml");
        write(&path, "[audio]\nvolume = 40\n");
        let (set, calls) = recorder();
        let mut v = Volume::with_setter(&path, 40, set);
        assert_eq!(
            *calls.lock().unwrap(),
            [40],
            "the greet must not play at the wrong level"
        );

        v.poll();
        assert_eq!(
            calls.lock().unwrap().len(),
            1,
            "an untouched file costs a stat and nothing else"
        );

        write(&path, "[audio]\nvolume = 75\n");
        v.poll();
        v.poll();
        assert_eq!(*calls.lock().unwrap(), [40, 75]);

        // A rewrite that leaves the value alone does not touch the card.
        write(&path, "[audio]\nvolume = 75\ngreet = false\n");
        v.poll();
        assert_eq!(*calls.lock().unwrap(), [40, 75]);
    }

    #[test]
    fn removing_the_key_puts_the_level_back_to_full() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("robotd.toml");
        write(&path, "[audio]\nvolume = 30\n");
        let (set, calls) = recorder();
        let mut v = Volume::with_setter(&path, 30, set);
        write(&path, "[audio]\ngreet = true\n");
        v.poll();
        assert_eq!(*calls.lock().unwrap(), [30, 100]);
    }

    #[test]
    fn a_broken_file_keeps_the_last_good_level_and_is_not_retried_every_second() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("robotd.toml");
        write(&path, "[audio]\nvolume = 55\n");
        let (set, calls) = recorder();
        let mut v = Volume::with_setter(&path, 55, set);
        write(&path, "[audio]\nvolume = loud\n");
        v.poll();
        write(&path, "[audio]\nvolume = 101\n");
        v.poll();
        assert_eq!(
            *calls.lock().unwrap(),
            [55],
            "neither a type error nor an out-of-range value reaches the card"
        );
        write(&path, "[audio]\nvolume = 20\n");
        v.poll();
        assert_eq!(
            *calls.lock().unwrap(),
            [55, 20],
            "and the next good edit still lands"
        );
    }
}
