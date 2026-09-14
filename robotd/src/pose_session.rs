//! Local command pipe for a supervised browser pose session. No network parsing here.
use duck_control::bus::{PoseCommand, PoseSession, SupportedPose};
use duck_control::io::{IoError, Result};
use std::collections::VecDeque;
use std::time::{Duration, Instant};

pub struct PipeSession {
    input: i32,
    progress: i32,
    pending: VecDeque<u8>,
    closed: bool,
}
fn failure(message: impl Into<String>) -> IoError {
    IoError::Bus(message.into())
}

impl PipeSession {
    pub fn open(progress: i32) -> Result<Self> {
        let input = std::env::var("DUCK_POSE_CONTROL_FD")
            .ok()
            .and_then(|s| s.parse::<i32>().ok())
            .ok_or_else(|| failure("interactive pose control requires its local command pipe"))?;
        let mut status = std::mem::MaybeUninit::<libc::stat>::uninit();
        if unsafe { libc::fstat(input, status.as_mut_ptr()) } != 0
            || unsafe { status.assume_init() }.st_mode & libc::S_IFMT != libc::S_IFIFO
        {
            return Err(failure("pose control descriptor is not a pipe"));
        }
        unsafe {
            let flags = libc::fcntl(input, libc::F_GETFL);
            if flags < 0 || libc::fcntl(input, libc::F_SETFL, flags | libc::O_NONBLOCK) < 0 {
                return Err(failure("cannot make pose command pipe nonblocking"));
            }
            libc::signal(libc::SIGPIPE, libc::SIG_IGN);
        }
        let mut this = Self {
            input,
            progress,
            pending: VecDeque::new(),
            closed: false,
        };
        let deadline = Instant::now() + Duration::from_secs(5);
        loop {
            this.receive()?;
            match this.pending.pop_front() {
                Some(b'G') => return Ok(this),
                Some(_) => return Err(failure("pose session cancelled before start")),
                None if this.closed || Instant::now() >= deadline => {
                    return Err(failure("pose supervisor did not arm the start barrier"));
                }
                None => std::thread::sleep(Duration::from_millis(5)),
            }
        }
    }
    fn receive(&mut self) -> Result<()> {
        let mut b = [0u8; 64];
        let n = unsafe { libc::read(self.input, b.as_mut_ptr().cast(), b.len()) };
        if n > 0 {
            self.pending.extend(&b[..n as usize]);
        } else if n == 0 {
            self.closed = true;
        } else if std::io::Error::last_os_error().kind() != std::io::ErrorKind::WouldBlock {
            return Err(failure("pose command pipe read failed"));
        }
        if self.pending.len() > 64 {
            return Err(failure("too many pending pose commands"));
        }
        Ok(())
    }
    fn send(&mut self, byte: u8) -> Result<()> {
        if unsafe { libc::write(self.progress, (&byte as *const u8).cast(), 1) } != 1 {
            return Err(failure("independent pose guardian is unavailable"));
        }
        Ok(())
    }
}
impl PoseSession for PipeSession {
    fn poll(&mut self) -> Result<PoseCommand> {
        self.receive()?;
        if self.closed || self.pending.contains(&b'S') {
            return Ok(PoseCommand::Relax);
        }
        match self.pending.pop_front() {
            Some(b'H') => Ok(PoseCommand::Move(SupportedPose::Home)),
            Some(b'Z') => Ok(PoseCommand::Move(SupportedPose::Zero)),
            None => Ok(PoseCommand::Continue),
            _ => Err(failure("invalid local pose command")),
        }
    }
    fn arm(&mut self) -> Result<()> {
        self.send(b'T')
    }
    fn progress(&mut self) -> Result<()> {
        self.send(b'P')
    }
    fn off(&mut self) -> Result<()> {
        self.send(b'D')
    }
}
