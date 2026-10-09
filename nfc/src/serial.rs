//! The reader's serial port: raw 8N1 at 115200, through termios.

use std::io;
use std::path::Path;
use std::time::Instant;

use crate::transport::Link;

pub struct Serial {
    #[cfg(target_os = "linux")]
    fd: std::os::fd::OwnedFd,
}

#[cfg(target_os = "linux")]
impl Serial {
    pub fn open(path: &Path) -> io::Result<Self> {
        use std::os::fd::{AsRawFd, FromRawFd, OwnedFd};
        use std::os::unix::ffi::OsStrExt;

        let cpath = std::ffi::CString::new(path.as_os_str().as_bytes())
            .map_err(|_| io::Error::from(io::ErrorKind::InvalidInput))?;
        // Safety: a NUL-terminated path; the descriptor is owned from here on.
        let raw = unsafe {
            libc::open(
                cpath.as_ptr(),
                libc::O_RDWR | libc::O_NOCTTY | libc::O_CLOEXEC,
            )
        };
        if raw < 0 {
            return Err(io::Error::last_os_error());
        }
        let fd = unsafe { OwnedFd::from_raw_fd(raw) };
        let raw = fd.as_raw_fd();

        // Safety: termios is plain data, filled by tcgetattr before any field is read.
        unsafe {
            let mut tio: libc::termios = std::mem::zeroed();
            if libc::tcgetattr(raw, &mut tio) != 0 {
                return Err(io::Error::last_os_error());
            }
            libc::cfmakeraw(&mut tio);
            tio.c_cflag |= libc::CLOCAL | libc::CREAD;
            tio.c_cflag &= !libc::CRTSCTS;
            // Reads are bounded by poll() below, never by the line discipline.
            tio.c_cc[libc::VMIN] = 0;
            tio.c_cc[libc::VTIME] = 0;
            libc::cfsetspeed(&mut tio, libc::B115200);
            if libc::tcsetattr(raw, libc::TCSANOW, &tio) != 0 {
                return Err(io::Error::last_os_error());
            }
            // DTR and RTS down, as `ntag663` does: measured to make no difference on this board,
            // but the pads are wired, and asserting them is only the kernel's default.
            let lines: libc::c_int = libc::TIOCM_DTR | libc::TIOCM_RTS;
            libc::ioctl(raw, libc::TIOCMBIC, &lines);
        }
        Ok(Self { fd })
    }

    fn raw(&self) -> std::os::fd::RawFd {
        use std::os::fd::AsRawFd;
        self.fd.as_raw_fd()
    }
}

#[cfg(target_os = "linux")]
impl Link for Serial {
    fn write_all(&mut self, mut bytes: &[u8]) -> io::Result<()> {
        while !bytes.is_empty() {
            // Safety: writes from a live slice to an owned descriptor.
            let n = unsafe { libc::write(self.raw(), bytes.as_ptr().cast(), bytes.len()) };
            if n < 0 {
                let e = io::Error::last_os_error();
                if e.kind() == io::ErrorKind::Interrupted {
                    continue;
                }
                return Err(e);
            }
            bytes = &bytes[n as usize..];
        }
        // Safety: as above.
        if unsafe { libc::tcdrain(self.raw()) } != 0 {
            return Err(io::Error::last_os_error());
        }
        Ok(())
    }

    fn read_until(&mut self, buf: &mut [u8], deadline: Instant) -> io::Result<usize> {
        let left = deadline.saturating_duration_since(Instant::now());
        let mut pfd = libc::pollfd {
            fd: self.raw(),
            events: libc::POLLIN,
            revents: 0,
        };
        // Safety: one pollfd, owned here.
        let ready = unsafe { libc::poll(&mut pfd, 1, left.as_millis().max(1) as libc::c_int) };
        if ready < 0 {
            let e = io::Error::last_os_error();
            return if e.kind() == io::ErrorKind::Interrupted {
                Ok(0)
            } else {
                Err(e)
            };
        }
        if ready == 0 {
            return Ok(0);
        }
        if pfd.revents & (libc::POLLHUP | libc::POLLERR) != 0 {
            // Unplugged. Said as what it is, so the daemon reopens rather than resets.
            return Err(io::Error::from(io::ErrorKind::BrokenPipe));
        }
        // Safety: reads into a live, correctly sized buffer.
        let n = unsafe { libc::read(self.raw(), buf.as_mut_ptr().cast(), buf.len()) };
        if n < 0 {
            return Err(io::Error::last_os_error());
        }
        Ok(n as usize)
    }

    fn drain(&mut self) -> io::Result<()> {
        // Safety: an owned descriptor.
        if unsafe { libc::tcflush(self.raw(), libc::TCIFLUSH) } != 0 {
            return Err(io::Error::last_os_error());
        }
        Ok(())
    }
}

/// Off Linux there is no reader to open: the crate builds, so the suite runs on a Mac, and the
/// daemon says why it has nothing to watch.
#[cfg(not(target_os = "linux"))]
impl Serial {
    pub fn open(_path: &Path) -> io::Result<Self> {
        Err(io::Error::new(
            io::ErrorKind::Unsupported,
            "the NFC reader is only driven on Linux",
        ))
    }
}

#[cfg(not(target_os = "linux"))]
impl Link for Serial {
    fn write_all(&mut self, _bytes: &[u8]) -> io::Result<()> {
        unreachable!("a Serial cannot be opened off Linux")
    }
    fn read_until(&mut self, _buf: &mut [u8], _deadline: Instant) -> io::Result<usize> {
        unreachable!("a Serial cannot be opened off Linux")
    }
    fn drain(&mut self) -> io::Result<()> {
        unreachable!("a Serial cannot be opened off Linux")
    }
}
