//! Register accesses over the serial link.
//!
//! The CLRC663's UART protocol answers exactly one byte per operation, in order:
//!  - write `[reg & 0x7F, value]` → the address byte, echoed
//!  - read  `[reg | 0x80]`        → the register's value
//!
//! The echo is a frame check, not noise: if it does not match the address sent, the stream is out
//! of step and [`Error::Desync`] says so. The link being full-duplex, a batch of operations goes out
//! in one write and the answers are read after — measured at 0.088 ms an access against 0.403 ms
//! one at a time, with no loss up to 512 operations a batch.

use std::io;
use std::time::{Duration, Instant};

use crate::{Error, Result};

pub const READ_FLAG: u8 = 0x80;
/// Largest batch measured without loss. The one place that knows the limit.
pub const BATCH_MAX: usize = 512;

/// The byte pipe underneath: a serial port on the board, a script in the tests.
pub trait Link {
    fn write_all(&mut self, bytes: &[u8]) -> io::Result<()>;
    /// Up to `buf.len()` bytes, waiting no later than `deadline` for the first. `Ok(0)` is a
    /// timeout.
    fn read_until(&mut self, buf: &mut [u8], deadline: Instant) -> io::Result<usize>;
    /// Discard whatever has arrived and not been read.
    fn drain(&mut self) -> io::Result<()>;
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Op {
    Read(u8),
    Write(u8, u8),
}

pub struct Transport<L> {
    link: L,
    timeout: Duration,
}

impl<L: Link> Transport<L> {
    pub fn new(link: L) -> Self {
        Self {
            link,
            timeout: Duration::from_millis(300),
        }
    }

    /// Send without checking anything. Only for the reset sequence, where the stream is by
    /// definition not in step.
    pub fn raw_write(&mut self, bytes: &[u8]) -> Result<()> {
        Ok(self.link.write_all(bytes)?)
    }

    pub fn drain(&mut self) -> Result<()> {
        Ok(self.link.drain()?)
    }

    /// Send every operation, then read exactly one answer each. Reads come back in order; writes
    /// contribute nothing.
    pub fn execute(&mut self, ops: &[Op]) -> Result<Vec<u8>> {
        let mut values = Vec::new();
        for batch in ops.chunks(BATCH_MAX) {
            self.batch(batch, &mut values)?;
        }
        Ok(values)
    }

    fn batch(&mut self, ops: &[Op], values: &mut Vec<u8>) -> Result<()> {
        let mut out = Vec::with_capacity(ops.len() * 2);
        for op in ops {
            match *op {
                Op::Read(reg) => out.push(reg | READ_FLAG),
                Op::Write(reg, value) => out.extend([reg & 0x7F, value]),
            }
        }

        self.link.drain()?;
        self.link.write_all(&out)?;

        let mut answers = vec![0u8; ops.len()];
        let mut got = 0;
        let deadline = Instant::now() + self.timeout + Duration::from_millis(ops.len() as u64);
        while got < answers.len() && Instant::now() < deadline {
            got += self.link.read_until(&mut answers[got..], deadline)?;
        }
        if got != answers.len() {
            return Err(Error::Desync(format!(
                "expected {} answer bytes, got {got}",
                answers.len()
            )));
        }

        for (op, answer) in ops.iter().zip(answers) {
            match *op {
                Op::Read(_) => values.push(answer),
                Op::Write(reg, _) if answer != reg & 0x7F => {
                    return Err(Error::Desync(format!(
                        "writing register 0x{reg:02X}: echo 0x{answer:02X}, expected 0x{:02X}",
                        reg & 0x7F
                    )));
                }
                Op::Write(..) => {}
            }
        }
        Ok(())
    }

    pub fn read_reg(&mut self, reg: u8) -> Result<u8> {
        Ok(self.execute(&[Op::Read(reg)])?[0])
    }

    pub fn write_reg(&mut self, reg: u8, value: u8) -> Result<()> {
        self.execute(&[Op::Write(reg, value)]).map(drop)
    }

    pub fn read_regs(&mut self, regs: &[u8]) -> Result<Vec<u8>> {
        let ops: Vec<Op> = regs.iter().map(|&r| Op::Read(r)).collect();
        self.execute(&ops)
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    use std::collections::VecDeque;

    /// A link that answers from a script and records what was sent.
    #[derive(Default)]
    pub(crate) struct Scripted {
        pub sent: Vec<u8>,
        pub answers: VecDeque<u8>,
    }

    impl Link for Scripted {
        fn write_all(&mut self, bytes: &[u8]) -> io::Result<()> {
            self.sent.extend_from_slice(bytes);
            Ok(())
        }
        fn read_until(&mut self, buf: &mut [u8], _deadline: Instant) -> io::Result<usize> {
            let n = buf.len().min(self.answers.len());
            for slot in &mut buf[..n] {
                *slot = self.answers.pop_front().unwrap();
            }
            Ok(n)
        }
        fn drain(&mut self) -> io::Result<()> {
            Ok(())
        }
    }

    fn transport(answers: &[u8]) -> Transport<Scripted> {
        let mut t = Transport::new(Scripted {
            sent: vec![],
            answers: answers.iter().copied().collect(),
        });
        t.timeout = Duration::ZERO;
        t
    }

    #[test]
    fn a_batch_is_one_write_with_reads_flagged_and_writes_echoed() {
        let mut t = transport(&[0x02, 0x1A]);
        let values = t.execute(&[Op::Write(0x02, 0x10), Op::Read(0x7F)]).unwrap();
        assert_eq!(values, vec![0x1A]);
        assert_eq!(t.link.sent, vec![0x02, 0x10, 0xFF]);
    }

    #[test]
    fn a_wrong_echo_is_a_desync_not_a_value() {
        let mut t = transport(&[0x05]);
        assert!(matches!(
            t.execute(&[Op::Write(0x02, 0x10)]),
            Err(Error::Desync(_))
        ));
    }

    #[test]
    fn missing_answers_are_a_desync() {
        let mut t = transport(&[0x1A]);
        assert!(matches!(t.read_regs(&[0x7F, 0x7F]), Err(Error::Desync(_))));
    }
}
