//! The CLRC663 itself: reset, FIFO, the RF field, and transceive.

use std::thread::sleep;
use std::time::Duration;

pub use crate::reader::Exchange;
use crate::reader::Reader;
use crate::registers as R;
use crate::transport::{Link, Op, Transport};
use crate::{Error, Result};

pub struct Clrc663<L> {
    t: Transport<L>,
}

impl<L: Link> Clrc663<L> {
    pub fn new(link: L) -> Self {
        Self {
            t: Transport::new(link),
        }
    }

    /// The opening sequence: reset, then the 512-byte FIFO a full NTAG215 read needs.
    pub fn begin(&mut self) -> Result<()> {
        self.soft_reset()?;
        self.set_fifo_512()
    }

    /// Reset the chip and check it answers as a CLRC66303.
    ///
    /// **Two attempts are needed, and three are made.** Measured, five tries a configuration: one
    /// attempt succeeds 0/5 whatever the delay (up to 300 ms) and whether the buffer is drained;
    /// two in a row succeed 5/5, always on exactly the second; closing and reopening the port
    /// changes nothing. So it is framing, not timing: out of step, the chip is waiting for a data
    /// byte, the first `[Command, SoftReset]` pair is swallowed as the tail of a frame and brings
    /// the stream back in step, and the second actually resets. Sent raw, because a desynchronised
    /// stream would fail the echo check.
    pub fn soft_reset(&mut self) -> Result<()> {
        let mut last = None;
        for _ in 0..3 {
            self.t.raw_write(&[R::REG_COMMAND, R::CMD_SOFTRESET])?;
            sleep(Duration::from_millis(50));
            self.t.drain()?;
            match self.t.read_reg(R::REG_VERSION) {
                Ok(R::VERSION_CLRC663) => return Ok(()),
                // Answered, and not what was expected: saying "no answer" would send someone
                // looking for a wiring fault on a sound board. A CLRC66301 or 02 answers 0x18.
                Ok(version) => return Err(Error::WrongChip(version)),
                Err(Error::Desync(why)) => last = Some(why),
                Err(other) => return Err(other),
            }
        }
        Err(Error::Desync(format!(
            "no answer after three soft resets ({})",
            last.unwrap_or_default()
        )))
    }

    /// FIFOControl resets to 0xA0 — 255 bytes, and a NTAG215's 504 do not fit. The datasheet
    /// wants the size changed on an empty FIFO, hence the flush first.
    fn set_fifo_512(&mut self) -> Result<()> {
        let current = self.t.execute(&[
            Op::Write(R::REG_FIFOCONTROL, R::FIFOCONTROL_FLUSH),
            Op::Read(R::REG_FIFOCONTROL),
        ])?[0];
        if current & R::FIFOCONTROL_SIZE_255 != 0 {
            self.t
                .write_reg(R::REG_FIFOCONTROL, current & !R::FIFOCONTROL_SIZE_255)?;
        }
        Ok(())
    }

    fn fifo_length(&mut self) -> Result<usize> {
        let regs = self.t.read_regs(&[R::REG_FIFOCONTROL, R::REG_FIFOLENGTH])?;
        let high = (regs[0] & R::FIFOCONTROL_LENGTH_EXT_MASK) as usize;
        Ok((high << 8) | regs[1] as usize)
    }

    fn read_fifo(&mut self, n: usize) -> Result<Vec<u8>> {
        self.t.read_regs(&vec![R::REG_FIFODATA; n])
    }

    pub fn field_on(&mut self) -> Result<()> {
        // Idle, flush, the two argument bytes and the command are independent writes: one round
        // trip.
        self.t.execute(&[
            Op::Write(R::REG_COMMAND, R::CMD_IDLE),
            Op::Write(R::REG_FIFOCONTROL, R::FIFOCONTROL_FLUSH),
            Op::Write(R::REG_FIFODATA, R::PROTO_ISO14443A_106),
            Op::Write(R::REG_FIFODATA, R::PROTO_ISO14443A_106),
            Op::Write(R::REG_COMMAND, R::CMD_LOADPROTOCOL),
        ])?;
        sleep(Duration::from_millis(10));
        let ops: Vec<Op> = R::RECOM_14443A_ID1_106
            .iter()
            .enumerate()
            .map(|(i, &value)| Op::Write(R::REG_DRVMODE + i as u8, value))
            .collect();
        self.t.execute(&ops).map(drop)
    }

    pub fn field_off(&mut self) -> Result<()> {
        self.t
            .execute(&[
                Op::Write(R::REG_COMMAND, R::CMD_IDLE),
                Op::Write(R::REG_DRVMODE, 0x00),
            ])
            .map(drop)
    }

    /// Field off, 10 ms, field on: every tag back to IDLE whatever it was doing.
    ///
    /// After a SELECT an ISO-14443-3 tag is ACTIVE and ignores REQA, so without this the second
    /// read of a session fails — alternately, the worst kind of failure to diagnose. Measured: REQA
    /// alone 3/6, WUPA alone 2/6, WUPA + HLTA 6/6, field cut 10 ms + WUPA 20/20. Preferred to HLTA
    /// because it also recovers after an interrupted exchange, where HLTA leaves a tag in HALT.
    pub fn reset_field(&mut self) -> Result<()> {
        self.field_off()?;
        sleep(Duration::from_millis(10));
        self.field_on()
    }

    /// Send `data` to the tag and return its answer.
    ///
    /// No IRQ polling loop: a USB round trip costs 0.4 ms, the radio exchange tens of microseconds,
    /// so the exchange has finished by the time the first read returns. One retry after the
    /// timeout covers the edge.
    ///
    /// `tx_bits` is how many bits of the last byte go out — 7 for REQA/WUPA, 8 otherwise. `crc`
    /// drives both directions unless `rx_crc` splits them, which only a Type 2 WRITE needs.
    pub fn transceive(&mut self, data: &[u8], exchange: Exchange) -> Result<Vec<u8>> {
        let num = if exchange.tx_bits == 8 {
            0
        } else {
            exchange.tx_bits
        };
        let crc = |on: bool| R::RECOM_14443A_CRC | u8::from(on);
        let ticks = ((exchange.timeout_ms as f64 * 1000.0 / R::TICK_US) as u32).clamp(1, 0xFFFF);
        let (hi, lo) = ((ticks >> 8) as u8, ticks as u8);

        // Idle and flush folded into the configuration batch: separate, they would cost two more
        // USB round trips on the critical path.
        let mut ops = vec![
            Op::Write(R::REG_COMMAND, R::CMD_IDLE),
            Op::Write(R::REG_FIFOCONTROL, R::FIFOCONTROL_FLUSH),
            Op::Write(R::REG_TXDATANUM, num | R::TXDATANUM_DATAEN),
            Op::Write(R::REG_TXCRCPRESET, crc(exchange.crc)),
            Op::Write(
                R::REG_RXCRCCON,
                crc(exchange.rx_crc.unwrap_or(exchange.crc)),
            ),
            // RxAlign 0: every exchange here starts on a byte boundary.
            Op::Write(R::REG_RXBITCTRL, 0),
            Op::Write(R::REG_IRQ0, R::IRQ_CLEAR_ALL),
            Op::Write(R::REG_IRQ1, R::IRQ_CLEAR_ALL),
            Op::Write(R::REG_IRQ0EN, R::IRQ0EN_RX | R::IRQ0EN_ERR),
            Op::Write(R::REG_IRQ1EN, R::IRQ1EN_TIMER0),
            Op::Write(
                R::REG_T0CONTROL,
                R::TCONTROL_CLK_211KHZ | R::TCONTROL_START_TX_END,
            ),
            Op::Write(R::REG_T0RELOADHI, hi),
            Op::Write(R::REG_T0RELOADLO, lo),
            Op::Write(R::REG_T0COUNTERVALHI, hi),
            Op::Write(R::REG_T0COUNTERVALLO, lo),
        ];
        self.t.execute(&ops)?;
        ops.clear();
        ops.extend(data.iter().map(|&b| Op::Write(R::REG_FIFODATA, b)));
        ops.push(Op::Write(R::REG_COMMAND, R::CMD_TRANSCEIVE));
        self.t.execute(&ops)?;

        // Error and RxColl in the same batch as the IRQs: free with pipelining, and CollDet and
        // CollPos are cleared when the next command starts, so this is the only moment to read them.
        let state = [R::REG_IRQ0, R::REG_IRQ1, R::REG_ERROR, R::REG_RXCOLL];
        let mut regs = self.t.read_regs(&state)?;
        if regs[0] & (R::IRQ0_RX | R::IRQ0_ERR) == 0 && regs[1] & R::IRQ1_TIMER0 == 0 {
            sleep(Duration::from_millis(exchange.timeout_ms));
            regs = self.t.read_regs(&state)?;
        }
        let (irq0, error, rxcoll) = (regs[0], regs[2], regs[3]);
        self.t.write_reg(R::REG_COMMAND, R::CMD_IDLE)?;

        if rxcoll & R::RXCOLL_POSVALID != 0 {
            return Err(Error::Collision(format!(
                "collision at bit {}",
                rxcoll & R::RXCOLL_POS_MASK
            )));
        }
        if irq0 & R::IRQ0_ERR != 0 {
            let length = self.fifo_length()?;
            if error & R::ERROR_INVALID_DATA != 0 {
                // The CRC is not in the FIFO, so the host cannot re-check: bytes that failed it
                // are refused rather than handed on as if they were sound.
                return Err(Error::Tag(format!(
                    "invalid frame, Error = 0x{error:02X} ({length} bytes in the FIFO)"
                )));
            }
            if length == 0 {
                return Err(Error::NoTag);
            }
        }
        if irq0 & R::IRQ0_RX == 0 {
            return Err(Error::NoTag);
        }
        let length = self.fifo_length()?;
        self.read_fifo(length)
    }
}

impl<L: Link> Reader for Clrc663<L> {
    fn reset_field(&mut self) -> Result<()> {
        Clrc663::reset_field(self)
    }

    fn transceive(&mut self, data: &[u8], exchange: Exchange) -> Result<Vec<u8>> {
        Clrc663::transceive(self, data, exchange)
    }
}
