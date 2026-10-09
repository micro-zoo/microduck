//! Bench: one ISO14443A request done by hand, with every IRQ bit and its timing printed.
//! `st25r100_raw [/dev/spidev1.0] [1|2]`
fn main() -> nfc::Result<()> {
    #[cfg(target_os = "linux")]
    {
        use nfc::reader::Reader;
        use nfc::spi::{Bus, Spidev};
        use nfc::st25r100::{Antenna, St25r100};
        use std::time::{Duration, Instant};
        let mut args = std::env::args().skip(1);
        let path = args.next().unwrap_or_else(|| "/dev/spidev1.0".to_owned());
        let antenna = if args.next().as_deref() == Some("2") {
            Antenna::Two
        } else {
            Antenna::One
        };
        let mut chip = St25r100::new(Spidev::open(std::path::Path::new(&path))?, antenna);
        chip.begin()?;
        let mut bus = Spidev::open(std::path::Path::new(&path))?;
        let rd = |bus: &mut Spidev, reg: u8, n: usize| -> Vec<u8> {
            let mut v = vec![0u8; n];
            bus.transfer(&[0x80 | reg], &mut v).unwrap();
            v
        };
        let wr = |bus: &mut Spidev, bytes: &[u8]| bus.transfer(bytes, &mut []).unwrap();
        if std::env::var_os("NFCA_RX").is_some() {
            wr(&mut bus, &[0x09, 0xc3]); // correlator 1: IIR coefficients
            wr(&mut bus, &[0x0d, 0x02]); // correlator 5: decimation factor 2
            println!("correlators: {:02x?}", rd(&mut bus, 0x09, 6));
        }
        for (name, request) in [("WUPA", 0x52u8), ("REQA", 0x26u8)] {
            for attempt in 0..5 {
                chip.reset_field()?;
                wr(&mut bus, &[0x62]); // stop all
                wr(&mut bus, &[0x66]); // clear rx gain
                wr(&mut bus, &[0x13, 0x40]); // parity, no CRC, OOK
                wr(&mut bus, &[0x16, 0x38]); // no CRC back
                wr(&mut bus, &[0x1e, 0x00, 0x04, 0x24]); // NRT 1060 x 4.72 us = 5 ms
                wr(&mut bus, &[0x34, 0x00, 0x07]); // 0 bytes + 7 bits
                wr(&mut bus, &[0x5f, request]);
                let op = rd(&mut bus, 0x00, 1)[0];
                wr(&mut bus, &[0x6a]); // transmit
                let t0 = Instant::now();
                let mut log = String::new();
                while t0.elapsed() < Duration::from_millis(8) {
                    let irq = rd(&mut bus, 0x3c, 3);
                    if irq != [0, 0, 0] {
                        log += &format!(
                            " {:>5}us:{:02x}{:02x}{:02x}",
                            t0.elapsed().as_micros(),
                            irq[0],
                            irq[1],
                            irq[2]
                        );
                    }
                }
                let status = rd(&mut bus, 0x11, 1)[0];
                let fifo = rd(&mut bus, 0x36, 2);
                let n = fifo[0] as usize;
                let data = if n > 0 { rd(&mut bus, 0x5f, n) } else { vec![] };
                let coll = rd(&mut bus, 0x38, 1)[0];
                let d2 = rd(&mut bus, 0x10, 1)[0];
                println!(
                    "{name} #{attempt} op={op:02x} irq[{log} ] status={status:02x} fifo={fifo:02x?} data={data:02x?} coll={coll:02x} gain={d2:02x}"
                );
            }
        }
    }
    Ok(())
}
