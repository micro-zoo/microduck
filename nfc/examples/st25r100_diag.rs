//! Bench diagnostics: is there a field on each antenna, and does a tag change it?
//! I/Q measurement of each RFI with the field off and on (DS14139 §5.14.10), plus Display 1
//! (regulator setting, current limit). `st25r100_diag [/dev/spidev1.0] [rounds]`
fn main() -> nfc::Result<()> {
    #[cfg(target_os = "linux")]
    {
        use nfc::spi::{Bus, Spidev};
        use nfc::st25r100::{Antenna, St25r100};
        use std::time::Duration;
        let mut args = std::env::args().skip(1);
        let path = args.next().unwrap_or_else(|| "/dev/spidev1.0".to_owned());
        let rounds: u32 = args.next().and_then(|s| s.parse().ok()).unwrap_or(10);
        let mut chip = St25r100::new(Spidev::open(std::path::Path::new(&path))?, Antenna::One);
        chip.begin()?;
        // Raw access next to the driver, for registers the driver has no reason to expose.
        let mut raw = Spidev::open(std::path::Path::new(&path))?;
        let mut rd = |reg: u8| -> u8 {
            let mut v = [0u8];
            raw.transfer(&[0x80 | reg], &mut v).unwrap();
            v[0]
        };
        let mut wr_raw = Spidev::open(std::path::Path::new(&path))?;
        let mut wr = |bytes: &[u8]| wr_raw.transfer(bytes, &mut []).unwrap();
        for round in 0..rounds {
            for antenna in [Antenna::One, Antenna::Two] {
                chip.set_antenna(antenna)?;
                let mut line = format!("round {round} {antenna:?}:");
                for (label, op) in [("off", 0x02u8), ("on ", 0x32u8)] {
                    wr(&[0x00, op]);
                    std::thread::sleep(Duration::from_millis(10));
                    let _ = rd(0x3e); // clear IRQ 3
                    wr(&[0x76]); // clear WU calibration
                    wr(&[0x7a]); // I/Q measurement
                    std::thread::sleep(Duration::from_millis(5));
                    let dct = rd(0x3e) & 0x10 != 0;
                    let (i, q) = (rd(0x2c), rd(0x31));
                    let d1 = rd(0x0f);
                    line += &format!(
                        "  field {label} I={i:3} Q={q:3}{} d1={d1:02x}{}",
                        if dct { "" } else { " (no dct)" },
                        if d1 & 0x80 != 0 { " I_LIM" } else { "" }
                    );
                }
                println!("{line}");
            }
            wr(&[0x00, 0x02]);
            std::thread::sleep(Duration::from_millis(700));
        }
    }
    Ok(())
}
