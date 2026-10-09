//! Bench probe: is an ST25R100 answering on the SPI bus? Reads its identity and the first
//! configuration registers. `st25r100_probe [/dev/spidev1.0]`
fn main() -> std::io::Result<()> {
    #[cfg(target_os = "linux")]
    {
        use nfc::spi::{Bus, Spidev};
        let path = std::env::args()
            .nth(1)
            .unwrap_or_else(|| "/dev/spidev1.0".to_owned());
        let mut bus = Spidev::open(std::path::Path::new(&path))?;
        let mut id = [0u8];
        bus.transfer(&[0x80 | 0x3f], &mut id)?;
        println!(
            "IC identity (0x3F) = {:#04x}  (ST25R100 is 0xa8 + revision)",
            id[0]
        );
        let mut regs = [0u8; 8];
        bus.transfer(&[0x80], &mut regs)?;
        println!("registers 0x00..0x07 = {regs:02x?}");
    }
    Ok(())
}
