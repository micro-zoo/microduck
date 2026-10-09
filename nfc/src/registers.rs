//! CLRC663 constants. No logic here. Names and values as in `ntag663/registers.py`.

pub const REG_COMMAND: u8 = 0x00;
pub const REG_FIFOCONTROL: u8 = 0x02;
pub const REG_FIFOLENGTH: u8 = 0x04;
pub const REG_FIFODATA: u8 = 0x05;
pub const REG_IRQ0: u8 = 0x06;
pub const REG_IRQ1: u8 = 0x07;
pub const REG_IRQ0EN: u8 = 0x08;
pub const REG_IRQ1EN: u8 = 0x09;
pub const REG_ERROR: u8 = 0x0A;
pub const REG_RXBITCTRL: u8 = 0x0C;
pub const REG_RXCOLL: u8 = 0x0D;
pub const REG_T0CONTROL: u8 = 0x0F;
pub const REG_T0RELOADHI: u8 = 0x10;
pub const REG_T0RELOADLO: u8 = 0x11;
pub const REG_T0COUNTERVALHI: u8 = 0x12;
pub const REG_T0COUNTERVALLO: u8 = 0x13;
pub const REG_DRVMODE: u8 = 0x28;
pub const REG_TXCRCPRESET: u8 = 0x2C;
pub const REG_RXCRCCON: u8 = 0x2D;
pub const REG_TXDATANUM: u8 = 0x2E;
pub const REG_VERSION: u8 = 0x7F;

pub const CMD_IDLE: u8 = 0x00;
pub const CMD_TRANSCEIVE: u8 = 0x07;
pub const CMD_LOADPROTOCOL: u8 = 0x0D;
pub const CMD_SOFTRESET: u8 = 0x1F;

/// FIFOControl bit 7: set means 255 bytes, clear means 512.
pub const FIFOCONTROL_SIZE_255: u8 = 1 << 7;
pub const FIFOCONTROL_FLUSH: u8 = 1 << 4;
/// Bits 9 and 8 of the FIFO length, in 512-byte mode.
pub const FIFOCONTROL_LENGTH_EXT_MASK: u8 = 0x03;

pub const IRQ0_RX: u8 = 1 << 2;
pub const IRQ0_ERR: u8 = 1 << 1;
pub const IRQ1_TIMER0: u8 = 1 << 0;
/// Writing 0x7F — bit 7 clear, bits 6..0 set — clears every IRQ0/IRQ1 flag.
pub const IRQ_CLEAR_ALL: u8 = 0x7F;
pub const IRQ0EN_RX: u8 = 1 << 2;
pub const IRQ0EN_ERR: u8 = 1 << 1;
pub const IRQ1EN_TIMER0: u8 = 1 << 0;

pub const ERROR_PROT: u8 = 1 << 1;
pub const ERROR_INTEG: u8 = 1 << 0;
/// Errors that make the received bytes untrustworthy: refuse to hand them on.
pub const ERROR_INVALID_DATA: u8 = ERROR_INTEG | ERROR_PROT;

pub const RXBITCTRL_RXALIGN_SHIFT: u8 = 4;
pub const RXBITCTRL_RXALIGN_MASK: u8 = 0b111 << 4;

/// RxColl.CollPosValid: the reliable collision signal, independent of ErrIRQ (which CollDet does
/// not raise). Measured at 0 over 15 exchanges with a single tag, at every protocol step.
pub const RXCOLL_POSVALID: u8 = 1 << 7;
pub const RXCOLL_POS_MASK: u8 = 0x7F;

pub const TCONTROL_START_TX_END: u8 = 0b01 << 4;
pub const TCONTROL_CLK_211KHZ: u8 = 0b01;
/// 211.875 kHz, so 4.7198 µs a tick; 16 bits of timer caps a timeout near 309 ms.
pub const TICK_US: f64 = 4.7198;

pub const RECOM_14443A_CRC: u8 = 0x18;
pub const TXDATANUM_DATAEN: u8 = 1 << 3;

pub const PROTO_ISO14443A_106: u8 = 0;

/// AN1102's recommended registers for ISO14443A at 106 kbit/s, written from REG_DRVMODE up.
pub const RECOM_14443A_ID1_106: [u8; 18] = [
    0x8A, 0x08, 0x21, 0x1A, 0x18, 0x18, 0x0F, 0x27, 0x00, 0xC0, 0x12, 0xCF, 0x00, 0x04, 0x90, 0x32,
    0x12, 0x0A,
];

pub const VERSION_CLRC663: u8 = 0x1A;
