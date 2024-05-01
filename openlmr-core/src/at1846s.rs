use embedded_hal::digital::OutputPin;
use stm32f4xx_hal::{
    i2c::{self, I2c},
    prelude::*,
};

const AT1846S_I2C_ADDRESS: u8 = 0x2Eu8;

const INIT_COMMANDS: [(u8, u16); 27] = [
    (0x30, 0x0004), // Chip enable
    (0x04, 0x0FD0), // 26MHz crystal frequency
    (0x1F, 0x1000), // Gpio6 squelch output
    (0x09, 0x03AC),
    (0x24, 0x0001),
    (0x31, 0x0031),
    (0x33, 0x45F5), // AGC number
    (0x34, 0x2B89), // RX digital gain
    (0x3F, 0x3263), // RSSI 3 threshold
    (0x41, 0x470F), // Tx digital gain
    (0x42, 0x1036),
    (0x43, 0x00BB),
    (0x44, 0x06FF), // Tx digital gain
    (0x47, 0x7F2F), // Soft mute
    (0x4E, 0x0082),
    (0x4F, 0x2C62),
    (0x53, 0x0094),
    (0x54, 0x2A3C),
    (0x55, 0x0081),
    (0x56, 0x0B02),
    (0x57, 0x1C00), // Bypass RSSI low-pass
    (0x5A, 0x4935), // SQ detection time
    (0x58, 0xBCCD),
    (0x62, 0x3263), // Modulation detect tresh
    (0x4E, 0x2082),
    (0x63, 0x16AD),
    (0x30, 0x40A4),
];

const POST_INIT_COMMANDS: [(u8, u16); 16] = [
    (0x58, 0xBCED),
    (0x0A, 0x7BA0), // PGA gain
    (0x41, 0x4731), // Tx digital gain
    (0x44, 0x05FF), // Tx digital gain
    (0x59, 0x09D2), // Mixer gain
    (0x44, 0x05CF), // Tx digital gain
    (0x44, 0x05CC), // Tx digital gain
    (0x48, 0x1A32), // Noise 1 threshold
    (0x60, 0x1A32), // Noise 2 threshold
    (0x3F, 0x29D1), // RSSI 3 threshold
    (0x0A, 0x7BA0), // PGA gain
    (0x49, 0x0C96), // RSSI SQL thresholds
    (0x33, 0x45F5), // AGC number
    (0x41, 0x470F), // Tx digital gain
    (0x42, 0x1036),
    (0x43, 0x00BB),
];

const FM_COMMANDS: [(u8, u16); 2] = [
    // Bit 0  = 1: CTCSS LPF badwidth to 250Hz
    // Bit 3  = 0: enable CTCSS HPF
    // Bit 4  = 0: enable CTCSS LPF
    // Bit 5  = 0: enable voice LPF
    // Bit 6  = 0: enable voice HPF
    // Bit 7  = 0: enable pre/de-emphasis
    // Bit 11 = 1: bypass VOX HPF
    // Bit 12 = 1: bypass VOX LPF
    // Bit 13 = 0: normal RSSI LPF bandwidth
    (0x58, 0x9C05),
    (0x40, 0x0030),
];

const DMR_COMMANDS: [(u8, u16); 8] = [
    (0x3A, 0x00C2),
    (0x33, 0x45F5),
    (0x41, 0x4731),
    (0x42, 0x1036),
    (0x43, 0x00BB),
    // Bit 0  = 1: CTCSS LPF bandwidth to 250Hz
    // Bit 3  = 1: bypass CTCSS HPF
    // Bit 4  = 1: bypass CTCSS LPF
    // Bit 5  = 1: bypass voice LPF
    // Bit 6  = 1: bypass voice HPF
    // Bit 7  = 1: bypass pre/de-emphasis
    // Bit 11 = 1: bypass VOX HPF
    // Bit 12 = 1: bypass VOX LPF
    // Bit 13 = 1: bypass RSSI LPF
    (0x58, 0xBCFD),
    (0x44, 0x06CC),
    (0x40, 0x0031),
];

pub struct AT1846S<
    I2cInst: i2c::Instance,
    LnaVhf: OutputPin,
    LnaUhf: OutputPin,
    PwrA1: OutputPin,
    PwrA2: OutputPin,
    PaSel: OutputPin,
> {
    i2c: I2c<I2cInst>,
    lna_vhf: LnaVhf,
    lna_uhf: LnaUhf,
    pwr_a1: PwrA1,
    pwr_a2: PwrA2,
    pa_sel: PaSel,
    is_transmitting: bool,
    is_receiving: bool,
    is_uhf: bool,
}

impl<
        I2cInst: i2c::Instance,
        LnaVhf: OutputPin,
        LnaUhf: OutputPin,
        PwrA1: OutputPin,
        PwrA2: OutputPin,
        PaSel: OutputPin,
    > AT1846S<I2cInst, LnaVhf, LnaUhf, PwrA1, PwrA2, PaSel>
{
    pub fn new(
        i2c: I2c<I2cInst>,
        lna_vhf: LnaVhf,
        lna_uhf: LnaUhf,
        pwr_a1: PwrA1,
        pwr_a2: PwrA2,
        pa_sel: PaSel,
    ) -> AT1846S<I2cInst, LnaVhf, LnaUhf, PwrA1, PwrA2, PaSel> {
        let at1846s = AT1846S {
            i2c,
            lna_vhf,
            lna_uhf,
            pwr_a1,
            pwr_a2,
            pa_sel,
            is_transmitting: false,
            is_receiving: false,
            is_uhf: false,
        };
        at1846s
    }

    pub async fn init(&mut self) {
        self.write_reg(0x30, 0x0001).await;
        crate::Mono::delay(50.millis().into()).await;
        for (reg, data) in &INIT_COMMANDS {
            self.write_reg(*reg, *data).await;
        }
        // Delay before starting cal.
        crate::Mono::delay(50.millis().into()).await;
        // Start cal.
        self.write_reg(0x30, 0x40A6).await;
        // Delay longer for cal to run.
        crate::Mono::delay(160.millis().into()).await;
        // Stop cal.
        self.write_reg(0x30, 0x4006).await;

        crate::Mono::delay(100.millis().into()).await;
        for (reg, data) in &POST_INIT_COMMANDS {
            self.write_reg(*reg, *data).await;
        }
        crate::Mono::delay(160.millis().into()).await;
        self.fm_mode().await;
        crate::Mono::delay(160.millis().into()).await;
    }

    pub async fn receive_mode(&mut self) {
        self.pwr_a1.set_low().unwrap();
        self.pwr_a2.set_low().unwrap();
        self.pa_sel.set_low().unwrap();
        self.is_transmitting = false;
        crate::Mono::delay(1.millis().into()).await;
        if self.is_uhf {
            self.lna_uhf.set_high().unwrap();
            self.lna_vhf.set_low().unwrap();
        } else {
            self.lna_uhf.set_low().unwrap();
            self.lna_vhf.set_high().unwrap();
        }
        self.mask_write_reg(0x30, 0x0060, 0x0020).await;
        self.is_receiving = true;
    }

    pub async fn transmit_mode(&mut self) {
        self.lna_vhf.set_low().unwrap();
        self.lna_uhf.set_low().unwrap();
        self.is_receiving = false;
        crate::Mono::delay(1.millis().into()).await;
        self.mask_write_reg(0x30, 0x0060, 0x0040).await;
        self.pa_sel.set_high().unwrap();
        self.pwr_a1.set_high().unwrap();
        self.pwr_a2.set_high().unwrap();
        self.is_transmitting = true;
    }

    pub fn receiving(&self) -> bool {
        self.is_receiving
    }

    pub fn transmitting(&self) -> bool {
        self.is_transmitting
    }

    pub async fn set_rx_gain(&mut self, gain: u8) {
        let value = (((gain / 16) & 0x0F) as u16) << 4;
        self.mask_write_reg(0x44, 0x00F0, value).await;
        // self.mask_write_reg(0x44, 0x000F, static_cast< uint16_t >(digitalGain));
    }

    pub async fn get_rssi(&mut self) -> i16 {
        -137i16 + (self.read_reg(0x1B).await >> 8) as i16
    }

    pub async fn fm_mode(&mut self) {
        for (reg, data) in &FM_COMMANDS {
            self.write_reg(*reg, *data).await;
        }
        self.reload_config().await;
    }

    pub async fn dmr_mode(&mut self) {
        for (reg, data) in &DMR_COMMANDS {
            self.write_reg(*reg, *data).await;
        }
        self.reload_config().await;
    }

    pub async fn tune(&mut self, frequency_hz: u32) {
        let val = ((frequency_hz as u64) * 16) / 1000;

        let freq_hi = (((val >> 16) & 0xFFFF) as u16).to_be_bytes();
        let freq_lo = ((val & 0xFFFF) as u16).to_be_bytes();

        self.is_uhf = frequency_hz > 300_000_000;

        if self.is_receiving {
            if self.is_uhf {
                self.lna_uhf.set_high().unwrap();
                self.lna_vhf.set_low().unwrap();
            } else {
                self.lna_uhf.set_low().unwrap();
                self.lna_vhf.set_high().unwrap();
            }
        }

        self.write_reg_raw(&[0x29, freq_hi[0], freq_hi[1]]).await;
        self.write_reg_raw(&[0x2A, freq_lo[0], freq_lo[1]]).await;

        self.reload_config().await;
    }

    pub async fn set_25khz_bw(&mut self) {
        self.write_reg(0x15, 0x1F00).await; // Tuning bit
        self.write_reg(0x32, 0x7564).await; // AGC target power
        self.write_reg(0x3A, 0x44C3).await; // Modulation detect sel
        self.write_reg(0x3F, 0x29D2).await; // RSSI 3 threshold
        self.write_reg(0x3C, 0x0E1C).await; // Peak detect threshold
        self.write_reg(0x48, 0x1E38).await; // Noise 1 threshold
        self.write_reg(0x62, 0x3767).await; // Modulation detect tresh
        self.write_reg(0x65, 0x248A).await;
        self.write_reg(0x66, 0xFF2E).await; // RSSI comp and AFC range
        self.write_reg(0x7F, 0x0001).await; // Switch to page 1
        self.write_reg(0x06, 0x0024).await; // AGC gain table
        self.write_reg(0x07, 0x0214).await;
        self.write_reg(0x08, 0x0224).await;
        self.write_reg(0x09, 0x0314).await;
        self.write_reg(0x0A, 0x0324).await;
        self.write_reg(0x0B, 0x0344).await;
        self.write_reg(0x0D, 0x1384).await;
        self.write_reg(0x0E, 0x1B84).await;
        self.write_reg(0x0F, 0x3F84).await;
        self.write_reg(0x12, 0xE0EB).await;
        self.write_reg(0x7F, 0x0000).await; // Back to page 0
        self.mask_write_reg(0x30, 0x3000, 0x3000).await;
        self.reload_config().await;
    }

    pub async fn reload_config(&mut self) {
        let mode = self.read_reg(0x30).await & 0x0060;
        self.mask_write_reg(0x30, 0x0060, 0x0000).await; // RX and TX off.
        self.mask_write_reg(0x30, 0x0060, mode).await; // Restore previous mode.
    }

    async fn write_reg_raw(&mut self, data: &[u8; 3]) {
        self.i2c.write(AT1846S_I2C_ADDRESS, data).unwrap();
    }

    async fn write_reg(&mut self, reg: u8, data: u16) {
        let bytes = data.to_be_bytes();
        self.write_reg_raw(&[reg, bytes[0], bytes[1]]).await;
    }
    async fn mask_write_reg(&mut self, reg: u8, mask: u16, data: u16) {
        let old_reg = self.read_reg(reg).await;
        let new_reg = (old_reg & (!mask)) | (data & mask);
        self.write_reg(reg, new_reg).await;
    }

    pub async fn read_reg(&mut self, reg: u8) -> u16 {
        let mut buf = [0u8; 2];
        self.i2c
            .write_read(AT1846S_I2C_ADDRESS, &[reg], &mut buf)
            .unwrap();
        u16::from_be_bytes(buf)
    }
}
