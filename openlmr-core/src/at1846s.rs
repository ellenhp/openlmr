use embassy_stm32::gpio::{Level, Speed};
use embassy_stm32::peripherals::{PA2, PA5, PC4, PC5};
use embassy_stm32::{
    gpio::Output,
    i2c::{self, I2c, RxDma, TxDma},
};
use embassy_time::Timer;

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

const FM_COMMANDS: [[u8; 3]; 8] = [
    [0x33, 0x44, 0xA5], // agc number (recommended value)
    [0x41, 0x44, 0x31], // Digital voice gain, (bits 6:0) however default value is supposed to be 0x4006 hence some bits are being set outside the documented range
    [0x42, 0x10, 0xF0], // RDA1846 lists this as Vox Shut threshold
    [0x43, 0x00, 0xA9], // FM deviation
    [0x58, 0xBC, 0x85], // Enable some filters for FM e.g. High and Low Pass Filters. G4EML...De-emphasis turned off as this is done by the HRC6000 on the MDUV380.
    [0x44, 0x06, 0xCC], // set internal volume to 80% .
    [0x3A, 0x00, 0xC3], // modu_det_sel (SQ setting)
    [0x40, 0x00, 0x30], // UNDOCUMENTED. THIS IS THE MAGIC REGISTER WHICH ALLOWS LOW FREQ AUDIO BY SETTING THE LS BIT. So it should be cleared to receive FM
];

const DMR_COMMANDS: [[u8; 3]; 12] = [
    [0x40, 0x00, 0x31], // UNDOCUMENTED. THIS IS THE MAGIC REGISTER WHICH ALLOWS LOW FREQ AUDIO BY SETTING THE LS BIT
    [0x15, 0x11, 0x00], // IF tuning bits (12:9)
    [0x32, 0x44, 0x95], // agc target power
    [0x3A, 0x00, 0xC3], // modu_det_sel (SQ setting). Tx No mic input, as the DMR signal directly modulates the master reference oscillator
    [0x3C, 0x1B, 0x34], // Pk_det_th (SQ setting)
    [0x3F, 0x29, 0xD1], // Rssi3_th (SQ setting)
    [0x41, 0x41, 0x22], // Digital voice gain, (bits 6:0) however default value is supposed to be 0x4006 hence some bits are being set outside the documented range
    [0x42, 0x10, 0x52], // RDA1846 lists this as Vox Shut threshold
    [0x43, 0x01, 0x00], // FM deviation
    [0x48, 0x19, 0xB1], // noise1_th (SQ setting)
    [0x58, 0x9C, 0xDD], // Disable all filters in DMR mode
    [0x44, 0x07, 0xFF], // set internal volume to 100% (doesn't seem to decode correctly at lower levels on this radio)
];

pub struct AT1846S<'a, T, TXDMA, RXDMA>
where
    T: i2c::Instance,
    TXDMA: TxDma<T>,
    RXDMA: RxDma<T>,
{
    i2c: I2c<'a, T, TXDMA, RXDMA>,
    lna_vhf: Output<'static>,
    lna_uhf: Output<'static>,
    pa_vhf: Output<'static>,
    pa_uhf: Output<'static>,
}

impl<'a, T, TXDMA, RXDMA> AT1846S<'a, T, TXDMA, RXDMA>
where
    T: i2c::Instance,
    TXDMA: TxDma<T>,
    RXDMA: RxDma<T>,
{
    pub async fn init(
        i2c: I2c<'a, T, TXDMA, RXDMA>,
        lna_vhf: PA5,
        lna_uhf: PA2,
        pa_vhf: PC5,
        pa_uhf: PC4,
    ) -> AT1846S<'a, T, TXDMA, RXDMA> {
        let lna_vhf = Output::new(lna_vhf, Level::Low, Speed::Low);
        let lna_uhf = Output::new(lna_uhf, Level::Low, Speed::Low);
        let pa_vhf = Output::new(pa_vhf, Level::Low, Speed::Low);
        let pa_uhf = Output::new(pa_uhf, Level::Low, Speed::Low);
        let mut at1846s = AT1846S {
            i2c,
            lna_vhf,
            lna_uhf,
            pa_vhf,
            pa_uhf,
        };
        at1846s.write_reg(0x30, 0x0001).await;
        Timer::after_millis(50).await;
        for (reg, data) in &INIT_COMMANDS {
            at1846s.write_reg(*reg, *data).await;
        }
        // Delay before starting cal.
        Timer::after_millis(50).await;
        // Start cal.
        at1846s.write_reg(0x30, 0x40A6).await;
        // Delay longer for cal to run.
        Timer::after_millis(160).await;
        // Stop cal.
        at1846s.write_reg(0x30, 0x4006).await;

        Timer::after_millis(100).await;
        for (reg, data) in &POST_INIT_COMMANDS {
            at1846s.write_reg(*reg, *data).await;
        }
        Timer::after_millis(160).await;
        at1846s.fm_mode().await;
        at1846s
    }

    pub async fn receive_mode(&mut self) {
        self.pa_vhf.set_low();
        self.pa_uhf.set_low();
        Timer::after_millis(1).await;
        self.lna_uhf.set_low();
        self.mask_write_reg(0x30, 0x0060, 0x0020).await;
    }

    pub async fn transmit_mode(&mut self) {
        self.lna_vhf.set_low();
        self.lna_uhf.set_low();
        Timer::after_millis(1).await;
        // self.mask_write_reg(0x30, 0x0060, 0x0040).await;
        self.pa_uhf.set_high();
    }

    pub async fn get_rssi(&mut self) -> i16 {
        -137i16 + (self.read_reg(0x1B).await >> 8) as i16
    }

    pub async fn fm_mode(&mut self) {
        for command in &FM_COMMANDS {
            self.write_reg_raw(&command).await;
        }
        self.reload_config().await;
    }

    pub async fn dmr_mode(&mut self) {
        for command in &DMR_COMMANDS {
            self.write_reg_raw(&command).await;
        }
        self.reload_config().await;
    }

    pub async fn tune(&mut self, frequency_hz: u32) {
        let val = ((frequency_hz as u64) * 16) / 1000;

        let freq_hi = (((val >> 16) & 0xFFFF) as u16).to_be_bytes();
        let freq_lo = ((val & 0xFFFF) as u16).to_be_bytes();

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
        self.i2c.write(AT1846S_I2C_ADDRESS, data).await.unwrap();
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
            .await
            .unwrap();
        u16::from_be_bytes(buf)
    }
}
