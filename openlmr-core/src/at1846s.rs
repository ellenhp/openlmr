use embassy_stm32::i2c::{self, I2c, RxDma, TxDma};
use embassy_time::Timer;

const AT1846S_I2C_ADDRESS: u8 = 0x2Eu8;

const INIT_COMMANDS: [[u8; 3]; 27] = [
    [0x30, 0x00, 0x04], // Poweron 1846s
    [0x04, 0x0F, 0xD0], // Clock mode 25.6MHz/26MHz
    [0x0A, 0x7C, 0x20], // Default Value
    [0x13, 0xA1, 0x00], // Unknown Register
    [0x1F, 0x10, 0x01], // gpio6 = sq out, GPIO1=CSS_OUT
    [0x31, 0x00, 0x31], // UNDOCUMENTED - use recommended value
    [0x33, 0x44, 0xA5], // agc number
    [0x34, 0x2B, 0x89], // Rx digital gain (recommend value)
    [0x41, 0x41, 0x22], // Digital voice gain, (bits 6:0) however default value is supposed to be 0x4006 hence some bits are being set outside the documented range
    [0x42, 0x10, 0x52], // RDA1846 lists this as Vox Shut threshold
    [0x43, 0x01, 0x00], // FM deviation
    [0x44, 0x07, 0xFF], // Rx and tx gain controls
    [0x3A, 0x00, 0xC3], // SQL Config
    [0x59, 0x0B, 0x90], // Deviation settings
    [0x47, 0x7F, 0x2F], // UNDOCUMENTED - UV82 and GD77 use the same values
    [0x4F, 0x2C, 0x62], // Undocumented
    [0x53, 0x00, 0x94], // UNDOCUMENTED - use recommended value
    [0x54, 0x2A, 0x3C], // UNDOCUMENTED - use recommended value
    [0x55, 0x00, 0x81], // UNDOCUMENTED - use recommended value
    [0x56, 0x0B, 0x02], // SQ detection time (SQ setting)
    [0x57, 0x1C, 0x00], // bypass rssi_lpfilter
    [0x58, 0x9C, 0xDD], // Filters custom setting
    [0x5A, 0x06, 0xDB], // Unknown
    [0x63, 0x16, 0xAD], // Pre_emphasis bypass threshold (recommended value)
    [0x0F, 0x8A, 0x24], // Unknown
    [0x05, 0x87, 0x63], // Unknown
    /*these settings are for the DTMF. Probably not needed on the UV380 as we use the HRC6000 for DTMF
    [0x67, 0x06, 0x28], // Set DTMF Tone (Probably not needed on the UV380)
    [0x68, 0x05, 0xE5], // Set DTMF Tone (Probably not needed on the UV380)
    [0x69, 0x05, 0x55], // Set DTMF Tone (Probably not needed on the UV380)
    [0x6A, 0x04, 0xB8], // Set DTMF Tone (Probably not needed on the UV380)
    [0x6B, 0x02, 0xFE], // Set DTMF Tone (Probably not needed on the UV380)
    [0x6C, 0x01, 0xDD], // Set DTMF Tone (Probably not needed on the UV380)
    [0x6D, 0x00, 0xB1], // Set DTMF Tone (Probably not needed on the UV380)
    [0x6E, 0x0F, 0x82], // Set DTMF Tone (Probably not needed on the UV380)
    [0x6F, 0x01, 0x7A], // Set DTMF 2nd Harmonic (Probably not needed on the UV380)
    [0x70, 0x00, 0x4C], // Set DTMF 2nd Harmonic (Probably not needed on the UV380)
    [0x71, 0x0F, 0x1D], // Set DTMF 2nd Harmonic (Probably not needed on the UV380)
    [0x72, 0x0D, 0x91], // Set DTMF 2nd Harmonic (Probably not needed on the UV380)
    [0x73, 0x0A, 0x3E], // Set DTMF 2nd Harmonic (Probably not needed on the UV380)
    [0x74, 0x09, 0x0F], // Set DTMF 2nd Harmonic (Probably not needed on the UV380)
    [0x75, 0x08, 0x33], // Set DTMF 2nd Harmonic (Probably not needed on the UV380)
    [0x76, 0x08, 0x06], // Set DTMF 2nd Harmonic (Probably not needed on the UV380)
    */
    [0x30, 0x40, 0xA4], // Setup to calibrate
];

const POST_INIT_COMMANDS: [[u8; 3]; 1] = [
    [0x15, 0x11, 0x00], // IF tuning bits (12:9)
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
}

impl<'a, T, TXDMA, RXDMA> AT1846S<'a, T, TXDMA, RXDMA>
where
    T: i2c::Instance,
    TXDMA: TxDma<T>,
    RXDMA: RxDma<T>,
{
    pub async fn init(i2c: I2c<'a, T, TXDMA, RXDMA>) -> AT1846S<'a, T, TXDMA, RXDMA> {
        let mut at1846s = AT1846S { i2c };
        at1846s.write_reg_raw(&[0x30, 0x00, 0x01]).await;
        Timer::after_millis(25).await;
        at1846s.write_reg_raw(&[0x30, 0x00, 0x01]).await;
        for command in &INIT_COMMANDS {
            at1846s.write_reg_raw(&command).await;
        }
        Timer::after_millis(25).await;
        for command in &POST_INIT_COMMANDS {
            at1846s.write_reg_raw(&command).await;
        }
        Timer::after_millis(25).await;
        at1846s.fm_mode().await;
        at1846s
    }

    pub async fn fm_mode(&mut self) {
        for command in &FM_COMMANDS {
            self.write_reg_raw(&command).await;
        }
    }

    pub async fn dmr_mode(&mut self) {
        for command in &DMR_COMMANDS {
            self.write_reg_raw(&command).await;
        }
    }

    pub async fn tune(&mut self, frequency_hz: u32) {
        let val = ((frequency_hz as u64) * 16) / 1000;

        let freq_hi = (((val >> 16) & 0xFFFF) as u16).to_be_bytes();
        let freq_lo = ((val & 0xFFFF) as u16).to_be_bytes();

        self.write_reg_raw(&[0x29, freq_hi[0], freq_hi[1]]).await;
        self.write_reg_raw(&[0x2A, freq_lo[0], freq_lo[1]]).await;

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

    async fn read_reg(&mut self, reg: u8) -> u16 {
        let mut buf = [0u8; 2];
        self.i2c
            .write_read(AT1846S_I2C_ADDRESS, &[reg], &mut buf)
            .await
            .unwrap();
        u16::from_be_bytes(buf)
    }
}
