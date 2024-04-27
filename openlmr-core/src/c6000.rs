use embedded_hal::digital::{InputPin, OutputPin};
use stm32f4xx_hal::prelude::*;

const INIT_SEQ1: [u8; 8] = [0x01, 0x04, 0xD5, 0xD7, 0xF7, 0x7F, 0xD7, 0x57];
const INIT_SEQ2: [u8; 46] = [
    0x04, 0x11, 0x80, 0x0C, 0x22, 0x01, 0x00, 0x00, 0x33, 0xEF, 0x00, 0xFF, 0xFF, 0xFF, 0xF0, 0xF0,
    0x10, 0x00, 0x00, 0x06, 0x3B, 0xF8, 0x0E, 0xFD, 0x40, 0xFF, 0x00, 0x0B, 0x00, 0x00, 0x00, 0x06,
    0x0B, 0x00, 0x17, 0x02, 0xFF, 0xE0, 0x14, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
];
const INIT_SEQ3: [u8; 34] = [
    0x01, 0x10, 0x69, 0x69, 0x96, 0x96, 0x96, 0x99, 0x99, 0x99, 0xA5, 0xA5, 0xAA, 0xAA, 0xCC, 0xCC,
    0x00, 0xF0, 0x01, 0xFF, 0x01, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x0D, 0x70, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00,
];
const INIT_SEQ4: [u8; 18] = [
    0x01, 0x30, 0x00, 0x00, 0x20, 0x3C, 0xFF, 0xFF, 0x3F, 0x50, 0x07, 0x60, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00,
];
const INIT_SEQ5: [u8; 9] = [0x01, 0x40, 0x00, 0x01, 0x01, 0x02, 0x01, 0x1E, 0xF0];
const INIT_SEQ6: [u8; 7] = [0x01, 0x50, 0x00, 0x08, 0xEB, 0x78, 0x67];
const INIT_SEQ7: [u8; 8] = [0x01, 0x04, 0xD5, 0xD7, 0xF7, 0x7F, 0xD7, 0x57];

enum RegOpmode {
    /// Auxiliary configuration registers.
    AUX = 1,
    /// Write TX data register and read RX data register.
    _DATA = 2,
    /// Voice prompt sample register.
    _SOUND = 3,
    /// Main configuration registers.
    CONFIG = 4,
    /// AMBE3000 configuration register.
    _AMBE3K = 5,
    /// Write RX data register and read TX data register.
    _DATAR = 6,
    /// AMBE1000 configuration register.
    _AMBE1K = 7,
}

pub struct C6000<
    ChipSelect: OutputPin,
    SpiClk: OutputPin,
    Mosi: OutputPin,
    Miso: InputPin,
    Standby: OutputPin,
> {
    /// Active-low.
    cs: ChipSelect,
    clk: SpiClk,
    mosi: Mosi,
    miso: Miso,
    /// Active-low.
    _standby: Standby,
}

impl<
        ChipSelect: OutputPin,
        SpiClk: OutputPin,
        Mosi: OutputPin,
        Miso: InputPin,
        Standby: OutputPin,
    > C6000<ChipSelect, SpiClk, Mosi, Miso, Standby>
{
    pub async fn init(
        mut cs: ChipSelect,
        clk: SpiClk,
        mosi: Mosi,
        miso: Miso,
        mut standby: Standby,
    ) -> C6000<ChipSelect, SpiClk, Mosi, Miso, Standby> {
        cs.set_high().unwrap();
        standby.set_high().unwrap();
        crate::Mono::delay(10.millis().into()).await;

        // Exit from sleep pulling down DMR_SLEEP
        standby.set_low().unwrap();
        crate::Mono::delay(10.millis().into()).await;

        let mut c6000 = C6000 {
            cs,
            clk,
            mosi,
            miso,
            _standby: standby,
        };

        c6000.startup().await;

        c6000.fm_mode().await;

        c6000
    }

    pub async fn startup(&mut self) {
        self.write_reg(RegOpmode::CONFIG, 0x0A, 0x80).await; // Clock connected to crystal
        self.write_reg(RegOpmode::CONFIG, 0x0B, 0x28).await; // Set PLL M Register
        self.write_reg(RegOpmode::CONFIG, 0x0C, 0x33).await; // Set PLL Dividers
        crate::Mono::delay(250.millis().into()).await;

        self.write_reg(RegOpmode::CONFIG, 0x0A, 0x00).await; // Clock connected to PLL
        self.write_reg(RegOpmode::CONFIG, 0xB9, 0x05).await; // System clock frequency
        self.write_reg(RegOpmode::CONFIG, 0xBA, 0x04).await; // Codec clock frequency
        self.write_reg(RegOpmode::CONFIG, 0xBB, 0x02).await; // Output clock frequency
        self.write_reg(RegOpmode::CONFIG, 0xA1, 0x80).await; // FM_mod, all modes cleared
        self.write_reg(RegOpmode::CONFIG, 0x10, 0xF3).await; // FM mode, Tier II, TimeSlot, 3rd layer mode, aligned (?)
        self.write_reg(RegOpmode::CONFIG, 0x40, 0x43).await; // Enable RX synchronisation, normal mode (no test)
        self.write_reg(RegOpmode::CONFIG, 0x07, 0x0B).await; // IF frequency - high 8 bit
        self.write_reg(RegOpmode::CONFIG, 0x08, 0xB8).await; // IF frequency - mid 8 bit
        self.write_reg(RegOpmode::CONFIG, 0x09, 0x00).await; // IF frequency - low 8 bit
        self.write_seq(&INIT_SEQ1).await;
        self.write_reg(RegOpmode::CONFIG, 0x01, 0xF8).await; // Swap TX IQ, swap RX IQ, two point mode for TX, baseband IQ mode for RX
        self.write_seq(&INIT_SEQ2).await;
        self.write_reg(RegOpmode::CONFIG, 0x00, 0x2A).await; // Reset codec, reset vocoder, reset I2S

        self.write_reg(RegOpmode::CONFIG, 0x06, 0x20).await; // Vocoder output connected to universal interface (?)
        self.write_reg(RegOpmode::CONFIG, 0x14, 0x59).await; // local address - low 8 bit
        self.write_reg(RegOpmode::CONFIG, 0x15, 0xF5).await; // local address - mid 8 bit
        self.write_reg(RegOpmode::CONFIG, 0x16, 0x21).await; // local address - high 8 bit
        self.write_seq(&INIT_SEQ3).await;
        self.write_seq(&INIT_SEQ4).await;
        self.write_seq(&INIT_SEQ5).await;
        self.write_seq(&INIT_SEQ6).await;
        self.write_reg(RegOpmode::AUX, 0x52, 0x08).await;
        self.write_reg(RegOpmode::AUX, 0x53, 0xEB).await;
        self.write_reg(RegOpmode::AUX, 0x54, 0x78).await;
        self.write_reg(RegOpmode::AUX, 0x45, 0x1E).await;
        self.write_reg(RegOpmode::AUX, 0x37, 0x50).await;
        self.write_reg(RegOpmode::AUX, 0x35, 0xFF).await;
        self.write_reg(RegOpmode::CONFIG, 0x39, 0x02).await; // Undocumented register
        self.write_reg(RegOpmode::CONFIG, 0x3D, 0x0A).await; // Undocumented register
        self.write_reg(RegOpmode::CONFIG, 0x83, 0xFF).await; // Clear all interrupt flags
        self.write_reg(RegOpmode::CONFIG, 0x87, 0x00).await; // Disable all interrupt sources
        self.write_reg(RegOpmode::CONFIG, 0x65, 0x0A).await; // Undocumented register
        self.write_reg(RegOpmode::CONFIG, 0x1D, 0xFF).await; // Local unaddress, mask unaddress (?)
        self.write_reg(RegOpmode::CONFIG, 0x1E, 0xF1).await; // Broadcast RX address, broadcast address mask
        self.write_reg(RegOpmode::CONFIG, 0xE2, 0x00).await; // DAC off, mic preamp disabled
        self.write_reg(RegOpmode::CONFIG, 0xE4, 0x27).await; // Lineout gain, first and second stage mic gain
        self.write_reg(RegOpmode::CONFIG, 0xE3, 0x52).await; // Internal ADC and DAC passthrough enable
        self.write_reg(RegOpmode::CONFIG, 0xE5, 0x1A).await; // Undocumented register
        self.write_reg(RegOpmode::CONFIG, 0xE1, 0x0F).await; // Undocumented register
        self.write_reg(RegOpmode::CONFIG, 0xD1, 0xC4).await; // DTMF code width (?)
        self.write_reg(RegOpmode::CONFIG, 0x25, 0x0E).await; // Undocumented register
        self.write_reg(RegOpmode::CONFIG, 0x26, 0xFD).await; // Undocumented register
        self.write_reg(RegOpmode::CONFIG, 0x64, 0x00).await; // Undocumented register
    }

    pub async fn fm_mode(&mut self) {
        self.write_reg(RegOpmode::CONFIG, 0x10, 0x80).await; // FM mode, Tier II, TimeSlot, 3rd layer mode, aligned (?)
        self.write_reg(RegOpmode::CONFIG, 0x01, 0xB0).await; // Swap TX IQ, two point mode for TX, IF mode for RX
        self.write_reg(RegOpmode::CONFIG, 0x81, 0x04).await; // Interrupt mask
        self.write_reg(RegOpmode::CONFIG, 0xE5, 0x1A).await; // Undocumented register
        self.write_reg(RegOpmode::CONFIG, 0xE4, 0x27).await; // Lineout gain, first and second stage mic gain
        self.write_reg(RegOpmode::CONFIG, 0x34, 0x98).await; // FM bpf enabled, 25kHz bandwidth
        self.write_reg(RegOpmode::CONFIG, 0x60, 0x00).await; // Disable both analog and DMR transmission
        self.write_reg(RegOpmode::CONFIG, 0x1F, 0x00).await; // Color code, encryption disabled
        self.write_reg(RegOpmode::AUX, 0x24, 0x00).await;
        self.write_reg(RegOpmode::AUX, 0x25, 0x00).await;
        self.write_reg(RegOpmode::AUX, 0x26, 0x00).await;
        self.write_reg(RegOpmode::AUX, 0x27, 0x00).await;
        self.write_reg(RegOpmode::CONFIG, 0x56, 0x00).await; // Undocumented register
        self.write_reg(RegOpmode::CONFIG, 0x41, 0x40).await; // Start RX for upcoming time slot interrupt
        self.write_reg(RegOpmode::CONFIG, 0x5C, 0x09).await; // Undocumented register
        self.write_reg(RegOpmode::CONFIG, 0x5F, 0xC0).await; // Detect BS and MS frame sequences in 2 layer mode
        self.write_seq(&INIT_SEQ7).await;
        self.write_reg(RegOpmode::CONFIG, 0x11, 0x80).await; // Local channel mode
        self.write_reg(RegOpmode::CONFIG, 0xE0, 0xC9).await; // Codec enabled, LineIn1, LineOut2, I2S slave mode
    }

    pub async fn dmr_mode(&mut self) {
        self.write_reg(RegOpmode::CONFIG, 0x10, 0x4F).await; // DMR mode, Tier I, TimeSlot, 2nd layer mode, relay, aligned (?)
        self.write_reg(RegOpmode::CONFIG, 0x81, 0x19).await; // Interrupt mask
        self.write_reg(RegOpmode::CONFIG, 0x01, 0xF0).await; // Swap TX IQ, swap RX IQ, two point mode for TX, IF mode for RX
        self.write_reg(RegOpmode::CONFIG, 0xE4, 0x27).await; // Lineout gain, first and second stage mic gain
        self.write_reg(RegOpmode::CONFIG, 0xE5, 0x1A).await; // Undocumented register
        self.write_reg(RegOpmode::CONFIG, 0x25, 0x0E).await; // Undocumented register
        self.write_reg(RegOpmode::CONFIG, 0x26, 0xFD).await; // Undocumented register
        self.write_reg(RegOpmode::AUX, 0x54, 0x78).await;
        self.write_reg(RegOpmode::CONFIG, 0x1F, 0x10).await; // Color code, encryption disabled
        self.write_reg(RegOpmode::AUX, 0x24, 0x00).await;
        self.write_reg(RegOpmode::AUX, 0x25, 0x00).await;
        self.write_reg(RegOpmode::AUX, 0x26, 0x00).await;
        self.write_reg(RegOpmode::AUX, 0x27, 0x00).await;
        self.write_reg(RegOpmode::CONFIG, 0x41, 0x40).await; // Start RX for upcoming time slot interrupt
        self.write_reg(RegOpmode::CONFIG, 0x56, 0x00).await; // Undocumented register
        self.write_reg(RegOpmode::CONFIG, 0x41, 0x40).await; // Start RX for upcoming time slot interrupt
        self.write_reg(RegOpmode::CONFIG, 0x5C, 0x09).await; // Undocumented register
        self.write_reg(RegOpmode::CONFIG, 0x5F, 0xC0).await; // Detect BS and MS frame sequences in 2 layer mode
        self.write_seq(&INIT_SEQ7).await;
        self.write_reg(RegOpmode::CONFIG, 0x11, 0x80).await; // Local channel mode
        self.write_reg(RegOpmode::CONFIG, 0xE0, 0xC9).await; // Codec enabled, LineIn1, LineOut2, I2S slave mode
    }

    pub async fn enable_audio_out(&mut self) {
        self.write_reg(RegOpmode::CONFIG, 0x36, 0x02).await;
    }

    pub async fn set_audio_volume(&mut self, volume: u8) {
        let gain = (volume / 4) as i8 - 31;
        if gain <= -31 {
            self.write_reg(RegOpmode::CONFIG, 0xE2, 0x00).await;
            return;
        }

        let mut value = (gain as u8) & 0x1F;
        if gain > 0 {
            value |= 0x40;
        }

        self.write_reg(RegOpmode::CONFIG, 0x37, value).await; // DAC gain
        self.write_reg(RegOpmode::CONFIG, 0xE2, 0x02).await; // Enable DAC
    }

    async fn write_reg(&mut self, mode: RegOpmode, reg: u8, data: u8) {
        let delay_micros = 1;
        let delay_cycles = delay_micros * 168;
        self.cs.set_low().unwrap();
        self.send_recv(mode as u8).await;
        self.send_recv(reg).await;
        self.send_recv(data).await;
        self.cs.set_high().unwrap();
        cortex_m::asm::delay(delay_cycles);
    }

    async fn write_seq(&mut self, data: &[u8]) {
        let delay_micros = 5;
        let delay_cycles = delay_micros * 168;
        self.cs.set_low().unwrap();
        for b in data {
            self.send_recv(*b).await;
        }
        self.cs.set_high().unwrap();
        cortex_m::asm::delay(delay_cycles);
    }

    async fn send_recv(&mut self, value: u8) -> u8 {
        let delay_micros = 5;
        let delay_cycles = delay_micros * 168;
        self.clk.set_low().unwrap();
        let mut incoming = 0u8;
        for i in 0..8 {
            // Bitmask for this iteration, msb to lsb.
            let bitmask = 0x80 >> i;
            self.clk.set_high().unwrap();
            if value & bitmask != 0 {
                self.mosi.set_high().unwrap();
            } else {
                self.mosi.set_low().unwrap();
            }
            cortex_m::asm::delay(delay_cycles);
            self.clk.set_low().unwrap();
            if self.miso.is_high().unwrap() {
                incoming |= bitmask;
            }
            cortex_m::asm::delay(delay_cycles);
        }
        incoming
    }
}
