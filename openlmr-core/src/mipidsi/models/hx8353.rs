use display_interface::{DataFormat, WriteOnlyDataCommand};
use embedded_graphics_core::{pixelcolor::Rgb565, prelude::IntoStorage};

use crate::mipidsi::{
    dcs::{
        BitsPerPixel, Dcs, ExitSleepMode, PixelFormat, SetAddressMode, SetDisplayOn, SetInvertMode,
        SetPixelFormat, WriteMemoryStart,
    },
    error::InitError,
    Error, ModelOptions,
};
use stm32f4xx_hal::prelude::*;

pub struct HX8353;

impl HX8353 {
    pub async fn init<DI>(
        &mut self,
        dcs: &mut Dcs<'_, DI>,
        options: &ModelOptions,
        skip_init: bool,
    ) -> Result<SetAddressMode, InitError>
    where
        DI: WriteOnlyDataCommand,
    {
        let madctl = SetAddressMode::from(options);

        if !skip_init {
            crate::Mono::delay(150.millis().into()).await;

            dcs.write_command(ExitSleepMode)?; // turn off sleep
            crate::Mono::delay(120.millis().into()).await;

            dcs.write_command(SetInvertMode(options.invert_colors))?; // set color inversion
            dcs.write_raw(0xB1, &[0x05, 0x3A, 0x3A])?; // set frame rate
            dcs.write_raw(0xB2, &[0x05, 0x3A, 0x3A])?; // set frame rate
            dcs.write_raw(0xB3, &[0x05, 0x3A, 0x3A, 0x05, 0x3A, 0x3A])?; // set frame rate
            dcs.write_raw(0xB4, &[0b0000_0011])?; // set inversion control
            dcs.write_raw(0xC0, &[0x62, 0x02, 0x04])?; // set power control 1
            dcs.write_raw(0xC1, &[0xC0])?; // set power control 2
            dcs.write_raw(0xC2, &[0x0D, 0x00])?; // set power control 3
            dcs.write_raw(0xC3, &[0x8D, 0x6A])?; // set power control 4
            dcs.write_raw(0xC4, &[0x8D, 0xEE])?; // set power control 5
            dcs.write_raw(0xC5, &[0x0E])?; // set VCOM control 1
            dcs.write_raw(
                0xE0,
                &[
                    0x10, 0x0E, 0x02, 0x03, 0x0E, 0x07, 0x02, 0x07, 0x0A, 0x12, 0x27, 0x37, 0x00,
                    0x0D, 0x0E, 0x10,
                ],
            )?; // set GAMMA +Polarity characteristics
            dcs.write_raw(
                0xE1,
                &[
                    0x10, 0x0E, 0x03, 0x03, 0x0F, 0x06, 0x02, 0x08, 0x0A, 0x13, 0x26, 0x36, 0x00,
                    0x0D, 0x0E, 0x10,
                ],
            )?; // set GAMMA -Polarity characteristics

            let pf = PixelFormat::with_all(BitsPerPixel::from_rgb_color::<Rgb565>());
            dcs.write_command(SetPixelFormat::new(pf))?; // set interface pixel format, 16bit pixel into frame memory

            dcs.write_command(madctl)?; // set memory data access control, Top -> Bottom, RGB, Left -> Right
            dcs.write_command(SetDisplayOn)?; // turn on display
            crate::Mono::delay(120.millis().into()).await;
        }

        Ok(madctl)
    }

    pub fn write_pixels<DI, I>(&mut self, dcs: &mut Dcs<DI>, colors: I) -> Result<(), Error>
    where
        DI: WriteOnlyDataCommand,
        I: IntoIterator<Item = Rgb565>,
    {
        dcs.write_command(WriteMemoryStart)?;

        let mut iter = colors.into_iter().map(Rgb565::into_storage);

        let buf = DataFormat::U16BEIter(&mut iter);
        dcs.di.send_data(buf)?;
        Ok(())
    }

    pub(crate) fn default_options() -> ModelOptions {
        ModelOptions::with_sizes((128, 160), (128, 160))
    }
}
