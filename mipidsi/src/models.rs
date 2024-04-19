//! Display models.

use crate::{
    dcs::{Dcs, SetAddressMode},
    error::InitError,
    Error, ModelOptions,
};
use display_interface::WriteOnlyDataCommand;
use embedded_graphics_core::prelude::RgbColor;
use embedded_hal::{blocking::delay::DelayUs, digital::v2::OutputPin};

mod st7789;

use rtic_monotonics::stm32::Tim2;
pub use st7789::*;
use stm32f4xx_hal::hal::delay::DelayNs;

/// Display model.
pub trait Model {
    /// The color format.
    type ColorFormat: RgbColor;

    /// Initializes the display for this model with MADCTL from [crate::Display]
    /// and returns the value of MADCTL set by init
    fn init<RST, DI>(
        &mut self,
        dcs: &mut Dcs<DI>,
        options: &ModelOptions,
        rst: &mut Option<&mut RST>,
        _skip_init: bool,
    ) -> Result<SetAddressMode, InitError<RST::Error>>
    where
        RST: OutputPin,
        DI: WriteOnlyDataCommand;

    /// Resets the display using a reset pin.
    fn hard_reset<RST>(&mut self, rst: &mut RST) -> Result<(), InitError<RST::Error>>
    where
        RST: OutputPin,
    {
        rst.set_low().map_err(InitError::Pin)?;
        Tim2.delay_us(10);
        rst.set_high().map_err(InitError::Pin)?;

        Ok(())
    }

    /// Writes pixels to the display IC via the given display interface.
    ///
    /// Any pixel color format conversion is done here.
    fn write_pixels<DI, I>(&mut self, di: &mut Dcs<DI>, colors: I) -> Result<(), Error>
    where
        DI: WriteOnlyDataCommand,
        I: IntoIterator<Item = Self::ColorFormat>;

    /// Creates default [ModelOptions] for this particular [Model].
    ///
    /// This serves as a "sane default". There can be additional variants which will be provided via
    /// helper constructors.
    fn default_options() -> ModelOptions;
}
