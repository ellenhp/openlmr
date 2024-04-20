//! [super::Display] builder module

use display_interface::WriteOnlyDataCommand;

use crate::mipidsi::{
    dcs::Dcs, error::InitError, models::HX8353, ColorInversion, ColorOrder, Display, ModelOptions,
    Orientation, RefreshOrder,
};

/// Builder for [Display] instances.
///
/// Exposes all possible display options.
pub struct Builder<'a, DI>
where
    DI: WriteOnlyDataCommand,
{
    di: &'a mut DI,
    model: HX8353,
    options: ModelOptions,
}

impl<'a, DI> Builder<'a, DI>
where
    DI: WriteOnlyDataCommand,
{
    ///
    /// Constructs a new builder from given [WriteOnlyDataCommand], [Model]
    /// and [ModelOptions]. For use by [Model] helpers, not public
    ///
    pub(crate) fn new(di: &'a mut DI, model: HX8353, options: ModelOptions) -> Self {
        Self { di, model, options }
    }

    ///
    /// Constructs a new builder for given [Model] using the model's
    /// `default_options`
    ///
    pub fn with_model(di: &'a mut DI, model: HX8353) -> Self {
        Self {
            di,
            model,
            options: HX8353::default_options(),
        }
    }

    ///
    /// Sets the invert color flag
    ///
    pub fn with_invert_colors(mut self, color_inversion: ColorInversion) -> Self {
        self.options.invert_colors = color_inversion;
        self
    }

    ///
    /// Sets the [ColorOrder]
    ///
    pub fn with_color_order(mut self, color_order: ColorOrder) -> Self {
        self.options.color_order = color_order;
        self
    }

    ///
    /// Sets the [Orientation]
    ///
    pub fn with_orientation(mut self, orientation: Orientation) -> Self {
        self.options.orientation = orientation;
        self
    }

    ///
    /// Sets refresh order
    ///
    pub fn with_refresh_order(mut self, refresh_order: RefreshOrder) -> Self {
        self.options.refresh_order = refresh_order;
        self
    }

    ///
    /// Sets the display size
    ///
    pub fn with_display_size(mut self, width: u16, height: u16) -> Self {
        self.options.display_size = (width, height);
        self
    }

    ///
    /// Sets the framebuffer size
    ///
    pub fn with_framebuffer_size(mut self, width: u16, height: u16) -> Self {
        self.options.framebuffer_size = (width, height);
        self
    }

    ///
    /// Sets the window offset handler
    ///
    pub fn with_window_offset_handler(
        mut self,
        window_offset_handler: fn(_: &ModelOptions) -> (u16, u16),
    ) -> Self {
        self.options.window_offset_handler = window_offset_handler;
        self
    }

    ///
    /// Consumes the builder to create a new [Display] with an optional reset [OutputPin].
    /// Blocks using the provided [DelayUs] `delay_source` to perform the display initialization.
    /// ### WARNING
    /// The reset pin needs to be in *high* state in order for the display to operate.
    /// If it wasn't provided the user needs to ensure this is the case.
    ///
    pub async fn init(mut self, skip_init: bool) -> Result<Display<'a, DI>, InitError> {
        let mut dcs = Dcs::write_only(self.di);
        let madctl = self.model.init(&mut dcs, &self.options, skip_init).await?;
        let display = Display {
            dcs,
            model: self.model,
            options: self.options,
            madctl,
        };

        Ok(display)
    }
}
