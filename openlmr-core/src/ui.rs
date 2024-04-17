use core::cell::RefCell;

use crate::{
    event::Event,
    gpio_display_iface::{Generic8BitBus, PGPIO8BitInterface},
    pubsub::{EVENT_CAP, EVENT_PUBS, EVENT_SUBS},
};
use alloc::{boxed::Box, format, rc::Rc};
use display_interface::DisplayError;
use embassy_stm32::{
    gpio::{Level, Output, Speed},
    peripherals::{PD0, PD1, PD12, PD13, PD14, PD15, PD5, PD6, PD8, PE10, PE7, PE8, PE9},
};
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex,
    pubsub::{Publisher, Subscriber},
};
use embassy_time::{Delay, Instant, Timer};
use embedded_graphics::{
    draw_target::DrawTarget,
    geometry::{OriginDimensions, Point, Size},
    mono_font::{
        self,
        ascii::{self, FONT_10X20, FONT_8X13_BOLD},
    },
    pixelcolor::{raw::RawU16, IntoStorage, WebColors},
    primitives::Rectangle,
    text::{Alignment, Text},
};
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyle},
    pixelcolor::{Rgb565, RgbColor},
    Drawable,
};
use mipidsi::{models::ST7789, Display};
use slint::{
    platform::{Key, WindowEvent},
    SharedString,
};

// Might make sense to make this generic eventually. Also needs a better name, but I can't figure out how else to describe a LCD+keypad combo.
pub struct UserInterface {
    lcd_d0: PD14,
    lcd_d1: PD15,
    lcd_d2: PD0,
    lcd_d3: PD1,
    lcd_d4: PE7,
    lcd_d5: PE8,
    lcd_d6: PE9,
    lcd_d7: PE10,

    cs: Output<'static>,
    wr: Output<'static>,
    dc: Output<'static>,
    rst: Output<'static>,
    backlight: Output<'static>,
}

impl UserInterface {
    async fn startup(&mut self) {
        let mut lcd_d0 = Output::new(&mut self.lcd_d0, Level::High, Speed::Low);
        let mut lcd_d1 = Output::new(&mut self.lcd_d1, Level::High, Speed::Low);
        let mut lcd_d2 = Output::new(&mut self.lcd_d2, Level::High, Speed::Low);
        let mut lcd_d3 = Output::new(&mut self.lcd_d3, Level::High, Speed::Low);
        let mut lcd_d4 = Output::new(&mut self.lcd_d4, Level::High, Speed::Low);
        let mut lcd_d5 = Output::new(&mut self.lcd_d5, Level::High, Speed::Low);
        let mut lcd_d6 = Output::new(&mut self.lcd_d6, Level::High, Speed::Low);
        let mut lcd_d7 = Output::new(&mut self.lcd_d7, Level::High, Speed::Low);

        let mut output_bus = Generic8BitBus::new((
            &mut lcd_d0,
            &mut lcd_d1,
            &mut lcd_d2,
            &mut lcd_d3,
            &mut lcd_d4,
            &mut lcd_d5,
            &mut lcd_d6,
            &mut lcd_d7,
        ))
        .unwrap();

        let mut parallel_bus =
            { PGPIO8BitInterface::new(&mut output_bus, &mut self.dc, &mut self.wr) };

        let mut display = mipidsi::Builder::with_model(&mut parallel_bus, ST7789)
            .with_invert_colors(mipidsi::ColorInversion::Normal)
            .with_display_size(128, 160)
            .with_framebuffer_size(128, 160)
            .with_orientation(mipidsi::Orientation::Landscape(true))
            .init(&mut Delay, Some(&mut self.rst), false)
            .unwrap();
        display.clear(Rgb565::BLACK).unwrap();
        Timer::after_millis(20).await;
        self.backlight.set_high();
    }

    pub fn with_display<
        CB: FnOnce(
            &mut Display<
                '_,
                PGPIO8BitInterface<
                    '_,
                    Generic8BitBus<
                        '_,
                        Output<'_>,
                        Output<'_>,
                        Output<'_>,
                        Output<'_>,
                        Output<'_>,
                        Output<'_>,
                        Output<'_>,
                        Output<'_>,
                    >,
                    Output<'_>,
                    Output<'_>,
                >,
                ST7789,
                Output<'_>,
            >,
        ) -> (),
    >(
        &mut self,
        cb: CB,
    ) {
        let mut lcd_d0 = Output::new(&mut self.lcd_d0, Level::High, Speed::Low);
        let mut lcd_d1 = Output::new(&mut self.lcd_d1, Level::High, Speed::Low);
        let mut lcd_d2 = Output::new(&mut self.lcd_d2, Level::High, Speed::Low);
        let mut lcd_d3 = Output::new(&mut self.lcd_d3, Level::High, Speed::Low);
        let mut lcd_d4 = Output::new(&mut self.lcd_d4, Level::High, Speed::Low);
        let mut lcd_d5 = Output::new(&mut self.lcd_d5, Level::High, Speed::Low);
        let mut lcd_d6 = Output::new(&mut self.lcd_d6, Level::High, Speed::Low);
        let mut lcd_d7 = Output::new(&mut self.lcd_d7, Level::High, Speed::Low);

        let mut output_bus = Generic8BitBus::new((
            &mut lcd_d0,
            &mut lcd_d1,
            &mut lcd_d2,
            &mut lcd_d3,
            &mut lcd_d4,
            &mut lcd_d5,
            &mut lcd_d6,
            &mut lcd_d7,
        ))
        .unwrap();

        let mut parallel_bus =
            { PGPIO8BitInterface::new(&mut output_bus, &mut self.dc, &mut self.wr) };

        let mut display = mipidsi::Builder::with_model(&mut parallel_bus, ST7789)
            .with_invert_colors(mipidsi::ColorInversion::Inverted)
            .with_display_size(128, 160)
            .with_framebuffer_size(128, 160)
            .with_orientation(mipidsi::Orientation::Landscape(true))
            .init(&mut Delay, Some(&mut self.rst), true)
            .unwrap();

        cb(&mut display);
    }

    pub async fn init(
        lcd_d0: PD14,
        lcd_d1: PD15,
        lcd_d2: PD0,
        lcd_d3: PD1,
        lcd_d4: PE7,
        lcd_d5: PE8,
        lcd_d6: PE9,
        lcd_d7: PE10,

        cs: PD6,
        wr: PD5,
        dc: PD12,
        rst: PD13,
        backlight: PD8,
    ) -> UserInterface {
        let cs = Output::new(cs, Level::Low, Speed::Low);
        let wr = Output::new(wr, Level::High, Speed::Low);
        let dc = Output::new(dc, Level::High, Speed::Low);
        let rst = Output::new(rst, Level::High, Speed::Low);
        let backlight = Output::new(backlight, Level::Low, Speed::Low);
        let mut ui = UserInterface {
            lcd_d0,
            lcd_d1,
            lcd_d2,
            lcd_d3,
            lcd_d4,
            lcd_d5,
            lcd_d6,
            lcd_d7,
            cs,
            wr,
            dc,
            rst,
            backlight,
        };
        ui.startup().await;
        return ui;
    }
}

#[embassy_executor::task()]
pub async fn trigger_redraw_task(
    state_update_pub: Publisher<
        'static,
        CriticalSectionRawMutex,
        Event,
        EVENT_CAP,
        EVENT_SUBS,
        EVENT_PUBS,
    >,
) {
    loop {
        state_update_pub.publish_immediate(Event::TriggerRedraw);
        Timer::after_millis(20).await;
    }
}

slint::slint! {}

struct DisplayWrapper<'a, T> {
    display: &'a mut T,
    line_buffer: &'a mut [slint::platform::software_renderer::Rgb565Pixel],
}
impl<T: DrawTarget<Color = Rgb565>> slint::platform::software_renderer::LineBufferProvider
    for DisplayWrapper<'_, T>
{
    type TargetPixel = slint::platform::software_renderer::Rgb565Pixel;
    fn process_line(
        &mut self,
        line: usize,
        range: core::ops::Range<usize>,
        render_fn: impl FnOnce(&mut [Self::TargetPixel]),
    ) {
        // Render into the line
        render_fn(&mut self.line_buffer[range.clone()]);

        // Send the line to the screen using DrawTarget::fill_contiguous
        self.display
            .fill_contiguous(
                &Rectangle::new(
                    Point::new(range.start as _, line as _),
                    Size::new(range.len() as _, 1),
                ),
                self.line_buffer[range.clone()]
                    .iter()
                    .map(|p| RawU16::new(p.0).into()),
            )
            .map_err(drop)
            .unwrap();
    }
}

slint::include_modules!();
struct Stm32Platform {
    ui: Rc<RefCell<UserInterface>>,
    window: alloc::rc::Rc<slint::platform::software_renderer::MinimalSoftwareWindow>,
}

impl slint::platform::Platform for Stm32Platform {
    fn create_window_adapter(
        &self,
    ) -> Result<alloc::rc::Rc<dyn slint::platform::WindowAdapter>, slint::PlatformError> {
        Ok(self.window.clone())
    }
    fn duration_since_start(&self) -> core::time::Duration {
        core::time::Duration::from_micros(Instant::now().as_micros())
    }
    fn run_event_loop(&self) -> Result<(), slint::PlatformError> {
        slint::platform::update_timers_and_animations();
        let ui = self.ui.clone();
        let mut line_buffer =
            [slint::platform::software_renderer::Rgb565Pixel(Rgb565::BLACK.into_storage()); 160];
        self.window.draw_if_needed(|renderer| {
            ui.borrow_mut().with_display(|display| {
                renderer.render_by_line(DisplayWrapper {
                    display,
                    line_buffer: &mut line_buffer,
                });
            });
        });
        Ok(())
    }
}

#[embassy_executor::task()]
pub async fn process_ui(
    mut ui: UserInterface,
    mut state_update_sub: Subscriber<
        'static,
        CriticalSectionRawMutex,
        Event,
        EVENT_CAP,
        EVENT_SUBS,
        EVENT_PUBS,
    >,
) {
    let mut rssi = -137i16;
    ui.with_display(|display| {
        display.clear(Rgb565::BLACK).unwrap();
    });
    let window = slint::platform::software_renderer::MinimalSoftwareWindow::new(
        slint::platform::software_renderer::RepaintBufferType::ReusedBuffer,
    );
    window.set_size(slint::PhysicalSize::new(160, 128));
    slint::platform::set_platform(alloc::boxed::Box::new(Stm32Platform {
        ui: Rc::new(RefCell::from(ui)),
        window: window.clone(),
    }))
    .unwrap();

    let hello_world = AppWindow::new().unwrap();
    loop {
        while state_update_sub.available() != 0 {
            match state_update_sub.next_message_pure().await {
                Event::PttOn => window.dispatch_event(WindowEvent::KeyPressed {
                    text: Key::DownArrow.into(),
                }),
                // Event::PttOff => todo!(),
                // Event::NewRSSI(_) => todo!(),
                // Event::RedLed(_) => todo!(),
                // Event::GreenLed(_) => todo!(),
                // Event::TriggerRedraw => todo!(),
                // Event::TuneFreq(_) => todo!(),
                _ => {}
            }
        }
        hello_world.run().unwrap();
        Timer::after_millis(25).await;
    }
}
