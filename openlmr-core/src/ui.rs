use core::{arch::asm, cell::RefCell};

use crate::{event::Event, iface::new_fsmc};
use alloc::{rc::Rc, string::ToString};
use cortex_m::interrupt::CriticalSection;
use embedded_graphics::pixelcolor::{Rgb565, RgbColor, WebColors};
use embedded_graphics::{
    draw_target::DrawTarget,
    geometry::{Point, Size},
    pixelcolor::{raw::RawU16, IntoStorage},
    primitives::Rectangle,
};
use embedded_hal::{blocking::delay::DelayUs, digital::v2::OutputPin};
use mipidsi::{models::ST7789, Display};
use rtic_monotonics::stm32::fugit::Instant;
use rtic_monotonics::stm32::Tim2;
use slint::platform::{Key, WindowEvent};
use stm32f4xx_hal::hal::delay::DelayNs;
use stm32f4xx_hal::timer::SysDelay;
use stm32f4xx_hal::{
    fsmc_lcd::{Lcd, SubBank1},
    gpio::{Output, Pin, PinState, PA6, PD13, PD2, PD3, PD8},
};

enum KeypadKey {
    Select = 0,
    Up = 1,
    Down = 2,
    Back = 3,
    Num1 = 4,
    Num2 = 5,
    Num3 = 6,
    Num4 = 7,
    Num5 = 8,
    Num6 = 9,
    Num7 = 10,
    Num8 = 11,
    Num9 = 12,
    Num0 = 13,
    Star = 14,
    Pound = 15,
    Function = 16,
    Moni = 17,
}

// Might make sense to make this generic eventually. Also needs a better name, but I can't figure out how else to describe a LCD+keypad combo.
pub struct UserInterface<BACKLIGHT: OutputPin> {
    rst: PD13,
    backlight: BACKLIGHT,
    kb1: PA6,
    kb3: PD3,
    kb2: PD2,
}

impl<BACKLIGHT: OutputPin> UserInterface<BACKLIGHT> {
    async fn startup(&mut self) {
        self.backlight.set_high().map_err(|_| "").unwrap();
        let mut interface = new_fsmc();
        self.rst.with_push_pull_output(|rst| {
            let builder = mipidsi::Builder::with_model(&mut interface, ST7789)
                .with_invert_colors(mipidsi::ColorInversion::Normal)
                .with_display_size(128, 160)
                .with_framebuffer_size(128, 160)
                .with_orientation(mipidsi::Orientation::Landscape(true));

            Tim2.delay_ms(150_000);

            let mut display = builder.init(Some(rst), false).unwrap();

            display.clear(Rgb565::BLACK).unwrap();
        });
        crate::Mono::delay(rtic_monotonics::stm32::ExtU64::millis(20u64)).await;
    }
    pub fn with_display<
        CB: FnOnce(&mut Display<Lcd<SubBank1>, ST7789, Pin<'D', 13, Output>>) -> (),
    >(
        &mut self,
        cb: CB,
    ) {
        let mut interface = new_fsmc();
        self.rst.with_push_pull_output(|rst| {
            let builder = mipidsi::Builder::with_model(&mut interface, ST7789)
                .with_invert_colors(mipidsi::ColorInversion::Normal)
                .with_display_size(128, 160)
                .with_framebuffer_size(128, 160)
                .with_orientation(mipidsi::Orientation::Landscape(true));
            let mut display = builder.init(Some(rst), true).unwrap();

            critical_section::with(|_| cb(&mut display));
        });
    }

    pub async fn init(
        rst: PD13,
        backlight: PD8,
        kb1: PA6,
        kb2: PD2,
        kb3: PD3,
    ) -> UserInterface<Pin<'D', 8, Output>> {
        let backlight = backlight.into_push_pull_output_in_state(PinState::High);
        let mut ui = UserInterface {
            rst,
            backlight,
            kb1,
            kb2,
            kb3,
        };
        ui.startup().await;
        return ui;
    }

    pub async fn scan_keypad(&mut self) -> [bool; 18] {
        let mut states = [false; 18];
        // let kb_d0 = Input::new(&mut self.lcd_d0, Pull::Down);
        // let kb_d1 = Input::new(&mut self.lcd_d1, Pull::Down);
        // let kb_d2 = Input::new(&mut self.lcd_d2, Pull::Down);
        // let kb_d3 = Input::new(&mut self.lcd_d3, Pull::Down);
        // let kb_d4 = Input::new(&mut self.lcd_d4, Pull::Down);
        // let kb_d5 = Input::new(&mut self.lcd_d5, Pull::Down);
        // let kb_d6 = Input::new(&mut self.lcd_d6, Pull::Down);
        // let kb_d7 = Input::new(&mut self.lcd_d7, Pull::Down);

        // let kb1 = Output::new(&mut self.kb1, Level::High, Speed::VeryHigh);
        // Timer::after_micros(10).await;

        // if kb_d7.is_high() {
        //     states[KeypadKey::Star as usize] = true;
        // }
        // if kb_d6.is_high() {
        //     states[KeypadKey::Num0 as usize] = true;
        // }
        // if kb_d5.is_high() {
        //     states[KeypadKey::Num6 as usize] = true;
        // }
        // if kb_d4.is_high() {
        //     states[KeypadKey::Num5 as usize] = true;
        // }
        // if kb_d3.is_high() {
        //     states[KeypadKey::Num4 as usize] = true;
        // }
        // if kb_d2.is_high() {
        //     states[KeypadKey::Num3 as usize] = true;
        // }
        // if kb_d1.is_high() {
        //     states[KeypadKey::Num2 as usize] = true;
        // }
        // if kb_d0.is_high() {
        //     states[KeypadKey::Num1 as usize] = true;
        // }

        // drop(kb1);
        // let kb2 = Output::new(&mut self.kb2, Level::High, Speed::VeryHigh);
        // Timer::after_micros(10).await;

        // if kb_d7.is_high() {
        //     states[KeypadKey::Back as usize] = true;
        // }
        // if kb_d2.is_high() {
        //     states[KeypadKey::Down as usize] = true;
        // }
        // if kb_d1.is_high() {
        //     states[KeypadKey::Up as usize] = true;
        // }
        // if kb_d0.is_high() {
        //     states[KeypadKey::Select as usize] = true;
        // }
        // if kb_d6.is_high() {
        //     states[KeypadKey::Pound as usize] = true;
        // }
        // if kb_d5.is_high() {
        //     states[KeypadKey::Num9 as usize] = true;
        // }
        // if kb_d4.is_high() {
        //     states[KeypadKey::Num8 as usize] = true;
        // }
        // if kb_d3.is_high() {
        //     states[KeypadKey::Num7 as usize] = true;
        // }

        // drop(kb2);
        // let kb3 = Output::new(&mut self.kb3, Level::High, Speed::VeryHigh);
        // Timer::after_micros(10).await;

        // if kb_d6.is_high() {
        //     states[KeypadKey::Moni as usize] = true;
        // }
        // if kb_d7.is_high() {
        //     states[KeypadKey::Function as usize] = true;
        // }

        // drop(kb3);
        // Timer::after_micros(10).await;

        states
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
struct Stm32Platform<BACKLIGHT: OutputPin> {
    ui: Rc<RefCell<UserInterface<BACKLIGHT>>>,
    window: alloc::rc::Rc<slint::platform::software_renderer::MinimalSoftwareWindow>,
}

impl<BACKLIGHT: OutputPin> slint::platform::Platform for Stm32Platform<BACKLIGHT> {
    fn create_window_adapter(
        &self,
    ) -> Result<alloc::rc::Rc<dyn slint::platform::WindowAdapter>, slint::PlatformError> {
        Ok(self.window.clone())
    }
    fn duration_since_start(&self) -> core::time::Duration {
        core::time::Duration::from_micros(0)
    }
    fn run_event_loop(&self) -> Result<(), slint::PlatformError> {
        slint::platform::update_timers_and_animations();
        let ui = self.ui.clone();
        let mut line_buffer =
            [slint::platform::software_renderer::Rgb565Pixel(Rgb565::BLACK.into_storage()); 160];
        self.window.draw_if_needed(|renderer| {
            ui.borrow_mut().with_display(|display| {
                display.clear(Rgb565::CSS_PURPLE).unwrap();
                renderer.render_by_line(DisplayWrapper {
                    display,
                    line_buffer: &mut line_buffer,
                });
            });
        });
        Ok(())
    }
}

pub async fn process_ui(rst: PD13, backlight: PD8, kb1: PA6, kb2: PD2, kb3: PD3) {
    let mut rssi = -137i16;

    let user_interface =
        UserInterface::<Pin<'D', 8, Output>>::init(rst, backlight, kb1, kb2, kb3).await;
    unsafe {
        asm!(
            "LDR R1, =0x40021014;",
            "LDR R0, [R1];",
            "ORR.W R0, #0x0003;",
            "STR R0, [R1];",
        );
    }

    let window = slint::platform::software_renderer::MinimalSoftwareWindow::new(
        slint::platform::software_renderer::RepaintBufferType::ReusedBuffer,
    );
    window.dispatch_event(WindowEvent::ScaleFactorChanged {
        scale_factor: 0.5f32,
    });
    window.set_size(slint::PhysicalSize::new(160, 128));
    let ui = Rc::new(RefCell::from(user_interface));
    slint::platform::set_platform(alloc::boxed::Box::new(Stm32Platform {
        ui: ui.clone(),
        window: window.clone(),
    }))
    .unwrap();

    let mut prev_states = [false; 18];

    let app = AppWindow::new().unwrap();
    loop {
        let states = ui.borrow_mut().scan_keypad().await;
        if states[KeypadKey::Select as usize] && !prev_states[KeypadKey::Select as usize] {
            window.dispatch_event(WindowEvent::KeyPressed {
                text: Key::Return.into(),
            })
        }
        if states[KeypadKey::Up as usize] && !prev_states[KeypadKey::Up as usize] {
            window.dispatch_event(WindowEvent::KeyPressed {
                text: Key::UpArrow.into(),
            })
        }
        if states[KeypadKey::Down as usize] && !prev_states[KeypadKey::Down as usize] {
            window.dispatch_event(WindowEvent::KeyPressed {
                text: Key::DownArrow.into(),
            })
        }
        if states[KeypadKey::Back as usize] && !prev_states[KeypadKey::Back as usize] {
            window.dispatch_event(WindowEvent::KeyPressed {
                text: Key::Backspace.into(),
            })
        }
        if states[KeypadKey::Num1 as usize] && !prev_states[KeypadKey::Num1 as usize] {
            window.dispatch_event(WindowEvent::KeyPressed { text: "1".into() })
        }
        if states[KeypadKey::Num2 as usize] && !prev_states[KeypadKey::Num2 as usize] {
            window.dispatch_event(WindowEvent::KeyPressed { text: "2".into() })
        }
        if states[KeypadKey::Num3 as usize] && !prev_states[KeypadKey::Num3 as usize] {
            window.dispatch_event(WindowEvent::KeyPressed { text: "3".into() })
        }
        if states[KeypadKey::Num4 as usize] && !prev_states[KeypadKey::Num4 as usize] {
            window.dispatch_event(WindowEvent::KeyPressed { text: "4".into() })
        }
        if states[KeypadKey::Num5 as usize] && !prev_states[KeypadKey::Num5 as usize] {
            window.dispatch_event(WindowEvent::KeyPressed { text: "5".into() })
        }
        if states[KeypadKey::Num6 as usize] && !prev_states[KeypadKey::Num6 as usize] {
            window.dispatch_event(WindowEvent::KeyPressed { text: "6".into() })
        }
        if states[KeypadKey::Num7 as usize] && !prev_states[KeypadKey::Num7 as usize] {
            window.dispatch_event(WindowEvent::KeyPressed { text: "7".into() })
        }
        if states[KeypadKey::Num8 as usize] && !prev_states[KeypadKey::Num8 as usize] {
            window.dispatch_event(WindowEvent::KeyPressed { text: "8".into() })
        }
        if states[KeypadKey::Num9 as usize] && !prev_states[KeypadKey::Num9 as usize] {
            window.dispatch_event(WindowEvent::KeyPressed { text: "9".into() })
        }
        if states[KeypadKey::Num0 as usize] && !prev_states[KeypadKey::Num0 as usize] {
            window.dispatch_event(WindowEvent::KeyPressed { text: "0".into() })
        }
        if states[KeypadKey::Star as usize] && !prev_states[KeypadKey::Star as usize] {
            window.dispatch_event(WindowEvent::KeyPressed { text: "*".into() })
        }
        if states[KeypadKey::Pound as usize] && !prev_states[KeypadKey::Pound as usize] {
            window.dispatch_event(WindowEvent::KeyPressed { text: "#".into() })
        }
        if states[KeypadKey::Function as usize] && !prev_states[KeypadKey::Function as usize] {}
        // if states[KeypadKey::Moni as usize] && !prev_states[KeypadKey::Moni as usize] {
        //     state_update_pub.publish_immediate(Event::MoniOn);
        // } else if !states[KeypadKey::Moni as usize] && prev_states[KeypadKey::Moni as usize] {
        //     state_update_pub.publish_immediate(Event::MoniOff);
        // }
        prev_states = states;

        app.set_rssi(rssi as i32);
        app.run().unwrap();
    }
}
