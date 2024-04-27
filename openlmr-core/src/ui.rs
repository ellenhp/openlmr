use core::marker::PhantomData;

use crate::event::Event;
use crate::iface::DisplayInterface;
use crate::mipidsi::models::HX8353;
use crate::mipidsi::{self, Display};
use crate::pubsub::{EVENT_CAP, EVENT_PUBS, EVENT_SUBS};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::pubsub::{Publisher, Subscriber};
use embedded_graphics::pixelcolor::{Rgb565, RgbColor};
use embedded_graphics::{
    draw_target::DrawTarget,
    geometry::{Point, Size},
    pixelcolor::{raw::RawU16, IntoStorage},
    primitives::Rectangle,
};
use rtic_monotonics::Monotonic;
use slint::platform::WindowEvent;
use stm32f4xx_hal::gpio::alt::fsmc;
use stm32f4xx_hal::gpio::{PinPull, PinState, Pull, ReadPin};
use stm32f4xx_hal::{
    fsmc_lcd::{Lcd, SubBank2},
    gpio::{PA6, PD2, PD3},
    prelude::*,
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

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum KeyState {
    NewlyPressed,
    Held,
    NewlyReleased,
    NotPressed,
}

// Might make sense to make this generic eventually. Also needs a better name, but I can't figure out how else to describe a LCD+keypad combo.
pub struct UserInterface<'a> {
    kb1: &'a mut PA6,
    kb3: &'a mut PD3,
    kb2: &'a mut PD2,
    interface: &'a mut DisplayInterface,
    _phantom: &'a PhantomData<()>,
}

impl<'a> UserInterface<'a> {
    async fn startup(&mut self) {
        self.interface
            .with_interface_async(async |interface| {
                let builder = mipidsi::Builder::with_model(interface, HX8353)
                    .with_invert_colors(mipidsi::ColorInversion::Normal)
                    .with_orientation(mipidsi::Orientation::Portrait(true));

                let mut display = builder.init(false).await.unwrap();

                display.clear(Rgb565::BLUE).unwrap();
            })
            .await;
        crate::Mono::delay(rtic_monotonics::stm32::ExtU64::millis(20u64)).await;
    }
    pub async fn with_display<CB: FnOnce(&mut Display<Lcd<SubBank2, u8>>) -> ()>(
        &mut self,
        cb: CB,
    ) {
        self.interface
            .with_interface_async(async |interface| {
                let builder = mipidsi::Builder::with_model(interface, HX8353)
                    .with_invert_colors(mipidsi::ColorInversion::Normal)
                    .with_orientation(mipidsi::Orientation::Portrait(true));
                let mut display = builder.init(true).await.unwrap();
                cb(&mut display);
            })
            .await;
    }

    pub async fn init(
        kb1: &'a mut PA6,
        kb2: &'a mut PD2,
        kb3: &'a mut PD3,
        interface: &'a mut DisplayInterface,
    ) -> UserInterface<'a> {
        let mut ui = UserInterface {
            kb1,
            kb2,
            kb3,
            interface,
            _phantom: &PhantomData,
        };
        ui.startup().await;

        return ui;
    }

    pub async fn scan_keypad(&mut self, old_states: [KeyState; 18]) -> [KeyState; 18] {
        let states = self
            .interface
            .pins
            .lend_to_async(
                async |fsmc,
                       mut kb_d0: fsmc::D0,
                       mut kb_d1: fsmc::D1,
                       mut kb_d2: fsmc::D2,
                       mut kb_d3: fsmc::D3,
                       mut kb_d4: fsmc::D4,
                       mut kb_d5: fsmc::D5,
                       mut kb_d6: fsmc::D6,
                       mut kb_d7: fsmc::D7,
                       address,
                       read_enable,
                       write_enable,
                       chip_select| {
                    kb_d0.set_internal_resistor(Pull::Down);
                    kb_d1.set_internal_resistor(Pull::Down);
                    kb_d2.set_internal_resistor(Pull::Down);
                    kb_d3.set_internal_resistor(Pull::Down);
                    kb_d4.set_internal_resistor(Pull::Down);
                    kb_d5.set_internal_resistor(Pull::Down);
                    kb_d6.set_internal_resistor(Pull::Down);
                    kb_d7.set_internal_resistor(Pull::Down);
                    let mut states = [false; 18];
                    {
                        self.kb1
                            .with_push_pull_output_in_state(PinState::High, |_| {
                                cortex_m::asm::delay(2000);
                                if kb_d7.is_high() {
                                    states[KeypadKey::Star as usize] = true;
                                }
                                if kb_d6.is_high() {
                                    states[KeypadKey::Num0 as usize] = true;
                                }
                                if kb_d5.is_high() {
                                    states[KeypadKey::Num6 as usize] = true;
                                }
                                if kb_d4.is_high() {
                                    states[KeypadKey::Num5 as usize] = true;
                                }
                                if kb_d3.is_high() {
                                    states[KeypadKey::Num4 as usize] = true;
                                }
                                if kb_d2.is_high() {
                                    states[KeypadKey::Num3 as usize] = true;
                                }
                                if kb_d1.is_high() {
                                    states[KeypadKey::Num2 as usize] = true;
                                }
                                if kb_d0.is_high() {
                                    states[KeypadKey::Num1 as usize] = true;
                                }
                            });
                        cortex_m::asm::delay(2000);
                        self.kb2
                            .with_push_pull_output_in_state(PinState::High, |_| {
                                cortex_m::asm::delay(2000);
                                if kb_d7.is_high() {
                                    states[KeypadKey::Back as usize] = true;
                                }
                                if kb_d2.is_high() {
                                    states[KeypadKey::Down as usize] = true;
                                }
                                if kb_d1.is_high() {
                                    states[KeypadKey::Up as usize] = true;
                                }
                                if kb_d0.is_high() {
                                    states[KeypadKey::Select as usize] = true;
                                }
                                if kb_d6.is_high() {
                                    states[KeypadKey::Pound as usize] = true;
                                }
                                if kb_d5.is_high() {
                                    states[KeypadKey::Num9 as usize] = true;
                                }
                                if kb_d4.is_high() {
                                    states[KeypadKey::Num8 as usize] = true;
                                }
                                if kb_d3.is_high() {
                                    states[KeypadKey::Num7 as usize] = true;
                                }
                            });
                        cortex_m::asm::delay(2000);
                        self.kb3
                            .with_push_pull_output_in_state(PinState::High, |_| {
                                cortex_m::asm::delay(2000);
                                if kb_d6.is_high() {
                                    states[KeypadKey::Moni as usize] = true;
                                }
                                if kb_d7.is_high() {
                                    states[KeypadKey::Function as usize] = true;
                                }
                            });
                        cortex_m::asm::delay(2000);
                    }
                    kb_d0.set_internal_resistor(Pull::None);
                    kb_d1.set_internal_resistor(Pull::None);
                    kb_d2.set_internal_resistor(Pull::None);
                    kb_d3.set_internal_resistor(Pull::None);
                    kb_d4.set_internal_resistor(Pull::None);
                    kb_d5.set_internal_resistor(Pull::None);
                    kb_d6.set_internal_resistor(Pull::None);
                    kb_d7.set_internal_resistor(Pull::None);

                    (
                        fsmc,
                        kb_d0,
                        kb_d1,
                        kb_d2,
                        kb_d3,
                        kb_d4,
                        kb_d5,
                        kb_d6,
                        kb_d7,
                        address,
                        read_enable,
                        write_enable,
                        chip_select,
                        states,
                    )
                },
            )
            .await;
        let mut new_states = [KeyState::NotPressed; 18];
        for i in 0..18 {
            let was_pressed = match old_states[i] {
                KeyState::NewlyPressed => true,
                KeyState::Held => true,
                KeyState::NewlyReleased => false,
                KeyState::NotPressed => false,
            };
            new_states[i] = match (states[i], was_pressed) {
                (true, true) => KeyState::Held,
                (true, false) => KeyState::NewlyPressed,
                (false, true) => KeyState::NewlyReleased,
                (false, false) => KeyState::NotPressed,
            };
        }
        new_states
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
                    Point::new(line as _, range.start as _),
                    Size::new(1, range.len() as _),
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
    window: alloc::rc::Rc<slint::platform::software_renderer::MinimalSoftwareWindow>,
}

impl slint::platform::Platform for Stm32Platform {
    fn create_window_adapter(
        &self,
    ) -> Result<alloc::rc::Rc<dyn slint::platform::WindowAdapter>, slint::PlatformError> {
        Ok(self.window.clone())
    }
    fn duration_since_start(&self) -> core::time::Duration {
        core::time::Duration::from_micros(crate::Mono::now().duration_since_epoch().to_micros())
    }
    fn run_event_loop(&self) -> Result<(), slint::PlatformError> {
        slint::platform::update_timers_and_animations();

        Ok(())
    }
}

pub async fn process_ui(
    ui: &'_ mut UserInterface<'_>,
    lcd_subscriber: &'_ mut Subscriber<
        '_,
        CriticalSectionRawMutex,
        Event,
        EVENT_CAP,
        EVENT_SUBS,
        EVENT_PUBS,
    >,
    lcd_publisher: &'_ mut Publisher<
        '_,
        CriticalSectionRawMutex,
        Event,
        EVENT_CAP,
        EVENT_SUBS,
        EVENT_PUBS,
    >,
) {
    let window = slint::platform::software_renderer::MinimalSoftwareWindow::new(
        slint::platform::software_renderer::RepaintBufferType::ReusedBuffer,
    );
    window.dispatch_event(WindowEvent::ScaleFactorChanged {
        scale_factor: 0.5f32,
    });
    window.set_size(slint::PhysicalSize::new(160, 128));
    slint::platform::set_platform(alloc::boxed::Box::new(Stm32Platform {
        window: window.clone(),
    }))
    .unwrap();

    let app = AppWindow::new().unwrap();
    let mut old_states = [KeyState::NotPressed; 18];
    loop {
        {
            let states = ui.scan_keypad(old_states).await;
            old_states = states;

            if states[KeypadKey::Moni as usize] == KeyState::NewlyPressed {
                lcd_publisher.publish_immediate(Event::MoniOn)
            } else if states[KeypadKey::Moni as usize] == KeyState::NewlyReleased {
                lcd_publisher.publish_immediate(Event::MoniOff)
            }
        }
        while lcd_subscriber.available() > 0 {
            let message = lcd_subscriber.next_message_pure().await;
            match message {
                Event::PttOn => {}
                Event::PttOff => {}
                Event::MoniOn => {}
                Event::MoniOff => {}
                Event::NewRSSI(rssi) => app.set_rssi(rssi as i32),
                Event::RedLed(_) => {}
                Event::GreenLed(_) => {}
                Event::TriggerRedraw => {}
                Event::TuneFreq(_) => {
                    // We want a past-tense tuned event, not a tune command.
                }
            }
        }

        app.run().unwrap();
        let mut line_buffer =
            [slint::platform::software_renderer::Rgb565Pixel(Rgb565::BLACK.into_storage()); 320];
        ui.with_display(|display| {
            window.draw_if_needed(|renderer| {
                renderer.render_by_line(DisplayWrapper {
                    display,
                    line_buffer: &mut line_buffer,
                });
            });
        })
        .await;

        crate::Mono::delay(25.millis().into()).await;
    }
}
