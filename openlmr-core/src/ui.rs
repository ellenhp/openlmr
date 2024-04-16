use crate::{
    event::Event,
    gpio_display_iface::{Generic8BitBus, PGPIO8BitInterface},
    pubsub::{EVENT_CAP, EVENT_PUBS, EVENT_SUBS},
};
use embassy_stm32::{
    gpio::{Level, Output, Speed},
    peripherals::{PD0, PD1, PD12, PD13, PD14, PD15, PD5, PD8, PE10, PE7, PE8, PE9},
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, pubsub::Subscriber};
use embassy_time::{Delay, Timer};
use embedded_graphics::draw_target::DrawTarget;
use embedded_graphics::pixelcolor::{Rgb565, RgbColor};
use mipidsi::models::ST7789;

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

    wr: Output<'static>,
    dc: Output<'static>,
    rst: Output<'static>,
    backlight: Output<'static>,
}

impl UserInterface {
    async fn startup(&mut self) {
        let mut lcd_d0 = Output::new(&mut self.lcd_d0, Level::High, Speed::High);
        let mut lcd_d1 = Output::new(&mut self.lcd_d1, Level::High, Speed::High);
        let mut lcd_d2 = Output::new(&mut self.lcd_d2, Level::High, Speed::High);
        let mut lcd_d3 = Output::new(&mut self.lcd_d3, Level::High, Speed::High);
        let mut lcd_d4 = Output::new(&mut self.lcd_d4, Level::High, Speed::High);
        let mut lcd_d5 = Output::new(&mut self.lcd_d5, Level::High, Speed::High);
        let mut lcd_d6 = Output::new(&mut self.lcd_d6, Level::High, Speed::High);
        let mut lcd_d7 = Output::new(&mut self.lcd_d7, Level::High, Speed::High);

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

    pub async fn init(
        lcd_d0: PD14,
        lcd_d1: PD15,
        lcd_d2: PD0,
        lcd_d3: PD1,
        lcd_d4: PE7,
        lcd_d5: PE8,
        lcd_d6: PE9,
        lcd_d7: PE10,

        wr: PD5,
        dc: PD12,
        rst: PD13,
        backlight: PD8,
    ) -> UserInterface {
        let wr = Output::new(wr, Level::High, Speed::High);
        let dc = Output::new(dc, Level::High, Speed::High);
        let rst = Output::new(rst, Level::Low, Speed::High);
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
pub async fn process_ui(
    ui: UserInterface,
    mut state_update_sub: Subscriber<
        'static,
        CriticalSectionRawMutex,
        Event,
        EVENT_CAP,
        EVENT_SUBS,
        EVENT_PUBS,
    >,
) {
    loop {
        let _message = state_update_sub.next_message_pure().await;
    }
}
