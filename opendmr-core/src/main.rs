#![feature(impl_trait_in_assoc_type)]
#![feature(type_alias_impl_trait)]
#![no_std]
#![no_main]

use core::{arch::asm, panic::PanicInfo};
use cortex_m_rt::entry;
use defmt_rtt as _;
use display_interface_parallel_gpio::Generic8BitBus;
use embassy_executor::Spawner;
use embassy_stm32::{
    gpio::{Level, Output, Speed},
    pac::RCC,
    rcc::{self, Hse, HseMode, PllSource},
    time::Hertz,
    Config, Peripherals,
};
use embassy_time::{Delay, Instant, Timer};
use embedded_graphics::Drawable;
use embedded_graphics::{
    draw_target::DrawTarget,
    geometry::Point,
    mono_font::{iso_8859_15::FONT_9X15_BOLD, MonoTextStyle},
    pixelcolor::{Rgb565, RgbColor},
    text::{Alignment, Text},
};

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    loop {
        unsafe {
            asm!(
                "LDR R1, =0x40021014;",
                "LDR R0, [R1];",
                "ORR.W R0, #0x0003;",
                "STR R0, [R1];",
            );
        }
    }
}

#[entry]
fn main() -> ! {
    RCC.cfgr().modify(|w| {
        w.set_sw(rcc::Sysclk::HSI);
    });
    while RCC.cfgr().read().sws() != rcc::Sysclk::HSI {}

    unsafe {
        asm!(
            "ldr SP, =0x20020000", // Relocate the stack, this is super dangerous so do it as early as possible.
            "b start_executor_post_relocate"
        );
    }
    // The above should never return, but if it does turn on both LEDs.
    unsafe {
        asm!(
            "LDR R1, =0x40021014;",
            "LDR R0, [R1];",
            "ORR.W R0, #0x0003;",
            "STR R0, [R1];",
        );
    }

    loop {}
}

#[no_mangle]
unsafe fn start_executor_post_relocate() -> ! {
    let mut config = Config::default();

    config.rcc.hse = Some(Hse {
        freq: Hertz(8_000_000),
        mode: HseMode::Oscillator,
    });
    config.rcc.pll_src = PllSource::HSE;
    config.rcc.pll = Some(rcc::Pll {
        prediv: rcc::PllPreDiv::DIV4,
        mul: rcc::PllMul::MUL168,
        divp: Some(rcc::PllPDiv::DIV2),
        divq: Some(rcc::PllQDiv::DIV7),
        divr: None,
    });
    config.rcc.ahb_pre = rcc::AHBPrescaler::DIV1;
    config.rcc.apb1_pre = rcc::APBPrescaler::DIV4;
    config.rcc.apb2_pre = rcc::APBPrescaler::DIV2;
    config.rcc.sys = rcc::Sysclk::PLL1_P;
    config.rcc.mux.clk48sel = rcc::mux::Clk48sel::PLL1_Q;

    let peripherals = embassy_stm32::init(config);

    let mut executor = ::embassy_executor::Executor::new();
    let executor = unsafe { __make_static(&mut executor) };

    executor.run(move |spawner| {
        spawner.must_spawn(main_post_relocate(spawner, peripherals));
    });
}

unsafe fn __make_static<T>(t: &mut T) -> &'static mut T {
    ::core::mem::transmute(t)
}

#[embassy_executor::task()]
async fn main_post_relocate(_spawner: Spawner, peripherals: Peripherals) {
    let mut led = Output::new(peripherals.PE1, Level::High, Speed::Low);

    let mut display = {
        let d0 = Output::new(peripherals.PD14, Level::High, Speed::Medium);
        let d1 = Output::new(peripherals.PD15, Level::High, Speed::Medium);
        let d2 = Output::new(peripherals.PD0, Level::High, Speed::Medium);
        let d3 = Output::new(peripherals.PD1, Level::High, Speed::Medium);
        let d4 = Output::new(peripherals.PE7, Level::High, Speed::Medium);
        let d5 = Output::new(peripherals.PE8, Level::High, Speed::Medium);
        let d6 = Output::new(peripherals.PE9, Level::High, Speed::Medium);
        let d7 = Output::new(peripherals.PE10, Level::High, Speed::Medium);

        let wr = Output::new(peripherals.PD5, Level::High, Speed::Medium);
        let dc = Output::new(peripherals.PD12, Level::High, Speed::Medium);
        let rst = Output::new(peripherals.PD13, Level::Low, Speed::Medium);

        let output_bus = Generic8BitBus::new((d0, d1, d2, d3, d4, d5, d6, d7)).unwrap();
        let parallel_bus =
            display_interface_parallel_gpio::PGPIO8BitInterface::new(output_bus, dc, wr);
        mipidsi::Builder::st7789(parallel_bus)
            .with_invert_colors(false)
            .with_display_size(128, 160)
            .with_framebuffer_size(128, 160)
            .with_orientation(mipidsi::Orientation::Landscape(true))
            .init(&mut Delay, Some(rst))
            .unwrap()
    };

    display.clear(Rgb565::WHITE).unwrap();
    let backlight = Output::new(peripherals.PD8, Level::High, Speed::Low);

    {
        let style = MonoTextStyle::new(&FONT_9X15_BOLD, Rgb565::BLACK);

        Text::with_alignment("Hello World.", Point::new(20, 30), style, Alignment::Left)
            .draw(&mut display)
            .unwrap();
    }

    let ticks = Instant::now().as_ticks();

    loop {
        if Instant::now().as_ticks() - ticks > 10_000 {
            led.set_high();
            Timer::after_millis(50).await;
            led.set_low();
            Timer::after_millis(50).await;
        } else {
            led.set_low();
        }
    }
}
