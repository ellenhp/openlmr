#![feature(impl_trait_in_assoc_type)]
#![feature(type_alias_impl_trait)]
#![no_std]
#![no_main]

mod at1846s;
mod event;
mod gpio_display_iface;
mod led;
mod ptt;
mod pubsub;
mod ui;

use at1846s::AT1846S;
use core::{arch::asm, panic::PanicInfo};
use cortex_m_rt::entry;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_stm32::{
    bind_interrupts,
    gpio::Level,
    i2c::{self, I2c},
    pac::RCC,
    peripherals,
    rcc::{self, Hse, HseMode, PllSource},
    time::Hertz,
    Config, Peripherals,
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, pubsub::PubSubChannel};
use embassy_time::{Duration, Timer};
use event::Event;
use led::{process_leds, Leds};
use ptt::{process_ptt, Ptt};
use pubsub::{EVENT_CAP, EVENT_PUBS, EVENT_SUBS};

bind_interrupts!(struct Irqs {
    I2C3_EV => i2c::EventInterruptHandler<peripherals::I2C3>;
    I2C3_ER => i2c::ErrorInterruptHandler<peripherals::I2C3>;
});

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
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
            "ldr SP, =0x10010000", // Relocate the stack, this is super dangerous so do it as early as possible.
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
async fn main_post_relocate(spawner: Spawner, peripherals: Peripherals) {
    // Reminder: Event buses can be an anti-pattern when overused in complex ways. Try to avoid this.
    let mut event_channel =
        PubSubChannel::<CriticalSectionRawMutex, Event, EVENT_CAP, EVENT_SUBS, EVENT_PUBS>::new();
    let user_interface = ui::UserInterface::init(
        peripherals.PD14,
        peripherals.PD15,
        peripherals.PD0,
        peripherals.PD1,
        peripherals.PE7,
        peripherals.PE8,
        peripherals.PE9,
        peripherals.PE10,
        peripherals.PD5,
        peripherals.PD12,
        peripherals.PD13,
        peripherals.PD8,
    )
    .await;
    let leds = Leds::init(peripherals.PE1, peripherals.PE0);
    let ptt = Ptt::init(peripherals.PE11);
    Timer::after_millis(100).await;
    let mut rf = {
        let mut i2c_config = i2c::Config::default();
        i2c_config.timeout = Duration::from_secs(1);
        AT1846S::init(I2c::new(
            peripherals.I2C3,
            peripherals.PA8,
            peripherals.PC9,
            Irqs,
            peripherals.DMA1_CH4,
            peripherals.DMA1_CH2,
            Hertz(40_000),
            i2c_config,
        ))
        .await
    };
    rf.tune(430_500_000).await;

    spawner.must_spawn(process_leds(
        leds,
        unsafe { __make_static(&mut event_channel) }
            .subscriber()
            .unwrap(),
    ));
    spawner.must_spawn(process_ptt(
        ptt,
        unsafe { __make_static(&mut event_channel) }
            .publisher()
            .unwrap(),
    ));

    let main_pub = event_channel.publisher().unwrap();
    let mut main_sub = event_channel.subscriber().unwrap();

    loop {
        match main_sub.next_message_pure().await {
            Event::PttOn => main_pub.publish_immediate(Event::RedLed(Level::High)),
            Event::PttOff => main_pub.publish_immediate(Event::RedLed(Level::Low)),
            Event::RedLed(_) => {}
            Event::GreenLed(_) => {}
            Event::TriggerRedraw => {}
            Event::SetVfoFreq(_) => {}
        }
    }
}
