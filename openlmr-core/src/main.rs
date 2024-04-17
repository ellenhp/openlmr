#![feature(impl_trait_in_assoc_type)]
#![feature(type_alias_impl_trait)]
#![no_std]
#![no_main]

extern crate alloc;

mod at1846s;
mod c6000;
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
    gpio::{Level, Output, Speed},
    i2c::{self, I2c},
    pac::RCC,
    peripherals,
    rcc::{self, Hse, HseMode, PllSource},
    time::Hertz,
    Config, Peripherals,
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, pubsub::PubSubChannel};
use embassy_time::{Duration, Timer};
use embedded_alloc::Heap;
use event::Event;
use futures::StreamExt;
use led::{process_leds, Leds};
use ptt::{process_ptt, Ptt};
use pubsub::{EVENT_CAP, EVENT_PUBS, EVENT_SUBS};
use ui::{process_ui, trigger_redraw_task};

bind_interrupts!(struct Irqs {
    I2C3_EV => i2c::EventInterruptHandler<peripherals::I2C3>;
    I2C3_ER => i2c::ErrorInterruptHandler<peripherals::I2C3>;
});

#[global_allocator]
static HEAP: Heap = Heap::empty();

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
    {
        use core::mem::MaybeUninit;
        const HEAP_SIZE: usize = 65536;
        static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
        unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE) }
    }

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
        peripherals.PD6,
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
        AT1846S::init(
            I2c::new(
                peripherals.I2C3,
                peripherals.PA8,
                peripherals.PC9,
                Irqs,
                peripherals.DMA1_CH4,
                peripherals.DMA1_CH2,
                Hertz(40_000),
                i2c_config,
            ),
            peripherals.PA5,
            peripherals.PA2,
            peripherals.PC5,
            peripherals.PC4,
            peripherals.PC6,
        )
        .await
    };
    rf.tune(146_960_000).await;
    rf.set_25khz_bw().await;
    rf.receive_mode().await;
    rf.set_rx_gain(20).await;
    let mut audio_pa = Output::new(peripherals.PB9, Level::Low, Speed::Low);
    let mut speaker_mute = Output::new(peripherals.PB8, Level::Low, Speed::Low);
    // let mut baseband = {
    //     let mut baseband = c6000::C6000::init(
    //         peripherals.PE2,
    //         peripherals.PE3,
    //         peripherals.PE4,
    //         peripherals.PE5,
    //         peripherals.PE6,
    //     )
    //     .await;
    //     // baseband.fm_mode().await;
    //     // baseband.enable_audio_out().await;
    //     // baseband.set_audio_volume(0).await;
    //     baseband
    // };

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
    spawner.must_spawn(process_ui(
        user_interface,
        unsafe { __make_static(&mut event_channel) }
            .subscriber()
            .unwrap(),
    ));
    spawner.must_spawn(trigger_redraw_task(
        unsafe { __make_static(&mut event_channel) }
            .publisher()
            .unwrap(),
    ));

    let main_pub = event_channel.publisher().unwrap();
    let mut main_sub = event_channel.subscriber().unwrap();

    let mut last_rssi = rf.get_rssi().await;
    loop {
        while main_sub.available() != 0 {
            let message = main_sub.next_message_pure().await;
            match message {
                Event::PttOn => {
                    audio_pa.set_high();
                    speaker_mute.set_low();
                    // rf.transmit_mode().await;
                }
                Event::PttOff => {
                    rf.receive_mode().await;
                    audio_pa.set_low();
                    speaker_mute.set_low();
                }
                _ => {}
            }
        }

        Timer::after_millis(100).await;

        let rssi = rf.get_rssi().await;
        main_pub.publish_immediate(Event::NewRSSI(rssi));

        if rssi > last_rssi {
            main_pub.publish_immediate(Event::RedLed(Level::Low));
            main_pub.publish_immediate(Event::GreenLed(Level::High));
        } else if rssi < last_rssi {
            main_pub.publish_immediate(Event::RedLed(Level::High));
            main_pub.publish_immediate(Event::GreenLed(Level::Low));
        } else if rssi == 0 {
            main_pub.publish_immediate(Event::RedLed(Level::High));
            main_pub.publish_immediate(Event::GreenLed(Level::High));
        } else {
            main_pub.publish_immediate(Event::RedLed(Level::Low));
            main_pub.publish_immediate(Event::GreenLed(Level::Low));
        }
        last_rssi = rssi;
    }
}
