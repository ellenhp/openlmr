#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]
#![feature(async_fn_traits)]
#![feature(async_closure)]

extern crate alloc;

use core::{
    arch::{asm, global_asm},
    panic::PanicInfo,
};
use rtic_monotonics::stm32::Tim2 as Mono;

use embedded_alloc::Heap;
use rtic::app;

mod at1846s;
mod event;
mod iface;
mod mipidsi;
mod pubsub;
mod ui;

core::arch::global_asm!(
    "
    .text
    .globl __pre_init
    __pre_init:
    ldr sp, =0x20020000
    ",
    // Initialise .bss memory. `__sbss` and `__ebss` come from the linker script.
    "ldr r0, =__sbss
     ldr r1, =__ebss
     movs r2, #0
     2:
     cmp r1, r0
     beq 3f
     stm r0!, {{r2}}
     b 2b
     3:",
    // Initialise .data memory. `__sdata`, `__sidata`, and `__edata` come from the linker script.
    "ldr r0, =__sdata
     ldr r1, =__edata
     ldr r2, =__sidata
     4:
     cmp r1, r0
     beq 5f
     ldm r2!, {{r3}}
     stm r0!, {{r3}}
     b 4b
     5:",
    // Enable the FPU.
    // SCB.CPACR is 0xE000_ED88.
    // We enable access to CP10 and CP11 from priviliged and unprivileged mode.
    "ldr r0, =0xE000ED88
     ldr r1, =(0b1111 << 20)
     ldr r2, [r0]
     orr r2, r2, r1
     str r2, [r0]
     dsb
     isb",
    // Jump to user main function.
    // `bl` is used for the extended range, but the user main function should not return,
    // so trap on any unexpected return.
    "bl main
     udf #0",
);
// global_asm!("__pre_init:", "ldr SP, =0x20020000",);

#[global_allocator]
static HEAP: Heap = Heap::empty();

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {
        // Turn on the red LED.
        unsafe {
            asm!(
                "LDR R1, =0x40021014;",
                "LDR R0, [R1];",
                "ORR.W R0, #0x0002;",
                "STR R0, [R1];",
            );
        }
    }
}

#[app(device = stm32f4xx_hal::pac, peripherals = true)]
mod app {
    use core::{
        arch::asm,
        mem::MaybeUninit,
        ops::DerefMut,
        ptr::{self, addr_of},
    };

    use crate::{
        at1846s::AT1846S,
        event::Event,
        iface::DisplayInterface,
        pubsub::{EVENT_CAP, EVENT_PUBS, EVENT_SUBS},
    };
    use cortex_m_rt::pre_init;
    use embassy_sync::{
        blocking_mutex::raw::CriticalSectionRawMutex,
        once_lock::OnceLock,
        pubsub::{PubSubChannel, Publisher, Subscriber},
    };
    use rtic_monotonics::stm32::Tim2;
    use stm32f4xx_hal::{
        gpio::{
            self,
            alt::{
                fsmc::{self},
                i2c3,
            },
            Edge, Input, Output, Pin, PushPull, PA6, PD13, PD2, PD3, PD8,
        },
        i2c::I2c,
        pac::{I2C3, RCC, TIM1},
        prelude::*,
        rtc::Rtc,
        timer,
    };

    use defmt_rtt as _;

    use crate::{
        ui::{process_ui, UserInterface},
        HEAP,
    };

    pub static event_channel_cell: OnceLock<
        PubSubChannel<CriticalSectionRawMutex, Event, EVENT_CAP, EVENT_SUBS, EVENT_PUBS>,
    > = OnceLock::new();

    // Resources shared between tasks
    #[shared]
    struct Shared {}

    // Local resources to specific tasks (cannot be shared)
    #[local]
    struct Local {
        // I/O.
        led1: gpio::PE0<Output<PushPull>>,
        led2: gpio::PE1<Output<PushPull>>,

        // RF chip stuff.
        rf_at1846s: AT1846S<
            I2C3,
            gpio::PA5<Output<PushPull>>,
            gpio::PA2<Output<PushPull>>,
            gpio::PC5<Output<PushPull>>,
            gpio::PC4<Output<PushPull>>,
            gpio::PC6<Output<PushPull>>,
        >,
        rf_publisher:
            Publisher<'static, CriticalSectionRawMutex, Event, EVENT_CAP, EVENT_SUBS, EVENT_PUBS>,
        rf_subscriber:
            Subscriber<'static, CriticalSectionRawMutex, Event, EVENT_CAP, EVENT_SUBS, EVENT_PUBS>,

        // Stuff that just can't be dropped.
        actual_lcd_cs: gpio::PD6<Output<PushPull>>,

        // Stuff for the display/keypad.
        rst: gpio::PD13<Output<PushPull>>,
        backlight: gpio::PD8<Output<PushPull>>,
        kb1: PA6,
        kb2: PD2,
        kb3: PD3,
        interface: DisplayInterface,
        lcd_publisher:
            Publisher<'static, CriticalSectionRawMutex, Event, EVENT_CAP, EVENT_SUBS, EVENT_PUBS>,
        lcd_subscriber:
            Subscriber<'static, CriticalSectionRawMutex, Event, EVENT_CAP, EVENT_SUBS, EVENT_PUBS>,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        {
            let rcc = unsafe { &*RCC::ptr() };
            rcc.cfgr.modify(|reg, regw| unsafe { regw.sw().bits(0) });
            while !rcc.cfgr.read().sws().is_hsi() {}
        }
        let mut dp = ctx.device;
        let rcc = dp.RCC.constrain();
        let clocks = rcc
            .cfgr
            .use_hse(8.MHz())
            .sysclk(168.MHz())
            // .require_pll48clk() // REQUIRED FOR RNG.
            .freeze();

        {
            use core::mem::MaybeUninit;
            const HEAP_SIZE: usize = 65536;
            // TODO: Can this be here? This stack frame gets clobbered after the method returns I think.
            static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
            unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE) }
        }

        let _rtc = Rtc::new(dp.RTC, &mut dp.PWR);
        {
            let token = rtic_monotonics::create_stm32_tim2_monotonic_token!();
            let timer_clock_hz = 32_000_000;
            crate::Mono::start(timer_clock_hz, token);
        }

        let port_a = dp.GPIOA.split();
        let port_c = dp.GPIOC.split();
        let port_d = dp.GPIOD.split();
        let port_e = dp.GPIOE.split();
        let port_f = dp.GPIOF.split();

        // Set up LEDs.
        let led1 = port_e.pe0.into_push_pull_output();
        let led2 = port_e.pe1.into_push_pull_output();

        // Set up LCD pins.
        let rst = port_d
            .pd13
            .into_push_pull_output_in_state(gpio::PinState::High);
        let backlight = port_d.pd8.into_push_pull_output();
        let kb1: PA6 = port_a.pa6;
        let kb2: PD2 = port_d.pd2;
        let kb3: PD3 = port_d.pd3;

        let fsmc = dp.FSMC;
        let d0: fsmc::D0 = port_d.pd14.into();
        let d1: fsmc::D1 = port_d.pd15.into();
        let d2: fsmc::D2 = port_d.pd0.into();
        let d3: fsmc::D3 = port_d.pd1.into();
        let d4: fsmc::D4 = port_e.pe7.into();
        let d5: fsmc::D5 = port_e.pe8.into();
        let d6: fsmc::D6 = port_e.pe9.into();
        let d7: fsmc::D7 = port_e.pe10.into();
        let pd6 = port_d
            .pd6
            .into_push_pull_output_in_state(gpio::PinState::Low);
        let address: fsmc::Address = port_d.pd12.into();
        let read_enable: fsmc::Noe = port_d.pd4.into();
        let write_enable: fsmc::Nwe = port_d.pd5.into();
        let chip_select: fsmc::ChipSelect1 = port_d.pd7.into();
        let interface = DisplayInterface::new(
            fsmc,
            d0,
            d1,
            d2,
            d3,
            d4,
            d5,
            d6,
            d7,
            address,
            read_enable,
            write_enable,
            chip_select,
        );

        // Set up RF chip.
        let i2c_rf = I2c::new(dp.I2C3, (port_a.pa8, port_c.pc9), 400.kHz(), &clocks);
        let rf_at1846s: AT1846S<
            I2C3,
            Pin<'A', 5, Output>,
            Pin<'A', 2, Output>,
            Pin<'C', 5, Output>,
            Pin<'C', 4, Output>,
            Pin<'C', 6, Output>,
        > = AT1846S::new(
            i2c_rf,
            port_a.pa5.into_push_pull_output(),
            port_a.pa2.into_push_pull_output(),
            port_c.pc5.into_push_pull_output(),
            port_c.pc4.into_push_pull_output(),
            port_c.pc6.into_push_pull_output(),
        );

        let ch = event_channel_cell.get_or_init(|| {
            PubSubChannel::<CriticalSectionRawMutex, Event, EVENT_CAP, EVENT_SUBS, EVENT_PUBS>::new(
            )
        });
        let rf_publisher: Publisher<
            'static,
            CriticalSectionRawMutex,
            Event,
            EVENT_CAP,
            EVENT_SUBS,
            EVENT_PUBS,
        > = ch.publisher().unwrap();
        let lcd_publisher: Publisher<
            'static,
            CriticalSectionRawMutex,
            Event,
            EVENT_CAP,
            EVENT_SUBS,
            EVENT_PUBS,
        > = ch.publisher().unwrap();
        let rf_subscriber: Subscriber<
            'static,
            CriticalSectionRawMutex,
            Event,
            EVENT_CAP,
            EVENT_SUBS,
            EVENT_PUBS,
        > = ch.subscriber().unwrap();
        let lcd_subscriber: Subscriber<
            'static,
            CriticalSectionRawMutex,
            Event,
            EVENT_CAP,
            EVENT_SUBS,
            EVENT_PUBS,
        > = ch.subscriber().unwrap();

        let _syscfg = dp.SYSCFG.constrain();
        run_ui::spawn().ok();
        run_rf::spawn().ok();
        heartbeat::spawn().ok();

        (
            // Initialization of shared resources
            Shared {},
            // Initialization of task local resources
            Local {
                // I/O.
                led1,
                led2,

                // RF chip.
                rf_at1846s,
                rf_publisher,
                rf_subscriber,

                // Stuff that shouldn't get dropped.
                actual_lcd_cs: pd6,

                // LCD/Keypad.
                rst,
                backlight,
                kb1,
                kb2,
                kb3,
                interface,
                lcd_publisher,
                lcd_subscriber,
            },
        )
    }

    #[task(local = [led1])]
    async fn heartbeat(ctx: heartbeat::Context) {
        let led = ctx.local.led1;
        loop {
            crate::Mono::delay(100.millis().into()).await;
            led.set_low();
            crate::Mono::delay(900.millis().into()).await;
            led.set_high();
        }
    }

    #[task(local = [rf_at1846s, rf_publisher, rf_subscriber], shared = [])]
    async fn run_rf(mut ctx: run_rf::Context) {
        let rf = ctx.local.rf_at1846s;
        rf.init().await;
        rf.receive_mode().await;
        rf.fm_mode().await;
        rf.tune(146_520_000).await;
        let rf_publisher = ctx.local.rf_publisher;
        let rf_subscriber = ctx.local.rf_subscriber;
        loop {
            crate::Mono::delay(100.millis().into()).await;
            let rssi = rf.get_rssi().await;
            rf_publisher.publish_immediate(Event::NewRSSI(rssi));
        }
    }

    #[task(local = [
        rst,
        backlight,
        kb1,
        kb2,
        kb3,
        interface,
        lcd_publisher,
        lcd_subscriber,
    ])]
    async fn run_ui(ctx: run_ui::Context) {
        let lcd_subscriber = ctx.local.lcd_subscriber;
        let lcd_publisher = ctx.local.lcd_publisher;
        let rst = ctx.local.rst;
        rst.set_low();
        crate::Mono::delay(100.millis().into()).await;
        rst.set_high();
        let interface = ctx.local.interface;
        let mut ui =
            UserInterface::init(ctx.local.kb1, ctx.local.kb2, ctx.local.kb3, interface).await;
        ctx.local.backlight.set_high();

        process_ui(&mut ui, lcd_subscriber, lcd_publisher).await;
    }
}
