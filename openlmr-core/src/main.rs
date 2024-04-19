#![no_main]
#![no_std]

use core::{arch::asm, panic::PanicInfo};
use rtic_monotonics::stm32::Tim2 as Mono;

use embedded_alloc::Heap;
use rtic::app;

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

#[app(device = stm32f4xx_hal::pac, peripherals = true)]
mod app {
    use core::{
        arch::asm,
        ptr::{self, addr_of},
    };

    use stm32f4xx_hal::{
        gpio::{self, Edge, Input, Output, PushPull},
        pac::{RCC, TIM1},
        prelude::*,
        rtc::{Event, Rtc},
        timer,
    };

    use defmt_rtt as _;

    use crate::HEAP;

    // Resources shared between tasks
    #[shared]
    struct Shared {}

    // Local resources to specific tasks (cannot be shared)
    #[local]
    struct Local {
        led: gpio::PE0<Output<PushPull>>,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        {
            use core::mem::MaybeUninit;
            const HEAP_SIZE: usize = 65536;
            static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
            unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE) }
        }

        let mut dp = ctx.device;

        {
            let rcc = unsafe { &*RCC::ptr() };
            rcc.cfgr.modify(|reg, regw| regw.sw().hsi());
            let mut iter = 0u32;
            while !rcc.cfgr.read().sws().is_hsi() {
                if iter == 10_000 {
                    panic!();
                }
            }
        }

        let rcc = dp.RCC.constrain();

        // let clocks = rcc
        //     .cfgr
        //     .use_hse(8.MHz())
        //     .sysclk(168.MHz())
        //     // .pclk1(24.MHz())
        //     // .require_pll48clk()
        //     .freeze();

        //Configure RTC
        let mut rtc = Rtc::new(dp.RTC, &mut dp.PWR);
        // loop {}
        // unsafe {
        //     asm!(
        //         "LDR R1, =0x40021014;",
        //         "LDR R0, [R1];",
        //         "ORR.W R0, #0x000;",
        //         "STR R0, [R1];",
        //     );
        // }

        // //Set date and time
        // let _ = rtc.set_year(2023);
        // let _ = rtc.set_month(11);
        // let _ = rtc.set_day(25);
        // let _ = rtc.set_hours(22);
        // let _ = rtc.set_minutes(46);
        // let _ = rtc.set_seconds(00);

        //Start listening to WAKE UP INTERRUPTS
        rtc.enable_wakeup(10.secs());
        rtc.listen(&mut dp.EXTI, Event::Wakeup);

        // 3) Create delay handle
        // Configure the LED pin as a push pull ouput and obtain handle
        // On the Blackpill STM32F411CEU6 there is an on-board LED connected to pin PC13
        // 1) Promote the GPIOC PAC struct
        let gpioe = dp.GPIOE.split();

        // 2) Configure PORTC OUTPUT Pins and Obtain Handle
        let led = gpioe.pe0.into_push_pull_output();
        let token = rtic_monotonics::create_stm32_tim2_monotonic_token!();
        let timer_clock_hz = 16_000_000;
        crate::Mono::start(timer_clock_hz, token);

        // Configure Button Pin for Interrupts
        // 1) Promote SYSCFG structure to HAL to be able to configure interrupts
        let mut syscfg = dp.SYSCFG.constrain();
        foo::spawn().ok();
        (
            // Initialization of shared resources
            Shared {},
            // Initialization of task local resources
            Local { led },
        )
    }

    #[task(local = [led])]
    async fn foo(ctx: foo::Context) {
        let led = ctx.local.led;
        loop {
            led.set_low();
            crate::Mono::delay(rtic_monotonics::stm32::ExtU64::millis(1000u64)).await;
            led.set_high();
            crate::Mono::delay(rtic_monotonics::stm32::ExtU64::millis(1000u64)).await;
        }
    }

    #[task(binds = RTC_WKUP, shared = [])]
    fn rtc_wakeup(mut ctx: rtc_wakeup::Context) {}
}
