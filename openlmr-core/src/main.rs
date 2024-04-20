#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]
#![feature(async_fn_traits)]
#![feature(async_closure)]

extern crate alloc;

use core::{arch::asm, mem::MaybeUninit, panic::PanicInfo};
use rtic_monotonics::stm32::Tim2 as Mono;

use embedded_alloc::Heap;
use rtic::app;

mod event;
mod iface;
mod mipidsi;
mod ui;

#[global_allocator]
static HEAP: Heap = Heap::empty();

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {
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

    use crate::iface::DisplayInterface;
    use rtic_monotonics::stm32::Tim2;
    use stm32f4xx_hal::{
        fsmc_lcd::{DataPins8, FsmcLcd, Lcd, LcdPins, SubBank1, Timing},
        gpio::{
            self,
            alt::fsmc::{self, Address, Ne1},
            Edge, Input, Output, Pin, PushPull, PA6, PD13, PD2, PD3, PD8,
        },
        pac::{RCC, TIM1},
        prelude::*,
        rtc::{Event, Rtc},
        timer,
    };

    use defmt_rtt as _;

    use crate::{
        ui::{process_ui, UserInterface},
        HEAP,
    };

    // Resources shared between tasks
    #[shared]
    struct Shared {}

    // Local resources to specific tasks (cannot be shared)
    #[local]
    struct Local {
        led1: gpio::PE0<Output<PushPull>>,
        led2: gpio::PE1<Output<PushPull>>,

        actual_lcd_cs: gpio::PD6<Output<PushPull>>,

        rst: gpio::PD13<Output<PushPull>>,
        backlight: gpio::PD8<Output<PushPull>>,
        kb1: PA6,
        kb2: PD2,
        kb3: PD3,
        interface: DisplayInterface,
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
            static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
            unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE) }
        }

        //Configure RTC
        let mut rtc = Rtc::new(dp.RTC, &mut dp.PWR);

        let token = rtic_monotonics::create_stm32_tim2_monotonic_token!();
        let timer_clock_hz = 32_000_000;
        crate::Mono::start(timer_clock_hz, token);

        let port_a = dp.GPIOA.split();
        let port_d = dp.GPIOD.split();
        let port_e = dp.GPIOE.split();
        let port_f = dp.GPIOF.split();

        let led1 = port_e.pe0.into_push_pull_output();
        let led2 = port_e.pe1.into_push_pull_output();
        let mut rst = port_d
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

        // Configure Button Pin for Interrupts
        // 1) Promote SYSCFG structure to HAL to be able to configure interrupts
        let syscfg = dp.SYSCFG.constrain();
        foo::spawn().ok();
        heartbeat::spawn().ok();
        (
            // Initialization of shared resources
            Shared {},
            // Initialization of task local resources
            Local {
                led1,
                led2,
                actual_lcd_cs: pd6,
                rst,
                backlight,
                kb1,
                kb2,
                kb3,
                interface,
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

    #[task(local = [
        rst,
        backlight,
        kb1,
        kb2,
        kb3,
        interface])]
    async fn foo(ctx: foo::Context) {
        let rst = ctx.local.rst;
        rst.set_low();
        crate::Mono::delay(100.millis().into()).await;
        rst.set_high();
        let interface = ctx.local.interface;
        let mut ui =
            UserInterface::init(ctx.local.kb1, ctx.local.kb2, ctx.local.kb3, interface).await;
        ctx.local.backlight.set_high();
        process_ui(&mut ui).await;
    }
}
