#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]
#![feature(async_fn_traits)]
#![feature(async_closure)]

extern crate alloc;
extern crate stm32f4xx_hal;

use core::{arch::asm, panic::PanicInfo};
use rtic_monotonics::stm32::Tim2 as Mono;

use embedded_alloc::Heap;
use rtic::app;

mod at1846s;
mod c6000;
mod dfu;
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
    ldr sp, =0x10010000
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

    use core::cell::{Cell, RefCell};

    use crate::{
        at1846s::AT1846S,
        c6000::C6000,
        event::Event,
        iface::DisplayInterface,
        pubsub::{EVENT_CAP, EVENT_PUBS, EVENT_SUBS},
        HEAP,
    };

    use embassy_sync::{
        blocking_mutex::raw::CriticalSectionRawMutex,
        once_lock::OnceLock,
        pubsub::{PubSubChannel, Publisher, Subscriber},
    };

    use stm32f4xx_hal::{
        gpio::{self, alt::fsmc, Input, Output, Pin, PushPull, PA6, PD2, PD3},
        i2c::I2c,
        otg_fs::{UsbBus, UsbBusType, USB},
        pac::{I2C3, RCC},
        prelude::*,
        rtc::Rtc,
    };

    use defmt_rtt as _;
    use usb_device::{
        bus::UsbBusAllocator,
        device::{StringDescriptors, UsbDevice, UsbDeviceBuilder, UsbVidPid},
    };
    use usbd_serial::SerialPort;

    use crate::ui::{process_ui, UserInterface};

    pub static EVENT_CHANNEL_CELL: OnceLock<
        PubSubChannel<CriticalSectionRawMutex, Event, EVENT_CAP, EVENT_SUBS, EVENT_PUBS>,
    > = OnceLock::new();

    pub(crate) static USB_BUS_ALLOC: OnceLock<UsbBusAllocator<UsbBusType>> = OnceLock::new();
    pub(crate) static USB_SERIAL: OnceLock<RefCell<Cell<Option<SerialPort<UsbBusType>>>>> =
        OnceLock::new();
    pub(crate) static USB_DEV: OnceLock<RefCell<Cell<Option<UsbDevice<'static, UsbBusType>>>>> =
        OnceLock::new();

    use core::mem::MaybeUninit;
    const HEAP_SIZE: usize = 65536;
    static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];

    // Resources shared between tasks
    #[shared]
    struct Shared {
        audio_pa: gpio::PB9<Output<PushPull>>,
        speaker_mute: gpio::PB8<Output<PushPull>>,
        usb_dev: UsbDevice<'static, UsbBusType>,
        usb_serial: SerialPort<'static, UsbBusType>,
    }

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

        // Baseband stuff
        c6000_cs: gpio::PE2<Output<PushPull>>,
        c6000_clk: gpio::PE3<Output<PushPull>>,
        c6000_mosi: gpio::PE4<Output<PushPull>>,
        c6000_miso: gpio::PE5<Input>,
        c6000_standby: gpio::PE6<Output<PushPull>>,

        // Stuff that just can't be dropped.
        _lcd_cs: gpio::PD6<Output<PushPull>>,

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
    fn init(cx: init::Context) -> (Shared, Local) {
        {
            // I think there's some kind of OTP set up to start the STM32 on a PLL, because stopping the PLL halts the processor.
            let rcc = unsafe { &*RCC::ptr() };
            rcc.cfgr.modify(|_, regw| unsafe { regw.sw().bits(0) });
            while !rcc.cfgr.read().sws().is_hsi() {}
        }
        let mut dp = cx.device;
        let rcc = dp.RCC.constrain();
        let clocks = rcc
            .cfgr
            .use_hse(8.MHz())
            .sysclk(84.MHz())
            .hclk(168.MHz())
            .pclk1(42.MHz())
            .pclk2(84.MHz())
            .require_pll48clk() // REQUIRED FOR RNG & USB
            .freeze();

        // Scary unsafe stuff.
        static mut USB_BUS: Option<usb_device::bus::UsbBusAllocator<UsbBusType>> = None;
        static mut EP_MEMORY: [u32; 1024] = [0; 1024];
        unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE) }
        // End scary unsafe stuff (for now).

        let _rtc = Rtc::new(dp.RTC, &mut dp.PWR);
        {
            let token = rtic_monotonics::create_stm32_tim2_monotonic_token!();
            let timer_clock_hz = 42_000_000;
            crate::Mono::start(timer_clock_hz, token);
        }

        let port_a = dp.GPIOA.split();
        let port_b = dp.GPIOB.split();
        let port_c = dp.GPIOC.split();
        let port_d = dp.GPIOD.split();
        let port_e = dp.GPIOE.split();
        let _port_f = dp.GPIOF.split();

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

        let ch = EVENT_CHANNEL_CELL.get_or_init(|| {
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

        let audio_pa = port_b.pb9.into_push_pull_output();
        let speaker_mute = port_b.pb8.into_push_pull_output();

        let c6000_cs = port_e.pe2.into_push_pull_output();
        let c6000_clk = port_e.pe3.into_push_pull_output();
        let c6000_mosi = port_e.pe4.into_push_pull_output();
        let c6000_miso = port_e.pe5.into_floating_input();
        let c6000_standby = port_e.pe6.into_push_pull_output();

        // Begin some scary unsafe stuff.
        let usb = USB::new(
            (dp.OTG_FS_GLOBAL, dp.OTG_FS_DEVICE, dp.OTG_FS_PWRCLK),
            (port_a.pa11, port_a.pa12),
            &clocks,
        );
        unsafe {
            USB_BUS.replace(UsbBus::new(usb, &mut EP_MEMORY));
        }

        let usb_serial = usbd_serial::SerialPort::new(unsafe { USB_BUS.as_ref().unwrap() });
        let usb_dev = UsbDeviceBuilder::new(
            unsafe { USB_BUS.as_ref().unwrap() },
            UsbVidPid(0x16c0, 0x27dd),
        )
        .device_class(usbd_serial::USB_CLASS_CDC)
        .strings(&[StringDescriptors::default()
            .manufacturer("Fake Company")
            .product("Product")
            .serial_number("TEST")])
        .unwrap()
        .build();
        // End some scary unsafe stuff.

        let _syscfg = dp.SYSCFG.constrain();

        run_ui::spawn().unwrap();
        run_rf::spawn().unwrap();
        run_baseband::spawn().unwrap();
        heartbeat::spawn().unwrap();

        (
            // Initialization of shared resources
            Shared {
                audio_pa,
                speaker_mute,
                usb_dev,
                usb_serial,
            },
            // Initialization of task local resources
            Local {
                // I/O.
                led1,
                led2,

                // RF chip.
                rf_at1846s,
                rf_publisher,
                rf_subscriber,

                // Baseband chip.
                c6000_cs,
                c6000_clk,
                c6000_mosi,
                c6000_miso,
                c6000_standby,

                // Stuff that shouldn't get dropped.
                _lcd_cs: pd6,

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

    #[task(local = [led1, led2])]
    async fn heartbeat(cx: heartbeat::Context) {
        let led1 = cx.local.led1;
        let led2 = cx.local.led2;
        loop {
            crate::Mono::delay(900.millis().into()).await;
            led1.set_high();
            crate::Mono::delay(100.millis().into()).await;
            led1.set_low();
            crate::Mono::delay(900.millis().into()).await;
            led2.set_high();
            crate::Mono::delay(100.millis().into()).await;
            led2.set_low();
        }
    }

    #[task(local = [
        c6000_cs,
        c6000_clk,
        c6000_mosi,
        c6000_miso,
        c6000_standby,
    ])]
    async fn run_baseband(cx: run_baseband::Context) {
        let c6000_cs = cx.local.c6000_cs;
        let c6000_clk = cx.local.c6000_clk;
        let c6000_mosi = cx.local.c6000_mosi;
        let c6000_miso = cx.local.c6000_miso;
        let c6000_standby = cx.local.c6000_standby;

        let mut baseband =
            C6000::init(c6000_cs, c6000_clk, c6000_mosi, c6000_miso, c6000_standby).await;
        baseband.enable_audio_out().await;
        baseband.set_audio_volume(20).await;

        crate::Mono::delay(100.millis().into()).await;
        loop {
            crate::Mono::delay(100.millis().into()).await;
        }
    }

    #[task(local = [
        rf_at1846s,
        rf_publisher,
        rf_subscriber
    ], shared = [
        audio_pa,
        speaker_mute,
    ])]
    async fn run_rf(mut cx: run_rf::Context) {
        let rf = cx.local.rf_at1846s;
        rf.init().await;
        rf.tune(146_520_000).await;
        rf.set_25khz_bw().await;
        rf.receive_mode().await;
        rf.set_rx_gain(20).await;
        let rf_publisher = cx.local.rf_publisher;
        let rf_subscriber = cx.local.rf_subscriber;
        loop {
            crate::Mono::delay(25.millis().into()).await;
            while rf_subscriber.available() > 0 {
                let message = rf_subscriber.next_message_pure().await;
                match message {
                    Event::PttOn => {}
                    Event::PttOff => {}
                    Event::MoniOn => {
                        if rf.receiving() {
                            cx.shared.audio_pa.lock(|pa| pa.set_high());
                            cx.shared.speaker_mute.lock(|mute| mute.set_low());
                        }
                    }
                    Event::MoniOff => {
                        cx.shared.audio_pa.lock(|pa| pa.set_low());
                        cx.shared.speaker_mute.lock(|mute| mute.set_high());
                    }
                    Event::NewRSSI(_) => {}
                    Event::RedLed(_) => {}
                    Event::GreenLed(_) => {}
                    Event::TriggerRedraw => {}
                    Event::TuneFreq(freq) => {
                        rf.tune(freq).await;
                    }
                }
            }
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
    async fn run_ui(cx: run_ui::Context) {
        let lcd_subscriber = cx.local.lcd_subscriber;
        let lcd_publisher = cx.local.lcd_publisher;
        let rst = cx.local.rst;
        rst.set_low();
        crate::Mono::delay(100.millis().into()).await;
        rst.set_high();
        let interface = cx.local.interface;
        let mut ui = UserInterface::init(cx.local.kb1, cx.local.kb2, cx.local.kb3, interface).await;
        cx.local.backlight.set_high();

        process_ui(&mut ui, lcd_subscriber, lcd_publisher).await;
    }

    #[task(binds=OTG_FS, shared=[usb_dev, usb_serial])]
    fn usb_fs(cx: usb_fs::Context) {
        (cx.shared.usb_dev, cx.shared.usb_serial).lock(|usb_dev, usb_serial| {
            if usb_dev.poll(&mut [usb_serial]) {
                panic!();
                let mut buf = [0u8; 64];

                match usb_serial.read(&mut buf) {
                    Ok(count) if count > 0 => {
                        let mut write_offset = 0;
                        while write_offset < count {
                            match usb_serial.write(&mut buf[write_offset..count]) {
                                Ok(len) if len > 0 => {
                                    write_offset += len;
                                }
                                _ => {}
                            }
                        }
                    }
                    _ => {}
                }
            }
        });
    }
}
