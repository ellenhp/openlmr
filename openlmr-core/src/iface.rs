use core::{borrow::BorrowMut, cell::RefCell, ops::AsyncFnOnce};

use cortex_m::interrupt::CriticalSection;
use stm32f4xx_hal::prelude::*;
use stm32f4xx_hal::{
    fsmc_lcd::{AccessMode, DataPins8, FsmcLcd, Lcd, LcdPins, SubBank1, Timing},
    gpio::{alt::fsmc, PinExt},
    pac::FSMC,
};

struct DisplayInterfacePeripherals {
    fsmc: RefCell<Option<FSMC>>,
    d0: RefCell<Option<fsmc::D0>>,
    d1: RefCell<Option<fsmc::D1>>,
    d2: RefCell<Option<fsmc::D2>>,
    d3: RefCell<Option<fsmc::D3>>,
    d4: RefCell<Option<fsmc::D4>>,
    d5: RefCell<Option<fsmc::D5>>,
    d6: RefCell<Option<fsmc::D6>>,
    d7: RefCell<Option<fsmc::D7>>,
    address: RefCell<Option<fsmc::Address>>,
    read_enable: RefCell<Option<fsmc::Noe>>,
    write_enable: RefCell<Option<fsmc::Nwe>>,
    chip_select: RefCell<Option<fsmc::ChipSelect1>>,
}

impl DisplayInterfacePeripherals {
    pub async fn lend_to_async<
        CB: AsyncFnOnce(
            FSMC,
            fsmc::D0,
            fsmc::D1,
            fsmc::D2,
            fsmc::D3,
            fsmc::D4,
            fsmc::D5,
            fsmc::D6,
            fsmc::D7,
            fsmc::Address,
            fsmc::Noe,
            fsmc::Nwe,
            fsmc::ChipSelect1,
        ) -> (
            FSMC,
            fsmc::D0,
            fsmc::D1,
            fsmc::D2,
            fsmc::D3,
            fsmc::D4,
            fsmc::D5,
            fsmc::D6,
            fsmc::D7,
            fsmc::Address,
            fsmc::Noe,
            fsmc::Nwe,
            fsmc::ChipSelect1,
        ),
    >(
        &mut self,
        cb: CB,
    ) {
        let cs = unsafe { CriticalSection::new() };
        let mut fsmc_borrow = self.fsmc.borrow_mut();
        let mut d0_borrow = self.d0.borrow_mut();
        let mut d1_borrow = self.d1.borrow_mut();
        let mut d2_borrow = self.d2.borrow_mut();
        let mut d3_borrow = self.d3.borrow_mut();
        let mut d4_borrow = self.d4.borrow_mut();
        let mut d5_borrow = self.d5.borrow_mut();
        let mut d6_borrow = self.d6.borrow_mut();
        let mut d7_borrow = self.d7.borrow_mut();

        let mut address_borrow = self.address.borrow_mut();
        let mut read_enable_borrow = self.read_enable.borrow_mut();
        let mut write_enable_borrow = self.write_enable.borrow_mut();
        let mut chip_select_borrow = self.chip_select.borrow_mut();

        let fsmc = fsmc_borrow.take().unwrap();
        let d0 = d0_borrow.take().unwrap();
        let d1 = d1_borrow.take().unwrap();
        let d2 = d2_borrow.take().unwrap();
        let d3 = d3_borrow.take().unwrap();
        let d4 = d4_borrow.take().unwrap();
        let d5 = d5_borrow.take().unwrap();
        let d6 = d6_borrow.take().unwrap();
        let d7 = d7_borrow.take().unwrap();

        let address = address_borrow.take().unwrap();
        let read_enable = read_enable_borrow.take().unwrap();
        let write_enable = write_enable_borrow.take().unwrap();
        let chip_select = chip_select_borrow.take().unwrap();

        let (fsmc, d0, d1, d2, d3, d4, d5, d6, d7, address, read_enable, write_enable, chip_select) =
            cb(
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
            )
            .await;
        fsmc_borrow.replace(fsmc);
        d0_borrow.replace(d0);
        d1_borrow.replace(d1);
        d2_borrow.replace(d2);
        d3_borrow.replace(d3);
        d4_borrow.replace(d4);
        d5_borrow.replace(d5);
        d6_borrow.replace(d6);
        d7_borrow.replace(d7);
        address_borrow.replace(address);
        read_enable_borrow.replace(read_enable);
        write_enable_borrow.replace(write_enable);
        chip_select_borrow.replace(chip_select);
    }
}

pub struct DisplayInterface {
    pins: DisplayInterfacePeripherals,
}

impl DisplayInterface {
    pub fn new(
        fsmc: FSMC,
        d0: fsmc::D0,
        d1: fsmc::D1,
        d2: fsmc::D2,
        d3: fsmc::D3,
        d4: fsmc::D4,
        d5: fsmc::D5,
        d6: fsmc::D6,
        d7: fsmc::D7,
        address: fsmc::Address,
        read_enable: fsmc::Noe,
        write_enable: fsmc::Nwe,
        chip_select: fsmc::ChipSelect1,
    ) -> Self {
        Self {
            pins: DisplayInterfacePeripherals {
                fsmc: RefCell::new(Option::Some(fsmc)),
                d0: RefCell::new(Option::Some(d0)),
                d1: RefCell::new(Option::Some(d1)),
                d2: RefCell::new(Option::Some(d2)),
                d3: RefCell::new(Option::Some(d3)),
                d4: RefCell::new(Option::Some(d4)),
                d5: RefCell::new(Option::Some(d5)),
                d6: RefCell::new(Option::Some(d6)),
                d7: RefCell::new(Option::Some(d7)),
                address: RefCell::new(Option::Some(address)),
                read_enable: RefCell::new(Option::Some(read_enable)),
                write_enable: RefCell::new(Option::Some(write_enable)),
                chip_select: RefCell::new(Option::Some(chip_select)),
            },
        }
    }

    fn scan(&mut self) {}

    pub async fn with_interface_async<CB: AsyncFnOnce(&mut Lcd<SubBank1>) -> ()>(
        &mut self,
        cb: CB,
    ) {
        self.pins
            .lend_to_async(
                async |fsmc,
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
                       chip_select| {
                    let read_timing = Timing::default()
                        .data(5)
                        .address_setup(7)
                        .bus_turnaround(0)
                        .access_mode(AccessMode::ModeA);
                    let write_timing = Timing::default()
                        .data(5)
                        .address_setup(7)
                        .bus_turnaround(0)
                        .access_mode(AccessMode::ModeA);

                    let lcd_pins = LcdPins::new(
                        DataPins8::new(d0, d1, d2, d3, d4, d5, d6, d7),
                        address,
                        read_enable,
                        write_enable,
                        chip_select,
                    );

                    crate::Mono::delay(1.millis().into()).await;

                    let (fsmc_lcd, mut lcd) =
                        FsmcLcd::new(fsmc, lcd_pins, &read_timing, &write_timing);

                    cb(&mut lcd).await;

                    let (fsmc, lcd_pins) = fsmc_lcd.release(lcd);
                    let (data_pins, address, read_enable, write_enable, chip_select) =
                        lcd_pins.split();
                    let (d0, d1, d2, d3, d4, d5, d6, d7) = data_pins.split();

                    (
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
                    )
                },
            )
            .await;
    }
}
