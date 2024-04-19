use display_interface::WriteOnlyDataCommand;
use stm32f4xx_hal::{
    fsmc_lcd::{DataPins8, FsmcLcd, Lcd, LcdPins, SubBank1, Timing},
    prelude::*,
};
use stm32f4xx_hal::{
    pac::{CorePeripherals, Peripherals},
    rcc::RccExt,
};

pub fn new_fsmc() -> Lcd<SubBank1> {
    let cp = CorePeripherals::take().unwrap();
    let dp = Peripherals::take().unwrap();

    let rcc = dp.RCC.constrain();
    // Make HCLK faster to allow updating the display more quickly
    let clocks = rcc.cfgr.hclk(100.MHz()).freeze();

    let mut delay = cp.SYST.delay(&clocks);

    let gpiod = dp.GPIOD.split();
    let gpioe = dp.GPIOE.split();
    let gpiof = dp.GPIOF.split();

    // Pins connected to the LCD on the 32F412GDISCOVERY board
    use stm32f4xx_hal::gpio::alt::fsmc as alt;
    let lcd_pins = LcdPins::new(
        DataPins8::new(
            gpiod.pd14, gpiod.pd15, gpiod.pd0, gpiod.pd1, gpioe.pe7, gpioe.pe8, gpioe.pe9,
            gpioe.pe10,
        ),
        alt::Address::from(gpiod.pd12),
        gpiod.pd4,
        gpiod.pd5,
        alt::ChipSelect1::from(gpiod.pd6),
    );
    let lcd_reset = gpiod.pd11.into_push_pull_output();
    let backlight_control = gpiof.pf5.into_push_pull_output();

    // Speed up timing settings, assuming HCLK is 100 MHz (1 cycle = 10 nanoseconds)
    // These read timings work to read settings, but slower timings are needed to read from the
    // frame memory.
    // Read timing: RD can go low at the same time as D/C changes and CS goes low.
    // RD must be low for at least 45 ns -> DATAST=8
    // Also, a read cycle must take at least 160 nanoseconds, so set ADDSET=8. This means
    // that a whole read takes 16 HCLK cycles (160 nanoseconds).
    // Bus turnaround time is zero, because no particular interval is required between transactions.
    let read_timing = Timing::default().data(8).address_setup(8).bus_turnaround(0);
    // Write timing: Minimum 10 nanoseconds from when WR goes high to CS goes high, so
    // HCLK can't be faster than 100 MHz.
    // NWE must be low for at least 15 ns -> DATAST=3
    // A write cycle must take at least 66 nanoseconds, so ADDSET=3. This means that a whole
    // write cycle takes 7 HCLK cycles (70 nanoseconds) (an extra HCLK cycle is added after NWE
    // goes high).
    // Bus turnaround time is zero, because no particular interval is required between transactions.
    let write_timing = Timing::default().data(3).address_setup(3).bus_turnaround(0);

    let (_fsmc, interface) = FsmcLcd::new(dp.FSMC, lcd_pins, &read_timing, &write_timing);

    interface
}
