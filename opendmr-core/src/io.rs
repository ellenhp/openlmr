use embassy_stm32::gpio::Output;

pub struct LED {
    pin: Output<'static>,
}

impl LED {
    pub fn on(&mut self) {
        self.pin.set_high();
    }

    pub fn off(&mut self) {
        self.pin.set_low();
    }
}
