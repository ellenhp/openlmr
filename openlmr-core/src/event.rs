use embassy_stm32::gpio::Level;

#[derive(Debug, Clone)]
pub enum Event {
    PttOn,
    PttOff,

    // State updates.
    RedLed(Level),
    GreenLed(Level),
    TriggerRedraw,
    SetVfoFreq(u32),
}
