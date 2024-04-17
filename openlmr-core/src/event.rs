use embassy_stm32::gpio::Level;

#[derive(Debug, Clone)]
pub enum Event {
    PttOn,
    PttOff,

    // State updates.
    NewRSSI(i16),
    RedLed(Level),
    GreenLed(Level),
    TriggerRedraw,
    TuneFreq(u32),
}
