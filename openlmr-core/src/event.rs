#[derive(Debug, Clone)]
pub enum Event {
    PttOn,
    PttOff,
    MoniOn,
    MoniOff,

    // State updates.
    NewRSSI(i16),
    RedLed(bool),
    GreenLed(bool),
    TriggerRedraw,
    TuneFreq(u32),
}
