use defmt::Format;

use crate::ui::KeypadKey;

#[derive(Format, Debug, Clone)]
pub enum Event {
    PttOn,
    PttOff,

    KeyOn(KeypadKey),
    KeyOff(KeypadKey),

    // State updates.
    NewRSSI(i16),
    RedLed(bool),
    GreenLed(bool),
    TriggerRedraw,
    TuneFreq(u32),
}
