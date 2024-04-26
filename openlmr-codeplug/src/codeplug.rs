use alloc::{string::String, vec::Vec};
use rkyv::{Archive, Deserialize, Serialize};

#[derive(Debug, Clone, PartialEq, Archive)]
pub struct LmrCodeplug {
    channels: Vec<LmrChannel>,
}

#[derive(Debug, Clone, PartialEq, Deserialize, Serialize, Archive)]
pub enum LmrMode {
    /// Frequency modulated voice. This does not imply any particular bandwidth.
    FM,
    /// DMR.
    DMR,
}

#[derive(Debug, Clone, PartialEq, Deserialize, Serialize, Archive)]
pub struct LmrChannel {
    /// Modulation and protocol.
    mode: LmrMode,
    /// Frequency in megahertz.
    freq: f32,
    /// Logical grouping for this channel.
    zone: String,
    /// DMR Timeslot.
    timeslot: u8,
}
