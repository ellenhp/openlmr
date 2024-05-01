#![no_std]

extern crate alloc;

use alloc::vec::Vec;
use channel::LmrChannel;
use serde::{Deserialize, Serialize};

pub mod channel;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LmrCodeplug {
    pub channels: Vec<LmrChannel>,
}
