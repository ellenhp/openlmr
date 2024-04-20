//! Module for the VSCRDEF visual scroll definition instruction constructors

use crate::mipidsi::{Error, ModelOptions};

use super::DcsCommand;

/// Set Scroll Area
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SetScrollArea {
    tfa: u16,
    vsa: u16,
    bfa: u16,
}

impl SetScrollArea {
    /// Creates a new Set Scroll Area command.
    ///
    /// VSA should default to the display's height (or width) framebuffer size.
    pub fn new(tfa: u16, vsa: u16, bfa: u16) -> Self {
        Self { tfa, vsa, bfa }
    }
}

impl DcsCommand for SetScrollArea {
    fn instruction(&self) -> u8 {
        0x33
    }

    fn fill_params_buf(&self, buffer: &mut [u8]) -> Result<usize, Error> {
        let tfa_bytes = self.tfa.to_be_bytes();
        let vsa_bytes = self.vsa.to_be_bytes();
        let bfa_bytes = self.bfa.to_be_bytes();

        buffer[0] = tfa_bytes[0];
        buffer[1] = tfa_bytes[1];
        buffer[2] = vsa_bytes[0];
        buffer[3] = vsa_bytes[1];
        buffer[4] = bfa_bytes[0];
        buffer[5] = bfa_bytes[1];

        Ok(6)
    }
}

impl From<&ModelOptions> for SetScrollArea {
    fn from(options: &ModelOptions) -> Self {
        Self {
            tfa: 0,
            vsa: options.framebuffer_size_max(),
            bfa: 0,
        }
    }
}
