//! Module for the RASET address window instruction constructors

use crate::mipidsi::Error;

use super::DcsCommand;

/// Set Page Address
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SetPageAddress {
    start_row: u16,
    end_row: u16,
}

impl SetPageAddress {
    /// Creates a new Set Page Address command.
    pub fn new(start_row: u16, end_row: u16) -> Self {
        Self { start_row, end_row }
    }
}

impl DcsCommand for SetPageAddress {
    fn instruction(&self) -> u8 {
        0x2B
    }

    fn fill_params_buf(&self, buffer: &mut [u8]) -> Result<usize, Error> {
        buffer[0..2].copy_from_slice(&self.start_row.to_be_bytes());
        buffer[2..4].copy_from_slice(&self.end_row.to_be_bytes());

        Ok(4)
    }
}
