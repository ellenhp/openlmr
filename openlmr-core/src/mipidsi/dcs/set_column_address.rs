//! Module for the CASET address window instruction constructors

use crate::mipidsi::Error;

use super::DcsCommand;

/// Set Column Address
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SetColumnAddress {
    start_column: u16,
    end_column: u16,
}

impl SetColumnAddress {
    /// Creates a new Set Column Address command.
    pub fn new(start_column: u16, end_column: u16) -> Self {
        Self {
            start_column,
            end_column,
        }
    }
}

impl DcsCommand for SetColumnAddress {
    fn instruction(&self) -> u8 {
        0x2A
    }

    fn fill_params_buf(&self, buffer: &mut [u8]) -> Result<usize, Error> {
        buffer[0..2].copy_from_slice(&self.start_column.to_be_bytes());
        buffer[2..4].copy_from_slice(&self.end_column.to_be_bytes());

        Ok(4)
    }
}
