use crate::mipidsi::{ColorInversion, Error};

use super::DcsCommand;

/// Set Invert Mode
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SetInvertMode(pub ColorInversion);

impl DcsCommand for SetInvertMode {
    fn instruction(&self) -> u8 {
        match self.0 {
            ColorInversion::Normal => 0x20,
            ColorInversion::Inverted => 0x21,
        }
    }

    fn fill_params_buf(&self, _buffer: &mut [u8]) -> Result<usize, Error> {
        Ok(0)
    }
}
