//! [Error] module for [super::Display]

use display_interface::DisplayError;

/// Error returned by [`Builder::init`](crate::Builder).
#[derive(Debug)]
pub enum InitError {
    /// Error caused by the display interface.
    DisplayError,
}

///
/// Alias of [DisplayError] for out-of-init use cases
/// since the pin error is only possible during [super::Builder] use
///
pub type Error = DisplayError;

impl From<DisplayError> for InitError {
    fn from(_: DisplayError) -> Self {
        InitError::DisplayError
    }
}
