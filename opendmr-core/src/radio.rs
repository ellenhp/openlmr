use embedded_graphics::{draw_target::DrawTarget, geometry::OriginDimensions};

use crate::display_keypad::LockedDisplay;

pub trait Radio<Target: DrawTarget + OriginDimensions> {
    fn indicate_rx(is_receiving: bool);
    fn indicate_tx(is_transmitting: bool);

    fn lock_display() -> LockedDisplay<Target>;
}
