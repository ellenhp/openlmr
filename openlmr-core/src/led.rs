use embassy_stm32::{
    gpio::{Level, Output, Speed},
    peripherals::{PE0, PE1},
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, pubsub::Subscriber};

use crate::{
    event::Event,
    pubsub::{EVENT_CAP, EVENT_PUBS, EVENT_SUBS},
};

pub struct Leds {
    red: Output<'static>,
    green: Output<'static>,
}

impl Leds {
    pub fn init(red: PE1, green: PE0) -> Leds {
        Leds {
            red: Output::new(red, Level::Low, Speed::Low),
            green: Output::new(green, Level::Low, Speed::Low),
        }
    }
}

#[embassy_executor::task()]
pub async fn process_leds(
    mut leds: Leds,
    mut state_update_sub: Subscriber<
        'static,
        CriticalSectionRawMutex,
        Event,
        EVENT_CAP,
        EVENT_SUBS,
        EVENT_PUBS,
    >,
) {
    loop {
        let message = state_update_sub.next_message_pure().await;
        match message {
            Event::RedLed(level) => leds.red.set_level(level),
            Event::GreenLed(level) => leds.green.set_level(level),
            _ => {}
        }
    }
}
