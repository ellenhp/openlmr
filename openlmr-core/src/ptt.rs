use embassy_stm32::{
    gpio::{Input, Level, Pull},
    peripherals::PE11,
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, pubsub::Publisher};
use embassy_time::Timer;

use crate::{
    event::Event,
    pubsub::{EVENT_CAP, EVENT_PUBS, EVENT_SUBS},
};

pub struct Ptt {
    ptt: Input<'static>,
}

impl Ptt {
    pub fn init(ptt: PE11) -> Ptt {
        Ptt {
            ptt: Input::new(ptt, Pull::Up),
        }
    }
}

#[embassy_executor::task()]
pub async fn process_ptt(
    ptt: Ptt,
    publisher: Publisher<
        'static,
        CriticalSectionRawMutex,
        Event,
        EVENT_CAP,
        EVENT_SUBS,
        EVENT_PUBS,
    >,
) {
    let mut previous_ptt_statuses: [Level; 3] = [Level::High, Level::High, Level::High];
    loop {
        // TODO: Include external PTT button.
        // Best-effort async debounce.
        Timer::after_millis(10).await;
        let level_this_iteration = ptt.ptt.get_level();
        let to_bump = previous_ptt_statuses[0];
        for i in 1..previous_ptt_statuses.len() {
            previous_ptt_statuses[i - 1] = previous_ptt_statuses[i];
        }
        *previous_ptt_statuses.last_mut().unwrap() = level_this_iteration;
        match to_bump {
            Level::Low => {
                if previous_ptt_statuses
                    .iter()
                    .all(|level| level == &Level::High)
                {
                    publisher.publish_immediate(Event::PttOff);
                }
            }
            Level::High => {
                if previous_ptt_statuses
                    .iter()
                    .all(|level| level == &Level::Low)
                {
                    publisher.publish_immediate(Event::PttOn);
                }
            }
        }
    }
}
