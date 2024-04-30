/// Hardcoded constants in this file only work in 1600bps mode.
use core::{
    borrow::Borrow,
    cell::{OnceCell, RefCell},
    ops::AsyncFn,
};

use alloc::string::String;
use codec2::{Codec2, Codec2Mode};
use defmt::Format;
use embassy_sync::{
    blocking_mutex::{raw::CriticalSectionRawMutex, Mutex},
    mutex,
};

use crate::app::decode_task;

#[derive(Debug, Clone, Copy, Format, PartialEq)]
pub enum PlaybackKey {
    OpenLmr,
}

#[derive(Debug, Clone, Copy, Format, PartialEq)]
enum ActivePlaybackBuffer {
    Ping,
    Pong,
}
#[derive(Debug, Clone, Copy, Format, PartialEq)]
struct PlaybackBufferStatus {
    /// Which playback buffer is being played from.
    buffer: ActivePlaybackBuffer,
    /// Used to keep track of whether playback is active for debugging purposes.
    active_playback: bool,
}

struct PlaybackState {
    buf: [i16; 320],
    cursor: u16,
}

struct DecoderState {
    codec: OnceCell<Codec2>,
    loader_key: Option<PlaybackKey>,
    cursor: usize,
}

static PLAYBACK_PING: Mutex<CriticalSectionRawMutex, RefCell<PlaybackState>> = Mutex::const_new(
    CriticalSectionRawMutex::new(),
    RefCell::new(PlaybackState {
        buf: [0; 320],
        cursor: 640,
    }),
);

static PLAYBACK_PONG: Mutex<CriticalSectionRawMutex, RefCell<PlaybackState>> = Mutex::const_new(
    CriticalSectionRawMutex::new(),
    RefCell::new(PlaybackState {
        buf: [0; 320],
        cursor: 640,
    }),
);

static DECODER: mutex::Mutex<CriticalSectionRawMutex, RefCell<DecoderState>> =
    mutex::Mutex::new(RefCell::new(DecoderState {
        codec: OnceCell::new(),
        loader_key: None,
        cursor: 0,
    }));

static PLAYBACK_BUFFER_STATUS: Mutex<CriticalSectionRawMutex, RefCell<PlaybackBufferStatus>> =
    Mutex::const_new(
        CriticalSectionRawMutex::new(),
        RefCell::new(PlaybackBufferStatus {
            buffer: ActivePlaybackBuffer::Ping,
            active_playback: false,
        }),
    );

pub async fn decode() {
    if PLAYBACK_BUFFER_STATUS.lock(|state| match state.borrow().buffer {
        ActivePlaybackBuffer::Pong => PLAYBACK_PING.lock(|ping| ping.borrow().cursor == 0),
        ActivePlaybackBuffer::Ping => PLAYBACK_PONG.lock(|pong| pong.borrow().cursor == 0),
    }) {
        return;
    }
    let mut samples: [i16; 320] = [0; 320];
    let mut decoder_ref = DECODER.lock().await;
    let mut decoder = decoder_ref.borrow_mut();
    let buf = match &decoder.loader_key {
        Some(key) => {
            let cursor = decoder.cursor;
            match load_chunk(&key, cursor).await {
                Some(buf) => {
                    decoder.cursor += 1;
                    buf
                }
                None => {
                    defmt::debug!("Playback stopped");
                    PLAYBACK_BUFFER_STATUS
                        .lock(|status_ref| status_ref.borrow_mut().active_playback = false);
                    decoder.loader_key = None;
                    return;
                }
            }
        }
        None => {
            return;
        }
    };
    decoder
        .codec
        .get_mut_or_init(|| codec2::Codec2::new(Codec2Mode::MODE_1600))
        .decode(&mut samples, &buf);

    PLAYBACK_BUFFER_STATUS.lock(|state| {
        match state.borrow().buffer {
            ActivePlaybackBuffer::Ping => PLAYBACK_PONG.lock(|playback| {
                playback.borrow_mut().buf.copy_from_slice(&samples);
                playback.borrow_mut().cursor = 0;
            }),
            ActivePlaybackBuffer::Pong => PLAYBACK_PING.lock(|playback| {
                playback.borrow_mut().buf.copy_from_slice(&samples);
                playback.borrow_mut().cursor = 0;
            }),
        };
        state.borrow_mut().active_playback = true;
    });
    defmt::debug!("Filled audio buffer");
}

async fn load_chunk(key: &PlaybackKey, chunk: usize) -> Option<[u8; 8]> {
    let raw = match key {
        PlaybackKey::OpenLmr => openlmr_voice::OPENLMR,
    };

    if raw.len() < 8 * (chunk + 1) {
        return None;
    }
    let base = chunk * 8;
    return Some([
        raw[base],
        raw[base + 1],
        raw[base + 2],
        raw[base + 3],
        raw[base + 4],
        raw[base + 5],
        raw[base + 6],
        raw[base + 7],
    ]);
}

pub async fn start_playback(key: PlaybackKey) {
    let decoder = DECODER.lock().await;
    decoder.borrow_mut().loader_key = Some(key);
    decoder.borrow_mut().cursor = 0;
    PLAYBACK_BUFFER_STATUS.lock(|state_ref| {
        let mut state = state_ref.borrow_mut();
        let _ = decode_task::spawn();
    })
}

pub fn pop_playback_sample() -> i16 {
    let samp = PLAYBACK_BUFFER_STATUS.lock(|state| {
        for _ in 0..=1 {
            let samp = match state.borrow().buffer {
                ActivePlaybackBuffer::Ping => PLAYBACK_PING.lock(|playback| {
                    if playback.borrow().cursor < 640 {
                        let samp = playback.borrow().buf[playback.borrow().cursor as usize / 2];
                        playback.borrow_mut().cursor += 1;
                        Some(samp)
                    } else {
                        None
                    }
                }),
                ActivePlaybackBuffer::Pong => PLAYBACK_PONG.lock(|playback| {
                    if playback.borrow().cursor < 640 {
                        let samp = playback.borrow().buf[playback.borrow().cursor as usize / 2];
                        playback.borrow_mut().cursor += 1;
                        Some(samp)
                    } else {
                        None
                    }
                }),
            };
            match samp {
                Some(samp) => {
                    return samp;
                }
                None => {
                    if state.borrow().active_playback {
                        defmt::debug!("Flipping audio buffers.");
                    }
                    let mut status = state.borrow_mut();
                    if status.buffer == ActivePlaybackBuffer::Ping {
                        status.buffer = ActivePlaybackBuffer::Pong;
                    } else {
                        status.buffer = ActivePlaybackBuffer::Ping;
                    }
                    let _ = decode_task::spawn();
                }
            }
        }
        if state.borrow().active_playback {
            defmt::warn!("Audio buffer underrun");
        }
        0
    });
    samp
}
