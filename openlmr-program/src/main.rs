use anyhow::{Context, Result};
use clap::Parser;
use dfu_libusb::{Dfu, DfuLibusb};
use embedded_storage::nor_flash::{ErrorType, ReadNorFlash};
use embedded_storage_inmemory::MemFlash;
use openlmr_codeplug::LmrCodeplug;
use sequential_storage::{cache::NoCache, map::store_item};
use std::{fs::File, io::Read};
use tokio::time::{sleep, Duration};

#[derive(Debug, Clone, Parser)]
struct Args {
    #[arg(long)]
    config: String,
}

pub fn parse_vid_pid(s: &str) -> Result<(u16, u16)> {
    let (vid, pid) = s
        .split_once(':')
        .context("could not parse VID/PID (missing `:')")?;
    let vid = u16::from_str_radix(vid, 16).context("could not parse VID")?;
    let pid = u16::from_str_radix(pid, 16).context("could not parse PID")?;

    Ok((vid, pid))
}

struct WrappedFlash {
    inner: MemFlash<65536, 4096, 1>,
}

impl ErrorType for WrappedFlash {
    type Error = <MemFlash<65536, 4096, 1> as ErrorType>::Error;
}

impl embedded_storage_async::nor_flash::ReadNorFlash for WrappedFlash {
    const READ_SIZE: usize = 1;

    async fn read(
        &mut self,
        offset: u32,
        bytes: &mut [u8],
    ) -> std::prelude::v1::Result<(), Self::Error> {
        self.inner.read(offset, bytes)
    }

    fn capacity(&self) -> usize {
        self.inner.capacity()
    }
}

impl embedded_storage_async::nor_flash::NorFlash for WrappedFlash {
    const WRITE_SIZE: usize = 1;

    const ERASE_SIZE: usize = 4096;

    async fn erase(&mut self, from: u32, to: u32) -> std::prelude::v1::Result<(), Self::Error> {
        let bytes = vec![0xFF; (to - from + 1) as usize];
        self.inner.program(from, &bytes)
    }

    async fn write(
        &mut self,
        offset: u32,
        bytes: &[u8],
    ) -> std::prelude::v1::Result<(), Self::Error> {
        self.inner.program(offset, &bytes)
    }
}

#[tokio::main]
async fn main() {
    let args = Args::parse();

    let mut config_str = String::new();
    File::open(args.config)
        .expect("Failed to open config file")
        .read_to_string(&mut config_str)
        .expect("Failed to read config file");

    let codeplug: LmrCodeplug = toml::from_str(&config_str).expect("Failed to parse config file");

    let flash_range = std::ops::Range {
        start: 0,
        end: 65536,
    };
    let mut flash = WrappedFlash {
        inner: embedded_storage_inmemory::MemFlash::new(0xFF),
    };

    let mut data_buffer = [0u8; 1024];

    let mut channel_num = 0u16;
    for channel in &codeplug.channels {
        let mut channel = channel.clone();
        match &channel.num {
            Some(requested_num) => {
                assert!(*requested_num >= channel_num);
                channel_num = *requested_num;
            }
            None => channel.num = Some(channel_num),
        }
        channel_num += 1;
        store_item(
            &mut flash,
            flash_range.clone(),
            NoCache::new(),
            &mut data_buffer,
            &channel,
        )
        .await
        .unwrap();
    }

    let context = rusb::Context::new().unwrap();

    let mut device: Dfu<rusb::Context> = DfuLibusb::open(&context, 0x0483, 0xdf11, 0, 0)
        .context("could not open device")
        .unwrap();


    sleep(Duration::from_millis(100)).await;

    device.usb_reset().unwrap();

    sleep(Duration::from_millis(100)).await;

    println!("Downloading to device");
    device.download_from_slice(&flash.inner.mem).unwrap();
}
