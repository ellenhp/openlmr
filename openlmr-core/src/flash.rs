use core::{borrow::BorrowMut, cell::RefCell, time::Duration};

use alloc::vec;
use defmt::Format;
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex, once_lock::OnceLock,
};
use embedded_hal::{delay::DelayNs as DelayNsSync, spi::ErrorType};
use embedded_hal_async::delay::DelayNs;
use openlmr_codeplug::channel::LmrChannel;
use rtic_monotonics::{stm32::Tim2, systick::ExtU32};
use sequential_storage::{cache::NoCache, map::fetch_item};
use spi_memory_async::series25::{self, Flash, FlashParameters};
use stm32f4xx_hal::{
    gpio::{self, Output, PushPull},
    pac::SPI1,
    spi::Spi,
};
use usbd_dfu::{DFUMemError, DFUMemIO};

static FLASH_SPI: OnceLock<RefCell<Option<SpiFlashDevice>>> = OnceLock::new();

static FLASH: OnceLock<
    Mutex<CriticalSectionRawMutex, RefCell<Flash<SpiFlashDevice, LmrFlashParams, LmrDelay>>>,
> = OnceLock::new();

struct LmrDelay {}

impl DelayNs for LmrDelay {
    async fn delay_ns(&mut self, ns: u32) {
        crate::Mono::delay(ns.nanos().into()).await;
    }
}

#[derive(Debug, Format, Clone)]
pub struct LmrFlashError {}

impl From<LmrFlashError> for spi_flash::Error {
    fn from(err: LmrFlashError) -> spi_flash::Error {
        spi_flash::Error::Access
    }
}

impl From<stm32f4xx_hal::spi::Error> for LmrFlashError {
    fn from(err: stm32f4xx_hal::spi::Error) -> LmrFlashError {
        LmrFlashError {}
    }
}

struct LmrFlashParams {}

impl FlashParameters for LmrFlashParams {
    const PAGE_SIZE: usize = 128;
    const SECTOR_SIZE: usize = 4096;
    // Less error-prone to just type out the multiplication for the bigger powers of two.
    const BLOCK_SIZE: usize = 64 * 1024;
    const CHIP_SIZE: usize = 16 * 1024 * 1024;
}

pub struct SpiFlashDevice {
    spi: Spi<SPI1>,
    cs: gpio::Pin<'D', 7, Output>,
    buffer: [u8; 128],
}

impl SpiFlashDevice {
    pub fn new(spi: Spi<SPI1>, cs: gpio::Pin<'D', 7, Output>) -> SpiFlashDevice {
        SpiFlashDevice {
            spi,
            cs,
            buffer: [0u8; 128],
        }
    }
}

impl DFUMemIO for SpiFlashDevice {
    const INITIAL_ADDRESS_POINTER: u32 = 0;
    const MEM_INFO_STRING: &'static str = "@Flash/0x00000000/64*4Kg";
    const PROGRAM_TIME_MS: u32 = 8;
    const ERASE_TIME_MS: u32 = 5;
    const FULL_ERASE_TIME_MS: u32 = 50;
    const TRANSFER_SIZE: u16 = 128;

    fn store_write_buffer(&mut self, src: &[u8]) -> Result<(), ()> {
        self.buffer[..src.len()].copy_from_slice(src);
        Ok(())
    }

    fn read(&mut self, address: u32, length: usize) -> Result<&[u8], usbd_dfu::DFUMemError> {
        assert!(length as u16 <= Self::TRANSFER_SIZE);
        let mut flash = spi_flash::Flash::new(self);
        flash.set_erase_opcode(0x20);
        flash.set_erase_size(4096);
        let buf = match flash.read(address, length) {
            Ok(buf) => buf,
            Err(_) => {
                defmt::warn!("Error during read.");
                return Err(DFUMemError::Unknown);
            }
        };
        self.buffer.copy_from_slice(&buf);
        return Ok(&self.buffer);
    }

    fn program(&mut self, address: u32, length: usize) -> Result<(), usbd_dfu::DFUMemError> {
        let buf = self.buffer.to_vec();
        let mut flash = spi_flash::Flash::new(self);
        flash.set_erase_opcode(0x20);
        flash.set_erase_size(4096);
        if let Err(err) = flash.program(address, &buf[..length], true) {
            panic!();
            defmt::warn!("Error during program.");
            return Err(DFUMemError::Prog);
        }
        Ok(())
    }

    fn erase(&mut self, address: u32) -> Result<(), usbd_dfu::DFUMemError> {
        if address % 4096 != 0 {
            return Err(usbd_dfu::DFUMemError::Address);
        }
        let mut flash = spi_flash::Flash::new(self);
        flash.set_erase_opcode(0x20);
        flash.set_erase_size(4096);
        let page = [0xFF; 4096];
        if let Err(err) = flash.program(address, &page, false) {
            panic!();

            defmt::warn!("Error during erase cycle.");
            return Err(DFUMemError::Erase);
        }
        Ok(())
    }

    fn erase_all(&mut self) -> Result<(), usbd_dfu::DFUMemError> {
        let mut flash = spi_flash::Flash::new(self);
        flash.set_erase_opcode(0x20);
        flash.set_erase_size(4096);
        match flash.erase() {
            Ok(_) => Ok(()),
            Err(err) => Err(DFUMemError::Erase),
        }
    }

    fn manifestation(&mut self) -> Result<(), usbd_dfu::DFUManifestationError> {
        // No-op.
        Ok(())
    }
}

impl spi_flash::FlashAccess for SpiFlashDevice {
    type Error = LmrFlashError;

    fn exchange(&mut self, data: &[u8]) -> core::result::Result<alloc::vec::Vec<u8>, Self::Error> {
        self.cs.set_low();
        self.delay(Duration::from_micros(10));
        let mut rx = vec![0; data.len()];
        let result = self.spi.transfer(&mut rx, data);
        self.cs.set_high();
        result.unwrap();
        Ok(rx)
    }

    fn delay(&mut self, duration: Duration) {
        crate::Mono.delay_ns(duration.as_nanos() as u32);
    }
}

impl ErrorType for SpiFlashDevice {
    type Error = stm32f4xx_hal::spi::Error;
}

impl series25::SpiDevice for SpiFlashDevice {
    async fn transaction(
        &mut self,
        operations: &mut [series25::Operation<'_, u8>],
    ) -> Result<(), Self::Error> {
        self.cs.set_low();
        for operation in operations {
            let err = match operation {
                series25::Operation::Read(buf) => self.spi.read(*buf).err(),
                series25::Operation::Write(buf) => self.spi.write(*buf).err(),
                series25::Operation::Transfer(read, write) => {
                    self.spi.transfer(*read, *write).err()
                }
                series25::Operation::TransferInPlace(xfer) => {
                    self.spi.transfer_in_place(xfer).err()
                }
                series25::Operation::DelayNs(ns) => {
                    crate::Mono::delay(ns.nanos().into()).await;
                    None
                }
            };
            if let Some(err) = err {
                self.cs.set_high();
                return Err(err);
            }
        }
        self.cs.set_high();
        Ok(())
    }
}

struct LockedFlash<'a> {
    guard: embassy_sync::mutex::MutexGuard<
        'a,
        CriticalSectionRawMutex,
        Flash<SpiFlashDevice, LmrFlashParams, LmrDelay>,
    >,
}

impl embedded_storage_async::nor_flash::ErrorType for LockedFlash<'_> {
    type Error = spi_memory_async::Error<SpiFlashDevice>;
}

impl embedded_storage_async::nor_flash::NorFlash for LockedFlash<'_> {
    const WRITE_SIZE: usize = 1;

    const ERASE_SIZE: usize = 4096;

    async fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        self.guard.erase(from, to).await
    }

    async fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        self.guard.write(offset, bytes).await
    }
}

impl embedded_storage_async::nor_flash::ReadNorFlash for LockedFlash<'_> {
    const READ_SIZE: usize = 1;

    async fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        self.guard.read(offset, bytes).await
    }

    fn capacity(&self) -> usize {
        self.guard.capacity()
    }
}

pub fn create_flash_once(spi: Spi<SPI1>, mut cs: gpio::PD7<Output<PushPull>>) {
    cs.set_high();
    let device = SpiFlashDevice {
        spi,
        cs,
        buffer: [0u8; 128],
    };
    FLASH_SPI.get_or_init(|| RefCell::new(Some(device)));
}

pub async fn init_flash_once() -> bool {
    let device = if let Some(device) = FLASH_SPI.get_or_init(|| RefCell::new(None)).take() {
        device
    } else {
        defmt::warn!("Flash not created.");
        return false;
    };
    let flash = Flash::init(device, LmrDelay {}, 1_000, LmrFlashParams {})
        .await
        .unwrap();
    FLASH.get_or_init(|| Mutex::new(RefCell::new(flash)));
    true
}

pub async fn get_channel(key: u16) -> Option<LmrChannel> {
    let mut flash = FLASH.get().await.lock().await;
    let mut data_buffer = vec![0; 1024];
    let channel: Option<LmrChannel> = fetch_item(
        flash.get_mut(),
        0..65536,
        NoCache::new(),
        &mut data_buffer,
        key,
    )
    .await
    .unwrap();
    channel
}
