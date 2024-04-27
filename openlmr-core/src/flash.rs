use core::cell::RefCell;

use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex, once_lock::OnceLock,
};
use embedded_hal::spi::ErrorType;
use embedded_hal_async::delay::DelayNs;
use embedded_storage_async::nor_flash::{self, NorFlash, ReadNorFlash};
use rtic_monotonics::systick::ExtU32;
use spi_memory_async::series25::{self, Flash, FlashParameters};
use stm32f4xx_hal::{
    gpio::{self, Output, PushPull},
    pac::SPI1,
    spi::Spi,
};

static FLASH_SPI: OnceLock<RefCell<Option<SpiFlashDevice>>> = OnceLock::new();

static FLASH: OnceLock<
    Mutex<CriticalSectionRawMutex, Flash<SpiFlashDevice, LmrFlashParams, LmrDelay>>,
> = OnceLock::new();

struct LmrDelay {}

impl DelayNs for LmrDelay {
    async fn delay_ns(&mut self, ns: u32) {
        crate::Mono::delay(ns.nanos().into()).await;
    }
}

struct LmrFlashParams {}

impl FlashParameters for LmrFlashParams {
    const PAGE_SIZE: usize = 256;
    const SECTOR_SIZE: usize = 4096;
    // Less error-prone to just type out the multiplication for the bigger powers of two.
    const BLOCK_SIZE: usize = 64 * 1024;
    const CHIP_SIZE: usize = 16 * 1024 * 1024;
}

struct SpiFlashDevice {
    spi: Spi<SPI1>,
    cs: gpio::Pin<'D', 7, Output>,
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

impl nor_flash::ErrorType for LockedFlash<'_> {
    type Error = spi_memory_async::Error<SpiFlashDevice>;
}

impl NorFlash for LockedFlash<'_> {
    const WRITE_SIZE: usize = 1;

    const ERASE_SIZE: usize = 4096;

    async fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        self.guard.erase(from, to).await
    }

    async fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        self.guard.write(offset, bytes).await
    }
}

impl ReadNorFlash for LockedFlash<'_> {
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
    let device = SpiFlashDevice { spi, cs };
    FLASH_SPI.get_or_init(|| RefCell::new(Some(device)));
}

pub async fn init_flash_once() {
    let device = FLASH_SPI.get().await.take().unwrap();
    let flash = Flash::init(device, LmrDelay {}, 10_000, LmrFlashParams {})
        .await
        .unwrap();
    FLASH.get_or_init(|| Mutex::new(flash));
}

pub async fn print_flash_params() {
    let mut flash = FLASH.get().await.lock().await;
    let id = flash.read_jedec_id().await.unwrap();
    defmt::info!("JECEC ID: {}", id);
}
