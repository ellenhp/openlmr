use usb_device::prelude::*;
use usbd_dfu::*;

// DFUClass will use MyMem to actually read, erase or program the memory.
// Here, a set of constant parameters must be set. These parameters
// either change how DFUClass behaves, or define host's expectations.

pub struct MyMem {
    pub buffer: [u8; 64],
    pub flash_memory: [u8; 1024],
}

impl DFUMemIO for MyMem {
    const MEM_INFO_STRING: &'static str = "@Flash/0x00000000/1*1Kg";
    const INITIAL_ADDRESS_POINTER: u32 = 0x0;
    const PROGRAM_TIME_MS: u32 = 8;
    const ERASE_TIME_MS: u32 = 50;
    const FULL_ERASE_TIME_MS: u32 = 50;
    const TRANSFER_SIZE: u16 = 64;

    fn read(&mut self, address: u32, length: usize) -> Result<&[u8], DFUMemError> {
        // TODO: check address value
        let offset = address as usize;
        Ok(&self.flash_memory[offset..offset + length])
    }

    fn erase(&mut self, address: u32) -> Result<(), DFUMemError> {
        // TODO: check address value
        self.flash_memory.fill(0xff);
        // TODO: verify that block is erased successfully
        Ok(())
    }

    fn erase_all(&mut self) -> Result<(), DFUMemError> {
        // There is only one block, erase it.
        self.erase(0)
    }

    fn store_write_buffer(&mut self, src: &[u8]) -> Result<(), ()> {
        self.buffer[..src.len()].copy_from_slice(src);
        Ok(())
    }

    fn program(&mut self, address: u32, length: usize) -> Result<(), DFUMemError> {
        // TODO: check address value
        let offset = address as usize;

        // Write buffer to a memory
        self.flash_memory[offset..offset + length].copy_from_slice(&self.buffer[..length]);

        // TODO: verify that memory is programmed correctly
        Ok(())
    }

    fn manifestation(&mut self) -> Result<(), DFUManifestationError> {
        // Nothing to do to activate FW
        Ok(())
    }
}
