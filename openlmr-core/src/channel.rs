use defmt::Format;
use sequential_storage::map::StorageItem;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Format, Serialize, Deserialize)]
pub struct LmrChannel {
    pub freq: u32,
    pub num: u16,
}

impl StorageItem for LmrChannel {
    type Key = u16;

    type Error = ();

    fn serialize_into(&self, buffer: &mut [u8]) -> Result<usize, Self::Error> {
        if let Ok(used) = postcard::to_slice(self, buffer) {
            return Ok(used.len());
        } else {
            return Err(());
        }
    }

    fn deserialize_from(buffer: &[u8]) -> Result<Self, Self::Error>
    where
        Self: Sized,
    {
        postcard::from_bytes(buffer).map_err(|_| ())
    }

    fn key(&self) -> Self::Key {
        self.num
    }
}
