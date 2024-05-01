use alloc::string::String;
use sequential_storage::map::StorageItem;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LmrChannel {
    pub num: Option<u16>,
    pub freq: u32,
    pub name: String,

    #[serde(default = "default_false")]
    pub rxonly: bool,
}

fn default_false() -> bool {
    false
}

impl StorageItem for LmrChannel {
    type Key = u16;

    type Error = ();

    fn serialize_into(&self, buffer: &mut [u8]) -> Result<usize, Self::Error> {
        assert!(self.num.is_some());
        assert!(self.freq < 1_000_000_000);
        assert!(self.freq > 100_000_000);
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
        self.num.unwrap()
    }
}
