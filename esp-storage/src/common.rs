use core::{
    mem::{self, MaybeUninit},
    ops::{Deref, DerefMut},
};

use crate::chip_specific;

#[repr(C, align(4))]
pub struct FlashSectorBuffer {
    // NOTE: Ensure that no unaligned fields are added above `data` to maintain its required
    // alignment
    data: [MaybeUninit<u8>; FlashStorage::SECTOR_SIZE as usize],
}

impl FlashSectorBuffer {
    pub const fn uninit() -> Self {
        Self {
            data: [MaybeUninit::uninit(); FlashStorage::SECTOR_SIZE as usize],
        }
    }

    pub unsafe fn assume_init_mut(&mut self) -> &mut [u8; FlashStorage::SECTOR_SIZE as usize] {
        &mut *self.data.as_mut_ptr().cast()
    }
}

impl Deref for FlashSectorBuffer {
    type Target = [MaybeUninit<u8>; FlashStorage::SECTOR_SIZE as usize];

    fn deref(&self) -> &Self::Target {
        &self.data
    }
}

impl DerefMut for FlashSectorBuffer {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.data
    }
}

#[derive(Debug)]
#[non_exhaustive]
pub enum FlashStorageError {
    IoError,
    IoTimeout,
    CantUnlock,
    NotAligned,
    OutOfBounds,
    Other(i32),
}

#[inline(always)]
pub fn check_rc(rc: i32) -> Result<(), FlashStorageError> {
    match rc {
        0 => Ok(()),
        1 => Err(FlashStorageError::IoError),
        2 => Err(FlashStorageError::IoTimeout),
        _ => Err(FlashStorageError::Other(rc)),
    }
}

#[derive(Debug)]
pub struct FlashStorage {
    pub(crate) capacity: usize,
    unlocked: bool,
}

impl Default for FlashStorage {
    fn default() -> Self {
        Self::new()
    }
}

impl FlashStorage {
    pub const WORD_SIZE: u32 = 4;
    pub const SECTOR_SIZE: u32 = 4096;

    pub fn new() -> FlashStorage {
        let mut storage = FlashStorage {
            capacity: 0,
            unlocked: false,
        };

        #[cfg(not(any(feature = "esp32", feature = "esp32s2")))]
        const ADDR: u32 = 0x0000;
        #[cfg(any(feature = "esp32", feature = "esp32s2"))]
        const ADDR: u32 = 0x1000;

        let mut buffer = [MaybeUninit::new(0u8); 8];
        storage.internal_read(ADDR, &mut buffer).ok();
        let buffer = unsafe { mem::transmute::<_, [u8; 8]>(buffer) };
        let mb = match buffer[3] & 0xf0 {
            0x00 => 1,
            0x10 => 2,
            0x20 => 4,
            0x30 => 8,
            0x40 => 16,
            _ => 0,
        };
        storage.capacity = mb * 1024 * 1024;

        storage
    }

    /// Read bytes from the flash storage into the provided buffer, which can be uninitialized.
    /// The buffer must be aligned to `WORD_SIZE` and the length must be a multiple of `WORD_SIZE`.
    pub fn read_uninit(
        &mut self,
        offset: u32,
        bytes: &mut [MaybeUninit<u8>],
    ) -> Result<(), FlashStorageError> {
        self.check_bounds(offset, bytes.len())?;
        self.check_alignment::<{ Self::WORD_SIZE }>(offset, bytes.len())?;
        self.internal_read(offset, bytes)
    }

    /// Read bytes from the flash storage into the provided buffer.
    /// The buffer must be aligned to `WORD_SIZE` and the length must be a multiple of `WORD_SIZE`.
    pub fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), FlashStorageError> {
        // SAFETY: Transmuting to `MaybeUninit` is safe because `bytes` is initialized.
        let bytes = unsafe { mem::transmute(bytes) };
        self.read_uninit(offset, bytes)
    }

    /// Write bytes to the flash storage from the provided buffer.
    /// The buffer must be aligned to `WORD_SIZE` and the length must be a multiple of `WORD_SIZE`.
    pub fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), FlashStorageError> {
        self.check_bounds(offset, bytes.len())?;
        self.check_alignment::<{ Self::WORD_SIZE }>(offset, bytes.len())?;
        self.internal_write(offset, bytes)
    }

    #[inline(always)]
    pub(crate) fn check_alignment<const ALIGN: u32>(
        &self,
        offset: u32,
        length: usize,
    ) -> Result<(), FlashStorageError> {
        let offset = offset as usize;
        if offset % ALIGN as usize != 0 || length % ALIGN as usize != 0 {
            return Err(FlashStorageError::NotAligned);
        }
        Ok(())
    }

    #[inline(always)]
    pub(crate) fn check_bounds(&self, offset: u32, length: usize) -> Result<(), FlashStorageError> {
        let offset = offset as usize;
        if length > self.capacity || offset > self.capacity - length {
            return Err(FlashStorageError::OutOfBounds);
        }
        Ok(())
    }

    #[allow(clippy::all)]
    #[inline(never)]
    #[link_section = ".rwtext"]
    pub(crate) fn internal_read(
        &mut self,
        offset: u32,
        bytes: &mut [MaybeUninit<u8>],
    ) -> Result<(), FlashStorageError> {
        check_rc(chip_specific::esp_rom_spiflash_read(
            offset,
            bytes.as_mut_ptr() as *mut u32,
            bytes.len() as u32,
        ))
    }

    #[inline(always)]
    fn unlock_once(&mut self) -> Result<(), FlashStorageError> {
        if !self.unlocked {
            if chip_specific::esp_rom_spiflash_unlock() != 0 {
                return Err(FlashStorageError::CantUnlock);
            }
            self.unlocked = true;
        }
        Ok(())
    }

    #[inline(never)]
    #[link_section = ".rwtext"]
    pub(crate) fn internal_erase(&mut self, sector: u32) -> Result<(), FlashStorageError> {
        self.unlock_once()?;

        check_rc(chip_specific::esp_rom_spiflash_erase_sector(sector))
    }

    #[inline(never)]
    #[link_section = ".rwtext"]
    pub(crate) fn internal_write(
        &mut self,
        offset: u32,
        bytes: &[u8],
    ) -> Result<(), FlashStorageError> {
        self.unlock_once()?;

        check_rc(chip_specific::esp_rom_spiflash_write(
            offset,
            bytes.as_ptr() as *const u32,
            bytes.len() as u32,
        ))
    }
}
