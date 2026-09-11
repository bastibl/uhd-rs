//! Pinned UHD 4.8 image catalog. No build step downloads files.
use crate::{Error, Result, b2xx::Product};
use std::{borrow::Cow, collections::BTreeMap};

/// An image available for application overrides and (by default) embedding.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd)]
pub enum Image {
    Firmware,
    Bootloader,
    B200,
    B210,
    B200Mini,
    B205Mini,
}
impl Image {
    pub const ALL: [Self; 6] = [
        Self::Firmware,
        Self::Bootloader,
        Self::B200,
        Self::B210,
        Self::B200Mini,
        Self::B205Mini,
    ];
    pub const fn filename(self) -> &'static str {
        match self {
            Self::Firmware => "usrp_b200_fw.hex",
            Self::Bootloader => "usrp_b200_bl.img",
            Self::B200 => "usrp_b200_fpga.bin",
            Self::B210 => "usrp_b210_fpga.bin",
            Self::B200Mini => "usrp_b200mini_fpga.bin",
            Self::B205Mini => "usrp_b205mini_fpga.bin",
        }
    }
    pub const fn fpga(product: Product) -> Self {
        match product {
            Product::B200 => Self::B200,
            Product::B210 => Self::B210,
            Product::B200Mini => Self::B200Mini,
            Product::B205Mini => Self::B205Mini,
        }
    }
    /// UHD image identity hash for the pinned catalog, available without embedding.
    pub const fn pinned_hash(self) -> u32 {
        match self {
            Self::Firmware => 0x2e5ba544,
            Self::Bootloader => 0xdd95200c,
            Self::B200 => 0xfc7887aa,
            Self::B210 => 0xf6a60bdf,
            Self::B200Mini => 0x1df2a5ef,
            Self::B205Mini => 0x1624155e,
        }
    }
    pub fn embedded(self) -> Option<Cow<'static, [u8]>> {
        #[cfg(feature = "embedded-images")]
        {
            Assets::get(self.filename()).map(|file| file.data)
        }
        #[cfg(not(feature = "embedded-images"))]
        {
            None
        }
    }
}
#[cfg(feature = "embedded-images")]
#[derive(rust_embed::RustEmbed)]
#[folder = "images/assets/"]
struct Assets;

/// Application bytes take precedence over embedded images.
#[derive(Clone, Default, Debug)]
pub struct ImageCatalog {
    overrides: BTreeMap<Image, Vec<u8>>,
}
impl ImageCatalog {
    pub fn insert(&mut self, image: Image, bytes: impl Into<Vec<u8>>) {
        self.overrides.insert(image, bytes.into());
    }
    pub fn get(&self, image: Image) -> Result<Cow<'_, [u8]>> {
        self.overrides
            .get(&image)
            .map(|data| Cow::Borrowed(data.as_slice()))
            .or_else(|| image.embedded())
            .ok_or(Error::MissingImage(image.filename()))
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn overrides_win() {
        let mut c = ImageCatalog::default();
        c.insert(Image::Firmware, vec![1, 2]);
        assert_eq!(&*c.get(Image::Firmware).unwrap(), &[1, 2]);
    }
    #[cfg(feature = "embedded-images")]
    #[test]
    fn all_six_assets_embedded_and_firmware_valid() {
        for image in Image::ALL {
            assert!(!image.embedded().unwrap().is_empty());
            assert_eq!(
                crate::b2xx::image_hash(&image.embedded().unwrap()),
                image.pinned_hash()
            );
        }
        crate::ihex::parse(&Image::Firmware.embedded().unwrap()).unwrap();
    }
    #[cfg(not(feature = "embedded-images"))]
    #[test]
    fn missing_image_is_actionable() {
        assert!(matches!(
            ImageCatalog::default().get(Image::B200),
            Err(Error::MissingImage(_))
        ));
    }
}
