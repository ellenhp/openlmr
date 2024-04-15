use embedded_graphics::{geometry::OriginDimensions, prelude::DrawTarget};

pub struct LockedDisplay<Target: DrawTarget + OriginDimensions> {
    target: Target,
}

impl<Target: DrawTarget + OriginDimensions> DrawTarget for LockedDisplay<Target> {
    type Color = Target::Color;
    type Error = Target::Error;

    fn fill_contiguous<I>(
        &mut self,
        area: &embedded_graphics::primitives::Rectangle,
        colors: I,
    ) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Self::Color>,
    {
        self.target.fill_contiguous(area, colors)
    }

    fn fill_solid(
        &mut self,
        area: &embedded_graphics::primitives::Rectangle,
        color: Self::Color,
    ) -> Result<(), Self::Error> {
        self.target.fill_solid(area, color)
    }

    fn clear(&mut self, color: Self::Color) -> Result<(), Self::Error> {
        self.target.clear(color)
    }

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = embedded_graphics::prelude::Pixel<Self::Color>>,
    {
        self.target.draw_iter(pixels)
    }
}

impl<Target: DrawTarget + OriginDimensions> OriginDimensions for LockedDisplay<Target> {
    fn size(&self) -> embedded_graphics::prelude::Size {
        self.target.size()
    }
}

impl<Target: DrawTarget + OriginDimensions> Drop for LockedDisplay<Target> {
    fn drop(&mut self) {}
}
