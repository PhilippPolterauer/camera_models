// use common rust image package to read and write images
use image::{ImageBuffer, RgbImage, Rgb, ImageDe};

fn read_image(path: &str) -> RgbImage {
    image::open(path).
}