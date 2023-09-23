use camera_models::{CameraDistortion, Fisheye, ImageIndex, Pinhole, PlumbBob, CameraProjection};
use image::{GenericImageView, ImageBuffer, ImageDecoder, Rgb, RgbImage};
// load image

// fn interpolate_pixel(px1 ,) -> Rgb<u8> {
//     let x0 = x.floor() as u32;
//     let x1 = x0 + 1;
//     let y0 = y.floor() as u32;
//     let y1 = y0 + 1;
//     let x0y0 = img.get_pixel(x0, y0);
//     let x1y0 = img.get_pixel(x1, y0);
//     let x0y1 = img.get_pixel(x0, y1);
//     let x1y1 = img.get_pixel(x1, y1);
//     let x0y0 = Rgb::from(x0y0);
//     let x1y0 = Rgb::from(x1y0);
//     let x0y1 = Rgb::from(x0y1);
//     let x1y1 = Rgb::from(x1y1);
//     let x0 = x0 as f64;
//     let x1 = x1 as f64;
//     let y0 = y0 as f64;
//     let y1 = y1 as f64;
//     let x = x - x0;
//     let y = y - y0;
//     let x0y0 = x0y0.map(|c| c as f64);
//     let x1y0 = x1y0.map(|c| c as f64);
//     let x0y1 = x0y1.map(|c| c as f64);
//     let x1y1 = x1y1.map(|c| c as f64);
//     let x0y0 = x0y0 * (1.0 - x) * (1.0 - y);
//     let x1y0 = x1y0 * x * (1.0 - y);
//     let x0y1 = x0y1 * (1.0 - x) * y;
//     let x1y1 = x1y1 * x * y;
//     let r = x0y0[0] + x1y0[0] + x0y1[0] + x1y1[0];
//     let g = x0y0[1] + x1y0[1] + x0y1[1] + x1y1[1];
//     let b =

fn set_nearest_pixel(img: &mut RgbImage, x: f64, y: f64, src: &RgbImage) {
    let u = x.round() as u32;
    let v = y.round() as u32;
    // clamp to image size
    if u < img.width() && v < img.height() {
        img.put_pixel(u, v, src.get_pixel(u, v).clone())
    }
}

fn main() {
    let img = image::open("test.jpg").unwrap();
    let img = img.to_rgb8();
    let mut res = RgbImage::new(img.width(), img.height());

    let distortion = PlumbBob::new(0.0001, 0.0001, 0.0, 0.00001, 0.0);
    let ideal = Pinhole::from_resolution_fov((img.width(), img.height()), (90., 90.));
    for (u, v, px) in res.enumerate_pixels_mut() {
        // compute the distorted location

        // given a Pinhole Camera model without skew the ray from the camera center to the pixel is given by
        // camera model with skew is given by
        // [fx s cx] [x]   [u]
        // [0 fy cy] [y] = [v]
        //           [1]
        // y = (v - cy )/fy
        // x = (u - cx - s*y)/fx

        print!("u, v = {}, {}\n", u, v);
        // the problem is that we need to send in x' and y' from https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html#ga7dfb72c9cf9780a347fbe3d1c47e5d5a

        let ray = ideal.unproject(&ImageIndex(u, v).into());
        let pd = distortion.distort(&ray);
        let uv = ideal.project(&pd);
        print!("pd = {:?}\n", pd);

        let ImageIndex(u, v):ImageIndex<u32> = uv.into();
        // clamp to image size
        print!("u, v = {}, {}\n", u, v);
        if u < img.width() && v < img.height() {
            *px = img.get_pixel(u, v).clone();
        }
    }

    res.save("distorted.jpg").unwrap();
}
