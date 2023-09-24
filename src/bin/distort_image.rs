use camera_models::*;
use image::{RgbImage, GenericImage};
use itertools::{izip, Itertools};
use serde::Deserialize;
use std::array;
use std::iter::zip;
use std::time::Instant;
use toml;
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

use std::fs::File;
use std::io::Read;

#[derive(Debug, Deserialize)]
struct AppConfig {
    distortion: PlumbBob,
}

fn load_config() -> Result<AppConfig, toml::de::Error> {
    let mut file = File::open("config.toml").unwrap();
    let mut contents = String::new();
    file.read_to_string(&mut contents).unwrap();

    toml::from_str(&contents)
}

fn main() {
    let conf = load_config().unwrap();
    let img = image::open("tests/test.jpg").unwrap();
    let img = img.to_rgb8();
    let mut res = RgbImage::new(img.width(), img.height());
    let mut res2 = RgbImage::new(img.width(), img.height());

    let distortion = conf.distortion;
    let projection = Pinhole::from_resolution_fov((img.width(), img.height()), (90., 90.));
    let desired = Pinhole::from_resolution_fov((img.width(), img.height()), (80., 80.));
    let camera = CameraModel::new(projection, distortion);

    // precompute undistort

    fn get_distorted_pixel_idx(
        PixelIndex(u, v): PixelIndex<u32>,
        camera: &CameraModel<Pinhole, PlumbBob>,
        desired: &Pinhole,
    ) -> PixelIndex<u32> {
        let ray = desired.unproject(&PixelIndex(u as f64, v as f64));
        // we compute the undistorted image by answering the question "where would our ideal ray have landed if we would have used the distortion?"

        // we project the distorted ray into the image and crop it to the image size
        let uv = camera.project(&ray);
        // print!("pd = {:?}\n", pd);

        // version A
        PixelIndex(uv.0.round() as u32, uv.1.round() as u32)
    }

    fn compute_undistortion_map(
        resolution: (u32, u32),
        camera: &CameraModel<Pinhole, PlumbBob>,
        desired: &Pinhole,
    ) -> (Vec<u32>, Vec<u32>, Vec<u32>, Vec<u32>, Vec<bool>) {
        let (width, height) = resolution;
        let sz = width as usize * height as usize;
        let mut uvec: Vec<u32> = Vec::with_capacity(sz);
        let mut vvec: Vec<u32> = Vec::with_capacity(sz);
        let mut udvec: Vec<u32> = Vec::with_capacity(sz);
        let mut vdvec: Vec<u32> = Vec::with_capacity(sz);
        let mut valids: Vec<bool> = Vec::with_capacity(sz);

        for (u, v) in (0..width).cartesian_product(0..height) {
            uvec.push(u);
            vvec.push(v);
            let PixelIndex(u, v) = get_distorted_pixel_idx(PixelIndex(u, v), camera, desired);
            let valid = u < width && v < height;
            udvec.push(u);
            vdvec.push(v);
            valids.push(valid);
        }
        (uvec, vvec, udvec, vdvec, valids)
    }

    println!("Starting Image Conversion");
    let start_time = Instant::now();
    for (u, v, px) in res.enumerate_pixels_mut() {
        // compute the distorted location

        // given a Pinhole Camera model without skew the ray from the camera center to the pixel is given by
        // camera model with skew is given by
        // [fx s cx] [x]   [u]
        // [0 fy cy] [y] = [v]
        //           [1]
        // y = (v - cy )/fy
        // x = (u - cx - s*y)/fx

        // print!("u, v = {}, {}\n", u, v);
        // the problem is that we need to send in x' and y' from https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html#ga7dfb72c9cf9780a347fbe3d1c47e5d5a

        let PixelIndex(u, v) = get_distorted_pixel_idx(PixelIndex(u, v), &camera, &desired);
        // clamp to image size
        // print!("u, v = {}, {}\n", u, v);
        if 0 < u && u < img.width() && 0 < v && v < img.height() {
            *px = *img.get_pixel(u, v);
        }
    }
    let end_time = Instant::now();
    let elapsed = end_time - start_time;
    println!("Elapsed: {:?}", elapsed);
    //

    let (uvec, vvec, udvec, vdvec, valids) = compute_undistortion_map((img.width(), img.height()), &camera, &desired);
    println!("Starting Image Conversion Precomputed");
    let start_time = Instant::now();

    for (u, v, ud, vd, valid) in izip!(uvec, vvec, udvec, vdvec, valids) {
        if valid {
            unsafe{
                // this operation is safe since validity was checked before
                res2.unsafe_put_pixel(u, v, *img.get_pixel(ud, vd))
            }
        }
    }

    let end_time = Instant::now();
    let elapsed = end_time - start_time;
    println!("Elapsed: {:?}", elapsed);

    res.save("results/distorted.jpg").unwrap();
}
