use camera_models::*;
use image::{GenericImage, ImageBuffer, Pixel, Rgb, RgbImage};
use itertools::{izip, Itertools};
use serde::Deserialize;
use std::array;
use std::iter::zip;
use std::mem::size_of;
use std::ops::Deref;
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

fn get_distorted_pixel_idx(
    PixelIndex(u, v): PixelIndex<u32>,
    camera: &CameraModel<Pinhole, PlumbBob>,
    desired: &Pinhole,
) -> PixelIndex<f64> {
    let ray = desired.unproject(&PixelIndex(u as f64, v as f64));
    // we compute the undistorted image by answering the question "where would our ideal ray have landed if we would have used the distortion?"
    // then we use this value to find the color value of the current pixel

    // we project the distorted ray into the image and crop it to the image size
    camera.project(&ray)
    // print!("pd = {:?}\n", pd);
}

fn compute_undistortion_map(
    resolution: (u32, u32),
    camera: &CameraModel<Pinhole, PlumbBob>,
    desired: &Pinhole,
) -> (Vec<u32>, Vec<u32>, Vec<u32>, Vec<u32>) {
    let (width, height) = resolution;
    let sz = width as usize * height as usize;
    let mut uvec: Vec<u32> = Vec::with_capacity(sz);
    let mut vvec: Vec<u32> = Vec::with_capacity(sz);
    let mut udvec: Vec<u32> = Vec::with_capacity(sz);
    let mut vdvec: Vec<u32> = Vec::with_capacity(sz);

    for (vo, uo) in (0..height).cartesian_product(0..width) {
        let PixelIndex(u, v) = get_distorted_pixel_idx(PixelIndex(uo, vo), camera, desired);
        let u = u.round();
        let v = v.round();
        if 0. <= u && u < width as f64 && 0. <= v && v < height as f64 {
            // version A
            uvec.push(uo);
            vvec.push(vo);
            udvec.push(u as u32);
            vdvec.push(v as u32);
        }
    }
    (uvec, vvec, udvec, vdvec)
}

fn undisort_forloop(
    img: &RgbImage,
    camera: &CameraModel<Pinhole, PlumbBob>,
    desired: &Pinhole,
) -> RgbImage {
    let mut res = RgbImage::new(img.width(), img.height());
    let num_channel = Rgb::<u8>::CHANNEL_COUNT;
    print!("size = {}\n", (res.as_raw()).len());
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
        let x = u.round() as u32;
        let y = v.round() as u32;
        if 0 < x && x < img.width() && 0 < y && y < img.height() {
            *px = *img.get_pixel(x, y);
        }
    }
    res
}

#[inline]
fn linear_index<P: Pixel, C: Deref<Target = [P::Subpixel]>>(
    x: u32,
    y: u32,
    img: &ImageBuffer<P, C>,
) -> usize {
    let width = img.width();
    (y * width + x) as usize
}

#[inline]
fn byte_index<P: Pixel, C: Deref<Target = [P::Subpixel]>>(
    x: u32,
    y: u32,
    img: &ImageBuffer<P, C>,
) -> usize {
    let width = img.width();
    let num_channel = P::CHANNEL_COUNT;
    ((y * width + x) * num_channel as u32) as usize
}

fn compute_undistortion_map_linidx(
    img: &RgbImage,
    camera: &CameraModel<Pinhole, PlumbBob>,
    desired: &Pinhole,
) -> (Vec<usize>, Vec<usize>) {
    let (width, height) = img.dimensions();
    let sz = width as usize * height as usize;
    let mut linidx: Vec<usize> = Vec::with_capacity(sz);
    let mut linidx_dist: Vec<usize> = Vec::with_capacity(sz);

    for (vo, uo) in (0..height).cartesian_product(0..width) {
        let PixelIndex(u, v) = get_distorted_pixel_idx(PixelIndex(uo, vo), camera, desired);
        let u = u.round();
        let v = v.round();
        if 0. <= u && u < width as f64 && 0. <= v && v < height as f64 {
            // version A

            linidx.push(linear_index(uo as u32, vo as u32, img));
            linidx_dist.push(linear_index(u as u32, v as u32, img))
        }
    }
    (linidx, linidx_dist)
}

fn compute_undistortion_map_byteidx(
    img: &RgbImage,
    camera: &CameraModel<Pinhole, PlumbBob>,
    desired: &Pinhole,
) -> (Vec<usize>, Vec<usize>) {
    let (width, height) = img.dimensions();
    let sz = width as usize * height as usize;
    let mut linidx: Vec<usize> = Vec::with_capacity(sz);
    let mut linidx_dist: Vec<usize> = Vec::with_capacity(sz);

    for (vo, uo) in (0..height).cartesian_product(0..width) {
        let PixelIndex(u, v) = get_distorted_pixel_idx(PixelIndex(uo, vo), camera, desired);
        let u = u.round();
        let v = v.round();
        if 0. <= u && u < width as f64 && 0. <= v && v < height as f64 {
            // version A

            linidx.push(byte_index(uo as u32, vo as u32, img));
            linidx_dist.push(byte_index(u as u32, v as u32, img))
        }
    }
    (linidx, linidx_dist)
}

fn undistort_precomputed(
    img: &RgbImage,
    (uvec, vvec, udvec, vdvec): &(Vec<u32>, Vec<u32>, Vec<u32>, Vec<u32>),
) -> RgbImage {
    let mut res = RgbImage::new(img.width(), img.height());
    for (u, v, ud, vd) in izip!(uvec, vvec, udvec, vdvec) {
        unsafe {
            // this operation is safe since validity was checked before
            // print!("u,v = {},{}\n", u, v);
            // print!("ud,vd = {},{}\n", ud, vd);
            res.unsafe_put_pixel(*u, *v, img[(*ud, *vd)]);
        }
    }
    res
}

fn undistort_precomputed_linidx(
    img: &RgbImage,
    (linidx, linidx_dist): &(Vec<usize>, Vec<usize>),
) -> RgbImage {
    let mut res = RgbImage::new(img.width(), img.height());
    let img_raw = img.as_raw();
    let num_channel = Rgb::<u8>::CHANNEL_COUNT as usize;
    let out = res.as_mut();
    for (idx, idx_dist) in izip!(linidx, linidx_dist) {
        let idx = *idx * num_channel;
        let idx_dist = *idx_dist * num_channel;
        out[idx] = img_raw[idx_dist];
        out[idx + 1] = img_raw[idx_dist + 1];
        out[idx + 2] = img_raw[idx_dist + 2];
    }
    res
}

fn undistort_precomputed_byteidx(
    img: &RgbImage,
    (linidx, linidx_dist): &(Vec<usize>, Vec<usize>),
) -> RgbImage {
    let mut res = RgbImage::new(img.width(), img.height());
    let img_raw = img.as_raw();
    // let num_channel = Rgb::<u8>::CHANNEL_COUNT as usize;
    let out = res.as_mut();
    for i in 0..linidx.len() {
        let idx = linidx[i];
        let idx_dist = linidx_dist[i];
        out[idx] = img_raw[idx_dist];
        out[idx + 1] = img_raw[idx_dist + 1];
        out[idx + 2] = img_raw[idx_dist + 2];
    }
    res
}

// next would be to use rayon to compute an undistortion

fn main() {
    let conf = load_config().unwrap();
    let img = image::open("tests/test.jpg").unwrap();
    let img = img.to_rgb8();

    let distortion = conf.distortion;
    let projection = Pinhole::from_resolution_fov((img.width(), img.height()), (90., 90.));
    let desired = Pinhole::from_resolution_fov((img.width(), img.height()), (90., 90.));
    let camera = CameraModel::new(projection, distortion);

    //

    let map = compute_undistortion_map((img.width(), img.height()), &camera, &desired);
    let map_linidx = compute_undistortion_map_linidx(&img, &camera, &desired);
    let map_byte = compute_undistortion_map_byteidx(&img, &camera, &desired);


    fn measure<F>(img: &RgbImage, function: F, name: &str)
    where
        F: for<'a> Fn(&'a RgbImage) -> RgbImage,
    {
        println!("Starting Image Conversion Precomputed: '{}'", name);
        let start_time = Instant::now();
        let res = function(&img);
        let end_time = Instant::now();
        let elapsed = end_time - start_time;
        println!("Elapsed: {:?}", elapsed);
        res.save(format!("results/{}.jpg", name)).unwrap()
    }

    let f1 = |x: &RgbImage| undisort_forloop(x, &camera, &desired);
    let f2 = |x: &RgbImage| undistort_precomputed(x, &map);
    let f3 = |x: &RgbImage| undistort_precomputed_linidx(x, &map_linidx);
    let f4 = |x: &RgbImage| undistort_precomputed_byteidx(x, &map_byte);

    measure(&img, f1, "undistort_forloop");
    measure(&img, f2, "undistort_precomputed");
    measure(&img, f3, "undistort_precomputed_linidx");
    measure(&img, f4, "undistort_precomputed_byteidx");
}
