# Camera Models 
a rust library for working with common Camera Models. 

> **Note**
> This is a hobby project for learning rust. please do not expect production ready quality, although I would highly appreciate issues or other suggestion for improvements!


## Goal
A library which shall allow undistorting images from common camera models. We will try to reach speed comparibly to OpenCV.


## Projection models
- [x] Pinhole 
- [ ] Equidistant
- [ ] Stereographic

## Distortion models
- [x] Brown-Conrady model, Plumb-Bob
- [ ] Polynomial
- [ ] Polynomial Fisheye

## Infrastructure
- [ ] OpenCV Yaml loading
- [ ] Benchmark Suite
- [ ] Python Bindings

## Renderer
- [ ] Frame rendering 
- [ ] a simple scene renderer for testing the camera projections


## References
- http://16385.courses.cs.cmu.edu/spring2022content/lectures/09_cameras/09_cameras_slides.pdf


## Performance analysis for Undistortion

```console
$ cargo run --release --example undistort_benchmark
camera_matrix = Matrix { data: [1244.617161547647, 0.0, 0.0, 930.993392665601, 2016.0, 1508.0] }
projection = Pinhole { fx: 1244.617161547647, fy: 930.993392665601, cx: 2016.0, cy: 1508.0, skew: 0.0 }
distortion = PlumbBob { k1: 0.1, k2: 0.1, p1: 0.2, p2: 0.1, k3: 0.0 }
width = 4032
height = 3016
Starting Image Conversion Precomputed: 'undistort_forloop'
size = 36481536
Elapsed: 349.242845ms
Starting Image Conversion Precomputed: 'undistort_precomputed'
Elapsed: 28.74842ms
Starting Image Conversion Precomputed: 'undistort_precomputed_linidx'
Elapsed: 25.57148ms
Starting Image Conversion Precomputed: 'undistort_precomputed_byteidx'
Elapsed: 25.219873ms
Starting Image Conversion Precomputed: 'undistort_precomputed_rayon'
Elapsed: 8.621929ms
Starting Image Conversion Precomputed: 'undistort_precomputed_rayon_rows'
Elapsed: 11.293844ms
```
