# Camera Models 
a rust library for working with common Camera Models. 

**Note**
This is a hobby project for learning rust. please do not expect production ready quality, although I would highly appreciate issues or other suggestion for improvements!


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