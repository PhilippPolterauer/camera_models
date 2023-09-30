impl<T, V> Projection<CameraRay> for CameraModel<T, V>
where
    T: CameraProjection<f64>,
    V: CameraDistortion,
{
    fn project(&self, rhs: &CameraRay) -> PixelIndex<f64> {
        let ray = self.distortion().distort(&rhs);
        self.projection().project(&ray)
    }
}
