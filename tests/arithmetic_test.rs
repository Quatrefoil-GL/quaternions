extern crate quaternions;

use quaternions::Quaternion;

/// Tests

/// Fudge factor for float equality checks
static EPSILON: f32 = std::f32::EPSILON;
static PI: f32 = std::f32::consts::PI;

#[test]
fn test_arithmetic() {
  assert!(Quaternion::new(1., 2., 3., 4.) + Quaternion::<f32>::new(5., 6., 7., 8.) == Quaternion::<f32>::new(6., 8., 10., 12.));
}

#[test]
fn test_euler_angle() {
  let q: Quaternion<f32> = Quaternion::from_euler_angles(PI, PI, PI);
  // Should be a unit quaternion
  assert!((q.square_length() - 1.0).abs() < EPSILON);
}
