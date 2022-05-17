extern crate quaternions;

use quaternions::{q, qi, Quaternion};

/// Tests

/// Fudge factor for float equality checks
static EPSILON: f32 = std::f32::EPSILON;
static PI: f32 = std::f32::consts::PI;

#[test]
fn test_arithmetic() {
  assert_eq!(qi::<f32>(1, 2, 3, 4) + qi(5, 6, 7, 8), qi(6, 8, 10, 12));
  assert_eq!(qi::<f32>(1, 2, 3, 4) - qi(5, 6, 7, 8), qi(-4, -4, -4, -4));
  assert_eq!(qi::<f32>(1, 2, 3, 4).scale(1.5), q(1.5, 3., 4.5, 6.)); // float number comparing has problems

  // div depends on order

  assert_eq!(qi::<f32>(1, 2, 3, 4) * qi(5, 6, 7, 8), qi::<f32>(-60, 12, 30, 24));
  assert_eq!(qi::<f64>(-60, 12, 30, 24) / qi(5, 6, 7, 8), qi::<f64>(1, 2, 3, 4));

  assert_eq!(qi(5, 6, 7, 8) * qi::<f32>(1, 2, 3, 4), qi::<f32>(-60, 20, 14, 32));
  assert_eq!(qi::<f32>(-60, 20, 14, 32) / qi::<f32>(1, 2, 3, 4), qi(5, 6, 7, 8));

  assert_eq!(qi::<f32>(1, 2, 3, 4).conjugate(), qi::<f32>(1, -2, -3, -4));
  assert_eq!(qi::<f32>(1, 2, 3, 4).square_length(), 30.);
  assert_eq!(qi::<f32>(1, 2, 3, 4).length(), (30.0_f32).sqrt());

  assert_eq!(qi::<f32>(1, 2, 3, 4).inverse() * qi(1, 2, 3, 4), qi::<f32>(1, 0, 0, 0));
  assert_eq!(Quaternion::id(), qi::<f32>(1, 0, 0, 0));

  assert_eq!(Quaternion::default(), qi::<f32>(0, 0, 0, 0));

  assert_eq!(qi::<f32>(1, 2, 3, 4).dot(&qi::<f32>(5, 6, 7, 8)), 70.);

  let a = qi::<f64>(1, 2, 3, 4);
  assert_eq!(-a, qi::<f64>(-1, -2, -3, -4));
}

#[test]
fn test_arithmetic_mut() {
  let mut a = qi::<f32>(1, 2, 3, 4);
  a += qi(5, 6, 7, 8);
  assert_eq!(a, qi(6, 8, 10, 12));

  let mut b = qi::<f32>(1, 2, 3, 4);
  b *= qi(5, 6, 7, 8);
  assert_eq!(b, qi::<f32>(-60, 12, 30, 24));

  let mut c = qi::<f32>(1, 2, 3, 4);
  let c2 = c.to_owned();
  c.inverse_mut();
  assert_eq!(c * c2, qi::<f32>(1, 0, 0, 0));
}

#[test]
fn test_euler_angle() {
  let q: Quaternion<f32> = Quaternion::from_euler_angles(PI, PI, PI);
  // Should be a unit quaternion
  assert!((q.square_length() - 1.0).abs() < EPSILON);
}
