//! a simpler quaternion math library with traits.
//!
//! ```rust
//! extern crate quaternions;
//! use quaternions::{Quaternion, q, qi};
//!
//! let a = Quaternion { w: 1.0, x: 2.0, y: 3.0, z: 4.0 };
//! a.w;
//!
//! // quick creates
//! let b1 = q::<f32>(1.0, 2.0, 3.0, 4.0);
//!
//! // quick creates with integers
//! let b2 = qi::<f32>(1, 2, 3, 4);
//!
//! b1 + b2;
//! b1 - b2;
//! b1 * b2;
//! b1 / b2;
//! b1.conjugate();
//! b1.scale(1.5);
//! b1.square_length();
//! b1.length();
//! b1.inverse();
//!```
//!
//! There are also mutable APIs:
//!
//! ```rust
//! extern crate quaternions;
//! use quaternions::{Quaternion, q, qi};
//!
//! let mut c = Quaternion::id();
//! let b = qi::<f32>(1, 2, 3, 4);
//!
//! c += b;
//! c -= b;
//! c *= b;
//! // no division
//! c.inverse_mut();
//! c.conjugate_mut();
//! c.scale_mut(1.5);
//! ```
//!

use std::ops::{Add, AddAssign, Div, Mul, MulAssign, Neg, Sub, SubAssign};

use num_traits::Float;

/// Quaternion {w, x, y, z}
#[derive(Debug, Copy, Clone)]
pub struct Quaternion<T: Float> {
  pub w: T,
  pub x: T,
  pub y: T,
  pub z: T,
}

/// shortcut of create quaternion
pub fn q<T>(w: T, x: T, y: T, z: T) -> Quaternion<T>
where
  T: Float,
{
  Quaternion { w, x, y, z }
}
/// shortcut of creating quaternion from integers
pub fn qi<T>(w: i32, x: i32, y: i32, z: i32) -> Quaternion<T>
where
  T: Float,
{
  q(T::from(w).unwrap(), T::from(x).unwrap(), T::from(y).unwrap(), T::from(z).unwrap())
}

impl<T> Quaternion<T>
where
  T: Float,
{
  /// returns an identity quaternion
  pub fn id() -> Self {
    Quaternion {
      w: T::one(),
      x: T::zero(),
      y: T::zero(),
      z: T::zero(),
    }
  }

  /// returns a quaternion from xyz
  pub fn new(w: T, x: T, y: T, z: T) -> Self {
    Quaternion { w, x, y, z }
  }

  /// Scales a quaternion (element-wise) by a scalar, returns a new one
  pub fn scale(&self, t: T) -> Quaternion<T> {
    Quaternion {
      w: self.w * t,
      x: self.x * t,
      y: self.y * t,
      z: self.z * t,
    }
  }

  pub fn scale_mut(&mut self, t: T) {
    self.w = self.w * t;
    self.x = self.x * t;
    self.y = self.y * t;
    self.z = self.z * t;
  }

  /// return a inverse of a quaternion
  pub fn inverse(&self) -> Self {
    self.scale(T::from(1.).unwrap() / self.square_length()).conjugate()
  }

  /// mutable version of inverse
  pub fn inverse_mut(&mut self) {
    self.scale_mut(T::from(1.).unwrap() / self.square_length());
    self.conjugate_mut();
  }

  /// returns dot product of two quaternions
  pub fn dot(&self, other: &Quaternion<T>) -> T {
    self.w * other.w + self.x * other.x + self.y * other.y + self.z * other.z
  }

  /// Takes the quaternion conjugate, returns a new one.
  pub fn conjugate(&self) -> Self {
    Quaternion {
      w: self.w,
      x: -self.x,
      y: -self.y,
      z: -self.z,
    }
  }

  /// mutable version of conjugate
  pub fn conjugate_mut(&mut self) {
    self.x = -self.x;
    self.y = -self.y;
    self.z = -self.z;
  }

  /// Computes the square length of a quaternion.
  #[inline(always)]
  pub fn square_length(&self) -> T {
    self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z
  }

  /// Computes the length of a quaternion.
  #[inline(always)]
  pub fn length(&self) -> T {
    self.square_length().sqrt()
  }
}

impl<T> Default for Quaternion<T>
where
  T: Float,
{
  /// returns a zero quaternion
  fn default() -> Self {
    Quaternion {
      w: T::zero(),
      x: T::zero(),
      y: T::zero(),
      z: T::zero(),
    }
  }
}

impl<T> PartialEq for Quaternion<T>
where
  T: Float,
{
  /// Returns true if the two quaternions very cloase, compared like:
  /// ```ignore
  /// |a-b|^2 < epsilon
  /// ```
  fn eq(&self, other: &Self) -> bool {
    (*self - *other).square_length() < T::epsilon()
  }
}

impl<T> Eq for Quaternion<T> where T: Float {}

impl<T> Add for Quaternion<T>
where
  T: Float,
{
  type Output = Self;
  /// Adds two quaternions, returns a new one.
  fn add(self, other: Self) -> Self {
    Quaternion {
      w: self.w + other.w,
      x: self.x + other.x,
      y: self.y + other.y,
      z: self.z + other.z,
    }
  }
}

impl<T> AddAssign for Quaternion<T>
where
  T: Float,
{
  /// Adds two quaternions, returns a new one.
  fn add_assign(&mut self, other: Self) {
    self.w = self.w + other.w;
    self.x = self.x + other.x;
    self.y = self.y + other.y;
    self.z = self.z + other.z;
  }
}

impl<T> Mul for Quaternion<T>
where
  T: Float,
{
  type Output = Self;
  /// Multiplies two quaternions, returns a new one.
  fn mul(self, other: Self) -> Self {
    Quaternion {
      w: self.w * other.w - self.x * other.x - self.y * other.y - self.z * other.z,
      x: self.w * other.x + self.x * other.w + self.y * other.z - self.z * other.y,
      y: self.w * other.y - self.x * other.z + self.y * other.w + self.z * other.x,
      z: self.w * other.z + self.x * other.y - self.y * other.x + self.z * other.w,
    }
  }
}

impl<T> MulAssign for Quaternion<T>
where
  T: Float,
{
  /// Multiplies two quaternions, returns a new one.
  fn mul_assign(&mut self, other: Self) {
    let w = self.w * other.w - self.x * other.x - self.y * other.y - self.z * other.z;
    let x = self.w * other.x + self.x * other.w + self.y * other.z - self.z * other.y;
    let y = self.w * other.y - self.x * other.z + self.y * other.w + self.z * other.x;
    let z = self.w * other.z + self.x * other.y - self.y * other.x + self.z * other.w;
    self.w = w;
    self.x = x;
    self.y = y;
    self.z = z;
  }
}

impl<T> Div for Quaternion<T>
where
  T: Float,
{
  type Output = Self;
  fn div(self, b: Self) -> Self::Output {
    self.mul(b.inverse())
  }
}

impl<T> Sub for Quaternion<T>
where
  T: Float,
{
  type Output = Self;

  /// substraction of two quaternions, returning new value
  fn sub(self, other: Self) -> Self {
    Quaternion {
      w: self.w - other.w,
      x: self.x - other.x,
      y: self.y - other.y,
      z: self.z - other.z,
    }
  }
}

impl<T> SubAssign for Quaternion<T>
where
  T: Float,
{
  /// substraction of two quaternions, returning new value
  fn sub_assign(&mut self, other: Self) {
    self.w = self.w - other.w;
    self.x = self.x - other.x;
    self.y = self.y - other.y;
    self.z = self.z - other.z;
  }
}

impl<T> Neg for Quaternion<T>
where
  T: Float,
{
  type Output = Self;
  /// negate number
  fn neg(self) -> Self::Output {
    Quaternion {
      w: -self.w,
      x: -self.x,
      y: -self.y,
      z: -self.z,
    }
  }
}

impl<T> Quaternion<T>
where
  T: Float,
{
  /// Construct a quaternion representing the given euler angle rotations (in radians)
  #[inline(always)]
  pub fn from_euler_angles(x: T, y: T, z: T) -> Quaternion<T> {
    let two: T = T::one() + T::one();

    let half_x = x / two;
    let half_y = y / two;
    let half_z = z / two;

    let cos_x_2 = half_x.cos();
    let cos_y_2 = half_y.cos();
    let cos_z_2 = half_z.cos();

    let sin_x_2 = half_x.sin();
    let sin_y_2 = half_y.sin();
    let sin_z_2 = half_z.sin();

    Quaternion {
      w: cos_x_2 * cos_y_2 * cos_z_2 + sin_x_2 * sin_y_2 * sin_z_2,
      x: sin_x_2 * cos_y_2 * cos_z_2 + cos_x_2 * sin_y_2 * sin_z_2,
      y: cos_x_2 * sin_y_2 * cos_z_2 + sin_x_2 * cos_y_2 * sin_z_2,
      z: cos_x_2 * cos_y_2 * sin_z_2 + sin_x_2 * sin_y_2 * cos_z_2,
    }
  }
}
