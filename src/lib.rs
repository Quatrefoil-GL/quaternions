//! A simple and type agnostic quaternion math library designed for reexporting

use std::ops::{Add, Div, Mul, Sub};

use num_traits::Float;

/// Quaternion (w,x,y,z).
pub struct Quaternion<T: Float> {
  w: T,
  x: T,
  y: T,
  z: T,
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

  /// return a inverse of a quaternion
  pub fn inverse(&self) -> Self {
    self.scale(T::from(1.).unwrap() / self.square_length()).conjugate()
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

  /// negate value of a quaternion
  pub fn negate(&self) -> Self {
    Quaternion {
      w: -self.w,
      x: -self.x,
      y: -self.y,
      z: -self.z,
    }
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
  /// Returns true if the two quaternions are equal.
  fn eq(&self, other: &Self) -> bool {
    (self.w - other.x < T::epsilon())
      && (self.x - other.x < T::epsilon())
      && (self.y - other.y < T::epsilon())
      && (self.z - other.z < T::epsilon())
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

impl<T> Quaternion<T> where T: Float {}

impl<T> Div for Quaternion<T>
where
  T: Float,
{
  type Output = Self;
  fn div(self, b: Self) -> Self::Output {
    self.mul(b.inverse())
  }
}

impl Sub for Quaternion<f32> {
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
