use core::{fmt::write, ops::Mul};

use super::{
    atan::{self, atan2},
    hypot::hypot,
    sqrt,
};

#[derive(Clone, Copy)]
pub struct Complex {
    pub re: f64,
    pub im: f64,
}

impl core::fmt::Debug for Complex {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        core::fmt::Display::fmt(&self, f)
    }
}

impl core::fmt::Display for Complex {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match (self.re, self.im) {
            (0.0, 1.0) => write!(f, "i"),
            (0.0, -1.0) => write!(f, "-i"),
            (0.0, im) if im.is_infinite() => write!(f, "inf*i"),
            (0.0, im) => write!(f, "{im}i"),
            (re, 0.0) => write!(f, "{re}",),
            (re, 1.0) => write!(f, "{re} + i"),
            (re, -1.0) => write!(f, "{re} - i"),
            (re, im) if im < 0.0 => write!(f, "({re} - {}i)", -im),
            (re, im) => write!(f, "({re} + {im}i)"),
        }
    }
}

impl Complex {
    const i: Self = Self::new(0.0, 1.0);
    const j: Self = Self::i;
    const ONE: Self = Self::new(1.0, 0.0);

    pub const fn sqrt_r(re: f64) -> Self {
        if re >= 0.0 {
            Self::new(super::sqrt(re), 0.0)
        } else if re < 0.0 {
            Self::new(0.0, super::sqrt(-re))
        } else {
            panic!("undefined")
        }
    }

    pub const fn new(re: f64, im: f64) -> Self {
        Self { re, im }
    }

    /// re - z
    pub const fn r_sub(re: f64, rhs: Self) -> Self {
        Self::new(re - rhs.re, 0.0 - rhs.im)
    }

    /// re / z
    pub const fn r_div(re: f64, other: Complex) -> Self {
        // a / (c + i d) == [a * (c - i d)] / (c*c + d*d)
        let norm_sqr = other.norm_sqr();
        Self::new(re * other.re / norm_sqr, 0.0 - re * other.im / norm_sqr)
    }

    /// z / re
    pub const fn div_r(self, re: f64) -> Self {
        Self::new(self.re / re, self.im / re)
    }

    pub const fn add(self, rhs: Self) -> Self {
        Self {
            re: self.re + rhs.re,
            im: self.im + rhs.im,
        }
    }

    pub const fn sub(self, rhs: Self) -> Self {
        Self {
            re: self.re - rhs.re,
            im: self.im - rhs.im,
        }
    }

    pub const fn add_r(self, rhs: f64) -> Self {
        Self {
            re: self.re + rhs,
            im: self.im,
        }
    }

    const fn mul(self, rhs: Self) -> Self {
        // (x + iy) * (z + iw) =
        // = xz + xiw + iyz + iyiw = // i*i=-1
        // = xz + ixw + iyz - yw =
        // = (xz - yw) + i(xw + yz)
        Self {
            re: (self.re * rhs.re - self.im * rhs.im),
            im: (self.re * rhs.im + self.im * rhs.re),
        }
    }

    /// Returns the square of the norm, i.e. `re^2 + im^2`.
    #[inline]
    pub const fn norm_sqr(&self) -> f64 {
        // https://docs.rs/num-complex/0.4.6/src/num_complex/lib.rs.html#149
        self.re * self.re + self.im * self.im
    }

    const fn div(self, rhs: Self) -> Self {
        // https://docs.rs/num-complex/0.4.6/src/num_complex/lib.rs.html#820
        let norm_sqr = rhs.norm_sqr();
        let re = self.re * rhs.re + self.im * rhs.im;
        let im = self.im * rhs.re - self.re * rhs.im;
        Self {
            re: re / norm_sqr,
            im: im / norm_sqr,
        }
    }

    const fn mul_r(self, rhs: f64) -> Self {
        Self {
            re: self.re * rhs,
            im: self.im * rhs,
        }
    }

    /// Calculate |self|
    ///
    /// From https://docs.rs/num-complex/0.4.6/src/num_complex/lib.rs.html#217
    #[inline]
    pub const fn norm(self) -> f64 {
        super::hypot::hypot(self.re, self.im)
    }
    /// Calculate the principal Arg of self.
    ///
    /// https://docs.rs/num-complex/0.4.6/src/num_complex/lib.rs.html#222
    #[inline]
    pub const fn arg(self) -> f64 {
        super::atan::atan2(self.im, self.re)
    }

    /// Convert to polar form (r, theta), such that
    /// `self = r * exp(i * theta)`
    ///
    /// From https://docs.rs/num-complex/0.4.6/src/num_complex/lib.rs.html#233
    #[inline]
    pub const fn to_polar(self) -> (f64, f64) {
        (self.norm(), self.arg())
    }

    /// Computes the principal value of natural logarithm of `self`.
    ///
    /// From https://docs.rs/num-complex/0.4.6/src/num_complex/lib.rs.html#471
    ///
    /// This function has one branch cut:
    ///
    /// * `(-∞, 0]`, continuous from above.
    ///
    /// The branch satisfies `-π ≤ arg(ln(z)) ≤ π`.
    #[inline]
    pub const fn ln(self) -> Self {
        // formula: ln(z) = ln|z| + i*arg(z)
        let (r, theta) = self.to_polar();
        Self::new(super::log::log(r), theta)
    }

    pub const fn tan(self) -> Self {
        use super::{cos::cos, cosh::cosh, pow2, sin::sin, sinh::sinh};
        let x = self.re;
        let y = self.im;

        let re = sin(x) * cos(x);
        let im = -sinh(y) * cosh(y);

        let d = pow2(sin(x)) + pow2(sinh(y));

        Self { re, im }.div_r(d)
    }

    /// Computes the principal value of the inverse tangent of `self`.
    ///
    /// From https://docs.rs/num-complex/0.4.6/src/num_complex/lib.rs.html#471
    ///
    /// This function has two branch cuts:
    ///
    /// * `(-∞i, -i]`, continuous from the left.
    /// * `[i, ∞i)`, continuous from the right.
    ///
    /// The branch satisfies `-π/2 ≤ Re(atan(z)) ≤ π/2`.
    #[inline]
    pub const fn atan(self) -> Self {
        // formula: arctan(z) = (ln(1+iz) - ln(1-iz))/(2i)
        let i = Self::i;
        let one = Self::ONE;

        if self.re == 0.0 && self.im == 1.0 {
            return Self::new(0.0, f64::INFINITY);
        } else if self.re == 0.0 && self.im == -1.0 {
            return Self::new(0.0, -f64::INFINITY);
        }
        (one.add(i.mul(self)))
            .ln()
            .sub((one.sub(i.mul(self))).ln())
            .div(Self::new(0.0, 2.0))
    }
}

impl core::ops::Add for Complex {
    type Output = Complex;

    fn add(self, rhs: Complex) -> Self::Output {
        Complex {
            re: self.re + rhs.re,
            im: self.im + rhs.im,
        }
    }
}

impl core::ops::Add<f64> for Complex {
    type Output = Complex;

    fn add(self, rhs: f64) -> Self::Output {
        Complex {
            re: self.re + rhs,
            im: self.im,
        }
    }
}

impl core::ops::Add<Complex> for f64 {
    type Output = Complex;

    fn add(self, rhs: Complex) -> Self::Output {
        Complex {
            re: self + rhs.re,
            im: rhs.im,
        }
    }
}

impl core::ops::Sub<Complex> for f64 {
    type Output = Complex;

    fn sub(self, rhs: Complex) -> Self::Output {
        Complex::r_sub(self, rhs)
    }
}

impl core::ops::Mul for Complex {
    type Output = Complex;

    fn mul(self, rhs: Self) -> Self::Output {
        self.mul(rhs)
    }
}

impl core::ops::Mul<f64> for Complex {
    type Output = Complex;

    fn mul(self, rhs: f64) -> Self::Output {
        self.mul_r(rhs)
    }
}

impl core::ops::Mul<Complex> for f64 {
    type Output = Complex;

    fn mul(self, rhs: Complex) -> Self::Output {
        Complex {
            re: self * rhs.re,
            im: self * rhs.im,
        }
    }
}

impl core::ops::Div for Complex {
    type Output = Complex;

    fn div(self, rhs: Self) -> Self::Output {
        self.div(rhs)
    }
}

impl core::ops::Div<f64> for Complex {
    type Output = Complex;

    fn div(self, rhs: f64) -> Self::Output {
        self.div_r(rhs)
    }
}

impl core::ops::Div<Complex> for f64 {
    type Output = Complex;

    fn div(self, rhs: Complex) -> Self::Output {
        Complex::r_div(self, rhs)
    }
}
