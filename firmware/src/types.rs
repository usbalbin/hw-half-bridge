use core::ops;

macro_rules! impl_ops {
    ($t:ident) => {
        impl ops::Neg for $t {
            type Output = Self;
        
            fn neg(self) -> Self::Output {
                Self(-self.0)
            }
        }

        impl ops::Add<Self> for $t {
            type Output = Self;
        
            fn add(self, rhs: Self) -> Self::Output {
                Self(self.0 + rhs.0)
            }
        }
        
        impl ops::Sub<Self> for $t {
            type Output = Self;
        
            fn sub(self, rhs: Self) -> Self::Output {
                Self(self.0 - rhs.0)
            }
        }
        
        impl ops::Mul<$t> for f32 {
            type Output = $t;
        
            fn mul(self, rhs: $t) -> Self::Output {
                $t(self * rhs.0)
            }
        }
        
        impl ops::Mul<f32> for $t {
            type Output = $t;
        
            fn mul(self, rhs: f32) -> Self::Output {
                Self(self.0 * rhs)
            }
        }

        impl ops::Div<Self> for $t {
            type Output = f32;
        
            fn div(self, rhs: Self) -> Self::Output {
                self.0 / rhs.0
            }
        }
    };
}

pub struct Voltage(pub f32);
pub struct Amperes(pub f32);
pub struct Ohms(pub f32);
pub struct Henries(pub f32);
pub struct InvHenries(pub f32);
pub struct Farads(pub f32);
pub struct Seconds(pub f32);

impl_ops!(Voltage);
impl_ops!(Amperes);
impl_ops!(Ohms);
impl_ops!(Henries);
impl_ops!(Farads);
impl_ops!(Seconds);

impl Voltage {
    pub fn to_adc_value(self)-> u16 {
        (self / Voltage(3.3)).clamp(0, 1.0) * f32::from(0xFFF) as u16
    }
    
    pub fn to_dac_value(self)-> u16 {
        (self / Voltage(3.3)).clamp(0, 1.0) * f32::from(0xFFF) as u16
    }
}

impl ops::Div<Henries> for f32 {
    type Output = InvHenries;

    fn div(self, rhs: Henries) -> Self::Output {
        Self::Output(self / rhs.0)
    }
}

impl ops::Mul<InvHenries> for Henries {
    type Output = f32;

    fn mul(self, rhs: InvHenries) -> Self::Output {
        self.0 * rhs.0
    }
}

// u = i*r
impl ops::Mul<Ohms> for Amperes {
    type Output = Voltage;

    fn mul(self, rhs: Ohms) -> Self::Output {
        Voltage(self.0 * rhs.0)
    }
}

// u = r*i
impl ops::Mul<Amperes> for Ohms {
    type Output = Voltage;

    fn mul(self, rhs: Amperes) -> Self::Output {
        Voltage(self.0 * rhs.0)
    }
}

// r = u/i
impl ops::Div<Amperes> for Voltage {
    type Output = Ohms;

    fn div(self, rhs: Amperes) -> Self::Output {
        Ohms(self.0 * rhs.0)
    }
}

// i = u/r
impl ops::Div<Ohms> for Voltage {
    type Output = Amperes;

    fn div(self, rhs: Ohms) -> Self::Output {
        Amperes(self.0 * rhs.0)
    }
}

// L = r * t
impl ops::Mul<Seconds> for Ohms {
    type Output = Henries;

    fn mul(self, rhs: Seconds) -> Self::Output {
        Henries(self.0 * rhs.0)
    }
}