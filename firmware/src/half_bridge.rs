


pub struct HalfBridge<P: embedded_hal::pwm::SetDutyCycle> {
    pwm_control: P
}

impl HalfBridge {

    /// vout = vin * d
    #[cfg(feature = "current-mode")]
    pub fn update_buck(&mut self, target_u_lo: u16, measured_u_lo: u16) {
        // https://www.biricha.com/articles/step-by-step-design-guide-for-digital-peak-current-mode-control-a-single-chip-solution
        // https://www.st.com/en/embedded-software/x-cube-dpower.html
        // https://www.ti.com/lit/an/sprabe7a/sprabe7a.pdf?ts=1723931480534
    }

    /// vout = vin * d
    #[cfg(feature = "voltage-mode")]
    pub fn update_buck(&mut self, target_u_lo: u16, measured_u_lo: u16) {

    }

    /// vout = vin/(1-d)
    #[cfg(feature = "current-mode")]
    pub fn update_boost(&mut self, target_u_hi: u16, measured_u_hi: u16) {

    }

    /// vout = vin/(1-d)
    #[cfg(feature = "voltage-mode")]
    pub fn update_boost(&mut self, target_u_hi: u16, measured_u_hi: u16) {

    }
}