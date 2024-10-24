struct Z2P2 {}

const A1: u16 = 0;
const A2: u16 = 0;

const B0: u16 = 0;
const B1: u16 = 0;
const B2: u16 = 0;

/*pub fn nice() {
    let errors = [0, 0];
    let outputs = [0, 0];
    let n = 1;

    let output = B2 * errors[n - 2] + B1 * errors[n - 1] + B0 * errors[n]
        + A2 * outputs[n - 2] + A1 * outputs[n - 1];
}

pub fn fast() {
    let x = 0;
    let y = 0;

    let a = smlad(v(B2, B1), x, B0 * x[n]);
    let output = smlad(v(A2, A1), y, a);
}*/

fn v(x: i16, y: i16) -> u32 {
    (x as u32 & 0xFFFF) | ((y as u32) << 16)
}

fn hi(x: u32) -> i16 {
    (x >> 16) as i16
}

fn lo(x: u32) -> i16 {
    (x & 0xFFFF) as i16
}

/// Signed multiply accumulate dual
/// Rd = Ra + Rn[T]*Rm[T] + Rn[B]*Rm[B]
pub fn smlad_emulated(a: u32, b: u32, c: i32) -> i32 {
    let rd = c + i32::from(hi(a)) * i32::from(hi(b)) + i32::from(lo(a)) * i32::from(lo(b));
    rd
}

/// Signed multiply accumulate dual
/// Rd = Ra + Rn[T]*Rm[T] + Rn[B]*Rm[B]
pub fn smlad(a: u32, b: u32, c: i32) -> i32 {
    let rd;
    let flags: u32;

    //#[cfg(overflow_checks)]
    unsafe {
        core::arch::asm!(
            "mov {tmp}, #0",
            "msr APSR, {tmp}",
            "smlad {Rd}, {Rn}, {Rm}, {Ra}",
            "mrs {flags}, APSR",
            Rd = out(reg) rd,
            flags = out(reg) flags,
            Rn = in(reg) a,
            Rm = in(reg) b,
            Ra = in(reg) c,
            tmp = out(reg) _
        );
        assert_eq!(flags & (1 << 27), 0, "Overflow");
    }

    /*//#[cfg(not(overflow_checks))]
    unsafe {
        core::arch::asm!(
            "smlad {Rd}, {Rn}, {Rm}, {Ra}",
            Rd = out(reg) rd,
            Rn = in(reg) a,
            Rm = in(reg) b,
            Ra = in(reg) c,
        );
    }*/

    rd
}

fn test_(start: i16, max: i16, f: impl Fn(i16) -> u32, descr: &str) {
    let steps = max - start + 1;
    for a in start..=max {
        for b in start..=max {
            for c in start..=max {
                assert_eq!(
                    smlad(f(a), f(b), f(c) as i32),
                    smlad_emulated(f(a), f(b), f(c) as i32)
                );
            }
        }

        if (a - start) % (steps / 100) == 0 {
            defmt::println!("{}%", (a - start) / (steps / 100));
        }
    }

    defmt::println!("---- {} {:#x}-{:#x}, Done! Passed!🎉🚀😊 ----", descr, start, max);
}

pub fn test() {
    let f_same = |x: i16| v(x, x);
    let f_left = |x: i16| v(x, 0);
    let f_right = |x: i16| v(0, x);
    let start = 0xF + 0x0F_00;
    let max = 0xFF + 0x0F_00;

    test_(start, max, f_same, "f_same");
    test_(start, max, f_left, "f_left");
    test_(start, max, f_right, "f_right");

    {
        let res = smlad(f_same(i16::MAX), f_same(i16::MAX), 0x1FFFD);
        assert_eq!(
            res,
            smlad_emulated(f_same(i16::MAX), f_same(i16::MAX), 0x1FFFD)
        );
        assert_eq!(res, i32::MAX);
    }
    {
        let res = smlad(f_left(i16::MAX), f_left(i16::MAX), 0x4000FFFE);
        assert_eq!(
            res,
            smlad_emulated(f_left(i16::MAX), f_left(i16::MAX), 0x4000FFFE)
        );
        assert_eq!(res, i32::MAX);
    }
    {
        let res = smlad(f_right(i16::MAX), f_right(i16::MAX), 0x4000FFFE);
        assert_eq!(
            res,
            smlad_emulated(f_right(i16::MAX), f_right(i16::MAX), 0x4000FFFE)
        );
        assert_eq!(res, i32::MAX);
    }

    // -----

    {
        let res = smlad(f_same(i16::MIN), f_same(i16::MAX), -0x10000);
        assert_eq!(
            res,
            smlad_emulated(f_same(i16::MIN), f_same(i16::MAX), -0x10000)
        );
        assert_eq!(res, i32::MIN);
    }

    {
        let res = smlad(f_same(i16::MIN + 1), f_same(i16::MIN), 0xFFFF);
        assert_eq!(
            res,
            smlad_emulated(f_same(i16::MIN + 1), f_same(i16::MIN), 0xFFFF)
        );
        assert_eq!(res, i32::MAX);
    }

    defmt::println!("All Done! Passed!🎉🚀😊");
}
