#![deny(unsafe_code)]

extern crate cast;
extern crate fpa;

//use fpa::I25F7;
use fpa::I9F7;

pub fn convert_temp_from_register(msb: u8, lsb: u8) -> f32 {
    let value: i16 = (i16::from(msb) << 8) + i16::from(lsb);
    cast::f32(I9F7::from_bits(value))
}

pub fn convert_temp_to_register(temp: f32) -> (u8, u8) {
    let q = I9F7(temp).unwrap();
    let bits = q.into_bits() as i16;
    let msb :u8 = ((bits >> 8) & 0xff) as u8;
    let lsb :u8 = (bits & 0xff) as u8;
    (msb, lsb)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn can_convert_temperature_from_register() {
        assert_eq!(150.0, convert_temp_from_register(0b0100_1011, 0b0000_0000));
        assert_eq!(25.0, convert_temp_from_register(0b0000_1100, 0b1000_0000));
        assert_eq!(1.0, convert_temp_from_register(0b0000_0000, 0b1000_0000));
        assert_eq!(0.5, convert_temp_from_register(0b0000_0000, 0b0100_0000));
        assert_eq!(0.25, convert_temp_from_register(0b0000_0000, 0b0010_0000));
        assert_eq!(0.125, convert_temp_from_register(0b0000_0000, 0b0001_0000));
        assert_eq!(0.0625, convert_temp_from_register(0b0000_0000, 0b0000_1000));
        assert_eq!(
            0.03125,
            convert_temp_from_register(0b0000_0000, 0b0000_0100)
        );
        assert_eq!(0.0, convert_temp_from_register(0b0000_0000, 0b0000_0000));
        assert_eq!(
            -0.03125,
            convert_temp_from_register(0b1111_1111, 0b1111_1100)
        );
        assert_eq!(
            -0.0625,
            convert_temp_from_register(0b1111_1111, 0b1111_1000)
        );
        assert_eq!(-0.125, convert_temp_from_register(0b1111_1111, 0b1111_0000));
        assert_eq!(-0.25, convert_temp_from_register(0b1111_1111, 0b1110_0000));
        assert_eq!(-0.5, convert_temp_from_register(0b1111_1111, 0b1100_0000));
        assert_eq!(-1.0, convert_temp_from_register(0b1111_1111, 0b1000_0000));
        assert_eq!(-25.0, convert_temp_from_register(0b1111_0011, 0b1000_0000));
        assert_eq!(-40.0, convert_temp_from_register(0b1110_1100, 0b0000_0000));
    }

    #[test]
    fn can_convert_temperature_to_register() {
        assert_eq!(convert_temp_to_register(-0.125), (0b1111_1111, 0b1111_0000));
        assert_eq!(convert_temp_to_register(-40.0), (0b1110_1100, 0b0000_0000));
        assert_eq!(convert_temp_to_register(25.0), (0b0000_1100, 0b1000_0000));
    }
}
