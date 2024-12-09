use rppal::spi::{Bus, Mode, SlaveSelect, Spi};
use std::error::Error;
use std::thread::sleep;
use std::time::Duration;

// ADXL345 Register Addresses
const REG_BW_RATE: u8 = 0x2C;
const REG_POWER_CTL: u8 = 0x2D;
const REG_DATA_FORMAT: u8 = 0x31;
const REG_DATAX0: u8 = 0x32;
const REG_OFSZ: u8 = 0x20;

// SPI Configuration
const SPI_BUS: Bus = Bus::Spi0;
const SPI_SLAVE: SlaveSelect = SlaveSelect::Ss0;
const SPI_SPEED_HZ: u32 = 2_000_000; // Adjust to 562 kHz

// Write a value to a specific register
pub fn write_register(spi: &mut Spi, register: u8, value: u8) -> Result<(), Box<dyn Error>> {
    spi.write(&[register, value])?;
    Ok(())
}

// Read a value from a specific register

pub fn read_register(spi: &Spi, register: u8) -> Result<u8, Box<dyn Error>> {
    let mut response = [0; 2];
    spi.transfer(&mut response, &[register | 0x80, 0])?; // MSB 1 for read
    Ok(response[1])
}

// Read acceleration data from ADXL345
// pub fn read_accelerometer(spi: &Spi) -> Result<(i16, i16, i16), Box<dyn Error>> {
//     let x0 = read_register(spi, REG_DATAX0)?;
//     let x1 = read_register(spi, REG_DATAX0 + 1)?;
//     let y0 = read_register(spi, REG_DATAX0 + 2)?;
//     let y1 = read_register(spi, REG_DATAX0 + 3)?;
//     let z0 = read_register(spi, REG_DATAX0 + 4)?;
//     let z1 = read_register(spi, REG_DATAX0 + 5)?;
//     Ok((convert_to_signed(x0,x1), convert_to_signed(y0,y1), convert_to_signed(z0,z1)))
// }

// Read Z-axis acceleration data in m/s^2
pub fn read_accelerometer_z(spi: &Spi) -> Result<f32, Box<dyn Error>> {
    let z0 = read_register(spi, REG_DATAX0 + 4)?;
    let mut z1 = read_register(spi, REG_DATAX0 + 5)?;

    if (z1 & 0x80) == 0x80 { //if sign bit is high
        z1 |= 0xFC; //set all the upper bits
    } else { // and if it's low...
        z1 &= 0x03; //clear all the upper bits (or, preserve the lower 3 bits)
    };

    // let z1mod = if (z1&(0x02))!=0 {
    //     z1 & (0xFD)
    // } else {
    //     z1 | (0x03)
    // };

    let z_signed = convert_to_signed(z0, z1);
    
    // Scale the output to m/s^2
    let scaled_z = z_signed as f32 * 9.8 * 0.0039;
    Ok(scaled_z)
}

// Convert to signed 16-bit
pub fn convert_to_signed(v0: u8, v1: u8) -> i16 {
    (((v1 as u16) << 8) | (v0 as u16)) as i16
}



// // Example Usage
// pub fn main() -> Result<(), Box<dyn Error>> {
//     // Initialize SPI
//     let mut spi = Spi::new(SPI_BUS, SPI_SLAVE, SPI_SPEED_HZ, Mode::Mode3).expect("failed to init SPI");

//     // Initialize ADXL345
//     write_register(&mut spi, REG_POWER_CTL, 0x08).expect("a"); // Measure mode
//     write_register(&mut spi, REG_DATA_FORMAT, 0x00).expect("a"); // ±2g, full resolution

//     let sample_duration = Duration::from_millis(1); // 1000 Hz sample rate

//     loop {
//         let (x, y, z) = read_accelerometer(&spi).expect("a");
//         println!("X: {}, Y: {}, Z: {}", x, y, z);
//         sleep(sample_duration);
//     }
// }