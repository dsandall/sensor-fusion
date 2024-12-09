use rppal::i2c::I2c;
use std::error::Error;

pub struct Mpu {
    i2c: I2c,
    address: u16,
    accel_range_modifier: f32,
    gyro_range_modifier: f32,
}

impl Mpu {
    // mpu6050 Register addresses
    const PWR_MGMT_1: u8 = 0x6B;
    const MPU_CONFIG: u8 = 0x1A;
    const ACCEL_CONFIG: u8 = 0x1C;
    const GYRO_CONFIG: u8 = 0x1B;
    const ACCEL_XOUT0: u8 = 0x3B;
    const TEMP_OUT0: u8 = 0x41;
    const GYRO_XOUT0: u8 = 0x43;

    // Scale Modifiers
    const ACCEL_SCALE_MODIFIER_2G: f32 = 16384.0;
    const GYRO_SCALE_MODIFIER_250DEG: f32 = 131.0;
    const GRAVITY_MS2: f32 = 9.80665;

    pub fn new(address: u16) -> Result<Self, Box<dyn Error>> {
        let mut i2c = I2c::new()?;
        i2c.set_slave_address(address)?;

        // Wake up the MPU-6050 from sleep mode
        i2c.smbus_write_byte(Self::PWR_MGMT_1, 0x00)?;

        Ok(Self {
            i2c,
            address,
            accel_range_modifier: Self::ACCEL_SCALE_MODIFIER_2G,
            gyro_range_modifier: Self::GYRO_SCALE_MODIFIER_250DEG,
        })
    }

    pub fn verify_stuff(&mut self) {
        
        dbg!(self.i2c.capabilities());
        let f = self.i2c.smbus_read_byte(0x75);
        println!("{:#06x}", f.unwrap());

    }

    fn read_i2c_word(&mut self, register: u8) -> Result<i16, Box<dyn Error>> {
        
        // let mut buffer = [0; 2];
        // self.i2c.smbus_read_word(register, &mut buffer)?;
        // let high = buffer[0] as i16;
        // let low = buffer[1] as i16;
        // let value = (high << 8) | low;
        // Ok(if value >= 0x8000 { -(65535 - value + 1) } else { value })

        // Ok(self.i2c.smbus_read_word(register)? as i16)

        let upper = self.i2c.smbus_read_word_swapped(register)?;

        // let lower = self.i2c.smbus_read_byte(register+1)?;
        // Ok(((upper as u16) << 8  | lower as u16) as i16)

        Ok(upper as i16)

    }

    pub fn set_accel_range(&mut self, range: u8) -> Result<(), Box<dyn Error>> {
        self.i2c.smbus_write_byte(Self::ACCEL_CONFIG, range)?;
        self.accel_range_modifier = match range {
            0x00 => Self::ACCEL_SCALE_MODIFIER_2G,
            _ => Self::ACCEL_SCALE_MODIFIER_2G, // Expand to other ranges if needed
        };
        Ok(())
    }

    pub fn set_filter_range(&mut self, filter_range: u8) -> Result<(), Box<dyn Error>> {
        // Read the current EXT_SYNC_SET configuration from the MPU_CONFIG register
        let ext_sync_set = self.i2c.smbus_read_byte(Self::MPU_CONFIG)? & 0b00111000;
    
        // Write the new filter range, keeping the existing EXT_SYNC_SET configuration
        self.i2c.smbus_write_byte(Self::MPU_CONFIG, ext_sync_set | filter_range)?;
    
        Ok(())
    }
    

    pub fn get_accel_z(&mut self) -> Result<f32, Box<dyn Error>> {
        let z = self.read_i2c_word(Self::ACCEL_XOUT0 + 4)?;
        Ok((z as f32/ self.accel_range_modifier) * Self::GRAVITY_MS2)
    }

    pub fn get_temp(&mut self) -> Result<f32, Box<dyn Error>> {
        let raw_temp = self.read_i2c_word(Self::TEMP_OUT0)?;
        Ok((raw_temp as f32 / 340.0) + 36.53)
    }

    pub fn get_all_data(&mut self) -> Result<(f32, f32), Box<dyn Error>> {
        let temp = self.get_temp()?;
        let accel_z = self.get_accel_z()?;
        Ok((temp, accel_z))
    }
}

// fn main() -> Result<(), Box<dyn Error>> {
//     let mut mpu = Mpu::new(0x68)?;  // Default I2C address for MPU-6050
//     mpu.set_accel_range(0x00)?;  // Set to ±2g

//     println!("Temperature: {:.2} °C", mpu.get_temp()?);
//     println!("Accel Z: {:.2} m/s²", mpu.get_accel_z()?);

//     Ok(())
// }
