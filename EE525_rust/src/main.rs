use rppal::spi::{Bus, Mode, SlaveSelect, Spi};
use std::{fs::File, sync::{Arc, Mutex, mpsc}};
use std::process::Command;
use std::thread;
use std::time::{Duration, Instant};
use chrono::Local;
use csv::Writer;
use std::{env, io};


pub mod sensors;
use crate::sensors::{mpu6050, main_adxl345};
use mpu6050::Mpu;

// Test Setup Parameters
const END_TIME: f64 = 5.0;
const INITIAL_WAIT: f64 = 0.5;
const DISPLAY_TIMES: bool = false;
const CONTROL_PRINTER: bool = false;
const SAMPLE_PERIOD: f64 = 1.0/3000.0;

//ADXL
const SPI_SPEED_HZ: u32 = 4_000_000; // Adjust to 562 kHz


fn main() -> Result<(), io::Error> {

    // // Collect command-line arguments
    // let args: Vec<String> = env::args().collect();

    // // Check if we have at least one argument (the program name is always the first)
    // if args.len() < 2 {
    //     println!("Usage: {} <argument>", args[0]);
    //     return Err(io::Error::new(io::ErrorKind::InvalidInput, "Not enough arguments"));
    // }

    // // Access the argument (args[1] is the first user-provided argument)
    // let input = &args[1];
    // println!("Argument provided: {}", input);

    // if args.len() > 2{ //theres another argument hiding in there
    //     // do nothing
    // }

    //////////////////////////////////////

    // Initialize SPI for ADXL345

    // Initialize sensors
    let (spi, mpu) = initialize_sensors()?;

    // Home the printer
    println!("Homing printer");
    // if CONTROL_PRINTER {
    //     Command::new("sh").arg("-c")
    //         .arg("echo G28 > ~/printer_data/comms/klippy.serial")
    //         .spawn()
    //         .expect("Failed to send G-code command").wait()?;
    //     thread::sleep(Duration::from_secs(25));
    // }

    // Generate a unique filename with the current date and time
    let start_time = Local::now().format("%Y-%m-%d_%H-%M-%S").to_string();
    let filename = format!("sensors_data_{}_{}_{}s_{}_sample_freq.csv", start_time, if CONTROL_PRINTER  {"moving"} else {"static"}, END_TIME, 1.0/SAMPLE_PERIOD);

    // Start data collection
    println!("{}", filename.to_string());
    println!("Starting data collection...");
    let data_records = collect_data(&spi, mpu, END_TIME + INITIAL_WAIT, INITIAL_WAIT, SAMPLE_PERIOD);

    // Data capture is complete
    println!("Data capture complete.");

    // Write data to CSV after collection
    write_data_to_csv(&filename, &data_records)?;

    Ok(())
}




fn initialize_sensors() -> Result<(Spi, Mpu), io::Error> {
    // Initialize ADXL345
    let mut spi = Spi::new(Bus::Spi0, SlaveSelect::Ss0, SPI_SPEED_HZ, Mode::Mode3).unwrap();

    // Define constants for the ADXL345 registers
    const REG_BW_RATE: u8 = 0x2C;
    const REG_POWER_CTL: u8 = 0x2D;
    const REG_DATA_FORMAT: u8 = 0x31;

    spi.write(&[REG_POWER_CTL, 0x08]);  // Measure mode
    spi.write(&[REG_BW_RATE, 0x0F]);    // Max internal sample rate, 3200hz (BW of 1600hz)
    spi.write(&[REG_DATA_FORMAT, 0x00]); // +/- 2g, 10-bit mode, 4 wire mode, left justified now
    println!("ADXL345 Initialized");

    main_adxl345::read_register(&spi, 0x2C).unwrap();

    // Initialize MPU6050
    let mut mpu = Mpu::new(0x68).unwrap();

    mpu.set_accel_range(0x00); // +-2G
    mpu.set_filter_range(0x00); // 256 BW filter
    mpu.verify_stuff();
    println!("MPU6050 Initialized");


    Ok((spi, mpu))
}



fn collect_data(
    spi: &Spi,
    mut mpu: Mpu,
    duration: f64,
    initial_wait: f64,
    sample_interval: f64,
) -> Vec<Vec<f64>> {
    let mut sent_cmd = false;
    let mut data_records = Vec::new();
    let start_timestamp = Instant::now();
    let end_capture_time = start_timestamp + Duration::from_secs_f64(duration);

    let (tx, rx) = mpsc::channel::<f32>();

    // Spawn a thread to get acceleration data from MPU6050
    let mpu_thread = thread::spawn(move || {
        loop {
            let dat = mpu.get_accel_z().unwrap_or(-70.);
            tx.send(dat).unwrap();  // Send data back to main thread
        }
    });

    // Calculate the interval between samples in seconds
    let mut next_sample_time = start_timestamp + Duration::from_secs_f64(sample_interval);

    while Instant::now() < end_capture_time {
        let current_time = Instant::now();
        let elapsed_time = current_time.duration_since(start_timestamp).as_secs_f64();

        // Wait until the next sample time
        if current_time < next_sample_time {
            let sleep_duration = next_sample_time - current_time;
            thread::sleep(sleep_duration); // Sleep until the next sample time
        }

        // Now it's time for the next sample, update the next sample time
        next_sample_time += Duration::from_secs_f64(sample_interval);

        // Query the SPI data
        let adxl_data: f32 = main_adxl345::read_accelerometer_z(spi).unwrap();
        let rec: u128 = Instant::now().duration_since(current_time).as_micros();
        if DISPLAY_TIMES { println!("adxl finished in {}", rec); }

        // Check if time to start movement
        if CONTROL_PRINTER && !sent_cmd && elapsed_time > initial_wait {
            println!("Sending G-code command...");
            Command::new("sh").arg("-c")
                .arg("echo G91 > ~/printer_data/comms/klippy.serial")
                .spawn()
                .expect("Failed to send G-code command").wait().unwrap();
            Command::new("sh").arg("-c")
                .arg("echo G1 X-200 F6000 > ~/printer_data/comms/klippy.serial")
                .spawn()
                .expect("Failed to send G-code command").wait().unwrap();
            sent_cmd = true;
        }

        let mpu_data = rx.recv().unwrap(); // block until MPU thread has data ready

        let rec: u128 = Instant::now().duration_since(current_time).as_micros();
        if DISPLAY_TIMES { println!("mpu finished in {}", rec); }

        // Store the collected data
        data_records.push(vec![elapsed_time, 0.0, 0.0, mpu_data.into(), 0.0, 0.0, adxl_data.into()]);
    }

    drop(mpu_thread); // Complete the MPU thread

    data_records // Return the data for CSV formatting
}



/*
fn collect_data(
    spi: &Spi,
    mut mpu: Mpu,
    duration: f64,
    initial_wait: f64,
) -> Vec<Vec<f64>> {
    let mut sent_cmd = false;
    let mut data_records = Vec::new();
    let start_timestamp = Instant::now();
    let end_capture_time = start_timestamp + Duration::from_secs_f64(duration);

    let (tx, rx) = mpsc::channel::<f32>();
    // unsafe{TX = Some(tx);}

    // let mpu_data = Arc::clone(mpu);
    // let sender = unsafe{TX.as_ref()};
    
    // Spawn a thread to get acceleration data from MPU6050
    let mpu_thread = thread::spawn(move || {
        // let mut mpu_locked = mpu.lock().unwrap();
        loop {
            let dat = mpu.get_accel_z().unwrap_or(-70.);
            tx.send(dat).unwrap();  // Send data back to main thread
        }
    });

    while Instant::now() < end_capture_time {
        let current_time = Instant::now();
        let elapsed_time = current_time.duration_since(start_timestamp).as_secs_f64();

        let adxl_data: f32 = main_adxl345::read_accelerometer_z(spi).unwrap(); // query the SPI 
        let rec: u128 = Instant::now().duration_since(current_time).as_micros(); // mark spi completion timer
        if DISPLAY_TIMES {println!("adxl finished in {}", rec);}

        // Check if time to start movement
        if CONTROL_PRINTER && !sent_cmd && elapsed_time > initial_wait {
            println!("Sending G-code command...");
            Command::new("sh").arg("-c")
                .arg("echo G91 > ~/printer_data/comms/klippy.serial")
                .spawn()
                .expect("Failed to send G-code command").wait().unwrap();
            Command::new("sh").arg("-c")
                .arg("echo G1 X-100 F6000 > ~/printer_data/comms/klippy.serial")
                .spawn()
                .expect("Failed to send G-code command").wait().unwrap();
            sent_cmd = true;
        }
        
        let mpu_data = rx.recv().unwrap(); // block until MPU thread has data ready
        
        let rec: u128 = Instant::now().duration_since(current_time).as_micros(); //mark mpu completion timer
        if DISPLAY_TIMES {println!("mpu finished in {}", rec);}
        
        data_records.push(vec![elapsed_time, 0.0, 0.0, mpu_data.into(), 0.0, 0.0, adxl_data.into()]);
    }
    
    drop(mpu_thread); //complete the MPU thread

    data_records //return the data for csv formatting
}
*/

fn write_data_to_csv(filename: &str, data_records: &[Vec<f64>]) -> Result<(), io::Error> {
    let file = File::create(filename)?;
    let mut wtr = Writer::from_writer(file);
    wtr.write_record(["Time", "AccelX1", "AccelY1", "AccelZ1", "AccelX2", "AccelY2", "AccelZ2"])?;

    for record in data_records {
        wtr.write_record(record.iter().map(|v| v.to_string()))?;
    }

    wtr.flush()?;
    Ok(())
}
