#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use defmt::info;
use esp_hal::clock::CpuClock;
use esp_hal::delay::Delay;
use esp_hal::i2c::master::{Config, I2c};
use esp_hal::main;
use esp_hal::time::{Duration, Instant, Rate};
use panic_rtt_target as _;
use sht31::prelude::*;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    // generator version: 1.0.1

    rtt_target::rtt_init_defmt!();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);
    
    // Configure I2C with lower frequency (100kHz) and timeout
    let i2c_config = Config::default()
        .with_frequency(Rate::from_hz(100_000)); // 100 kHz for better reliability
    
    let mut i2c = I2c::new(peripherals.I2C0, i2c_config)
        .unwrap()
        .with_scl(peripherals.GPIO19)
        .with_sda(peripherals.GPIO20);
    
    // Perform I2C scan
    info!("Starting I2C scan...");
    let mut found_devices = 0;
    for addr in 1..=127 {
        // Try to write to the address
        if i2c.write(addr, &[]).is_ok() {
            info!("Found device at address: 0x{:02X}", addr);
            found_devices += 1;
        }
    }
    if found_devices == 0 {
        info!("No I2C devices found! Check wiring and pull-up resistors.");
    } else {
        info!("I2C scan complete. Found {} device(s).", found_devices);
    }
    
    info!("Initializing SHT31 in single-shot mode...");
    let delay = Delay::new();
    let mut sht = SHT31::periodic(i2c, Periodic::new().with_mps(MPS::Half));
    
    // Disable the heater
    info!("Disabling heater...");
    if let Err(_) = sht.set_heating(true) {
        info!("Failed to disable heater");
    }

    sht.set_accuracy(Accuracy::High);
    sht.set_unit(TemperatureUnit::Celsius);

    // Wait for sensor to stabilize
    info!("Waiting for sensor to stabilize...");
    let delay_start = Instant::now();
    while delay_start.elapsed() < Duration::from_millis(2000) {}
    
    let status = sht.status().unwrap();
    info!("SHT31 checksum: {}", status.checksum_failed);
    info!("SHT31 last_command_processed: {}", status.last_command_processed);
    info!("SHT31 system_reset: {}", status.system_reset);
    info!("SHT31 t_alert: {}", status.t_alert);
    info!("SHT31 rh_alert: {}", status.rh_alert);
    info!("SHT31 heater_on: {}", status.heater_on);
    info!("SHT31 pending_alert: {}", status.pending_alert);
    sht.measure().unwrap();
    let delay_start = Instant::now();
    while delay_start.elapsed() < Duration::from_millis(5000) {}
    
    loop {
        info!("Taking measurement...");
        match sht.read() {
            Ok(reading) => {
                let temp = reading.temperature;
                let humidity = reading.humidity;
                info!("Temperature: {}", temp);
                info!("Humidity: {} %", humidity);
            }
            Err(_e) => {
                info!("Error reading sensor");
            }
        }
        let delay_start = Instant::now();
        while delay_start.elapsed() < Duration::from_millis(5000) {}
    }

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/esp-hal-v1.0.0/examples/src/bin
}
