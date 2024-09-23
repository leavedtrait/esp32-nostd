
use esp_hal::{
    delay::Delay,
    gpio::{Flex, InputPin, OutputPin},
    peripheral::Peripheral,
};
use esp_println::println;

const DHT22: u8 = 22;
const DHT11: u8 = 11;

const TIMEOUT: u32 = 0xFFFFFFFF;
const MIN_INTERVAL: u32 = 2000;

pub enum DhttType {
    DHT11,
    DHT22
}
pub struct Dht<'a, P> {
    pin: Flex<'a, P>,
    last_read_time: u32,
    sensor_type: DhttType,
    max_cycles: u32,
    pull_time: u32,
}

impl<'a, P> Dht<'a, P>
where
    P: InputPin + OutputPin,
{
    pub fn new(pin: impl Peripheral<P = P> + 'a, sensor_type: DhttType) -> Self {
        let max_cycles = 1000; // max_cycles for timeout handling
        let pull_time = 55; // pull time default (microseconds)

        Dht {
            pin: Flex::new(pin),
            last_read_time: 0,
            sensor_type,
            max_cycles,
            pull_time,
        }
    }

    /// Begin communication with the sensor
    pub fn begin(&mut self, delay: &mut Delay) {
        self.pin.set_high();
        println!("setting data line high ");
        delay.delay_millis(MIN_INTERVAL); // Ensures >= MIN_INTERVAL
        self.last_read_time = 0; // Reset the last read time
    }

    /// Read temperature from the sensor
    pub fn read_temperature(&mut self, delay: &mut Delay, fahrenheit: bool) -> Option<f32> {
        if let Some(data) = self.read_sensor_data(delay) {
            let mut temperature = match self.sensor_type {
                DhttType::DHT11 => (data[2] as f32) + (data[3] as f32) * 0.1,
                DhttType::DHT22 => ((data[2] as u16 & 0x7F) << 8 | data[3] as u16) as f32 * 0.1,
            };

            if data[2] & 0x80 != 0 {
                temperature = -temperature;
            }

            if fahrenheit {
                Some(temperature * 1.8 + 32.0) // Convert to Fahrenheit
            } else {
                Some(temperature)
            }
        } else {
            None
        }
    }

    /// Read humidity from the sensor
    pub fn read_humidity(&mut self, delay: &mut Delay) -> Option<f32> {
        if let Some(data) = self.read_sensor_data(delay) {
            let humidity = match self.sensor_type {
                DhttType::DHT11 => (data[0] as f32) + (data[1] as f32) * 0.1,
                DhttType::DHT22 => ((data[0] as u16) << 8 | data[1] as u16) as f32 * 0.1,
            };

            Some(humidity)
        } else {
            None
        }
    }

    /// Reads raw sensor data
    fn read_sensor_data(&mut self, delay: &mut Delay) -> Option<[u8; 5]> {
        let mut data = [0u8; 5];

        // Send start signal
        self.pin.set_low();
        delay.delay_millis(18); // Pull low for 18 ms
        self.pin.set_high();
        delay.delay_micros(self.pull_time); // Wait for sensor response
        self.pin.set_low();

        // Now read the 40 bits of data
        for i in 0..40 {
            let result1 = self.expect_pulse(delay, false);
            let result2 = self.expect_pulse(delay, true);
            if result1.is_err() || result2.is_err() {
                return None;
            }

            data[i / 8] <<= 1;
            if self.pin.is_high() {
                data[i / 8] |= 1;
            }
        }

        // Verify checksum
        if data[4] == ((data[0] + data[1] + data[2] + data[3]) & 0xFF) {
            Some(data)
        } else {
            None
        }
    }

    /// Expect a pulse on the data line (low or high) and return its duration
    fn expect_pulse(&mut self, delay: &mut Delay, level: bool) -> Result<u32, ()> {
        let mut count = 0;
        while self.pin.is_high() == level {
            count += 1;
            if count >= self.max_cycles {
                return Err(());
            }
            delay.delay_micros(1);
        }
        Ok(count)
    }
}
