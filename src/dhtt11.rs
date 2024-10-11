use esp_hal::{
    delay::Delay,
    gpio::{Flex, InputPin, OutputPin},
};
use esp_println::println;
use log::info;

//const TIMEOUT: u32 = 0xFFFFFFFF;
const MIN_INTERVAL: u32 = 2000;
//const DHT11_START_LOW: u32 = 18; // milliseconds to pull data line low for DHT11
const DHT11_START_HIGH: u32 = 20; // microseconds to pull data line high

pub enum DhttType {
    DHT11,
    DHT22,
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
    pub fn new(pin: Flex<'a, P>, dht: DhttType) -> Self {
        let max_cycles = 1000; // max_cycles for timeout handling
        let pull_time = match dht {
            DhttType::DHT11 => DHT11_START_HIGH,
            DhttType::DHT22 => 75, // for DHT22 typically
        };
        Self {
            pin,
            last_read_time: 0,
            sensor_type: dht,
            max_cycles,
            pull_time,
        }
    }

    /// Begin communication with the sensor
    pub fn begin(&mut self, delay: &mut Delay) {
        self.pin.set_high();
        info!("Setting data line high");
        delay.delay_millis(MIN_INTERVAL); // Ensure >= MIN_INTERVAL
        self.last_read_time = 0; // Reset the last read time
    }

    // Send start signal and read data from the sensor
    pub fn read(&mut self, delay: &mut Delay) -> Result<(), &'static str> {
        // Send the start signal
        self.send_start_signal(delay);

        // Read the response from the sensor
        let mut data: [u8; 5] = [0; 5];

        // Read each byte (8 bits) from the sensor
        for i in 0..40 {
            data[i/8] <<=1;
            if self.read_bit(delay) {
                data[i/8] |= 1;
            }
            
        }

        for i in 0..data.len() {
            println!("data {}", data[i]);
        }

        // Validate checksum (data[4] should be the sum of data[0..3])
        let checksum = data[0]
            .wrapping_add(data[1])
            .wrapping_add(data[2])
            .wrapping_add(data[3]);
        if checksum != data[4] {
            return Err("Checksum error");
        }

        // Extract temperature and humidity values
        let humidity = u16::from(data[1]) * 10 + u16::from(data[0]); // For DHT11, the humidity is in the first byte
        let mut temperature = i16::from(data[3] & 0x7f)* 10 + i16::from(data[2]); // For DHT11, the temperature is in the third byte
        if(data[2] & 0x80) != 0 {
            temperature = -temperature;
        }

        // Print out the sensor values
        println!("Humidity: {}%", humidity);
        println!("Temperature: {}°C", temperature);

        Ok(())
    }
  

    /// Send the start signal to the DHT sensor
    fn send_start_signal(&mut self, delay: &mut Delay) {
        self.pin.set_as_output();
        self.pin.set_high();
        delay.delay_millis(1);
        self.pin.set_low();
        delay.delay_millis(20);
        self.pin.set_high();
        self.pin.set_as_input(esp_hal::gpio::Pull::None);
        delay.delay_micros(40);
        self.read_bit(delay);
    }
    fn read_bit(&mut self, delay: &mut Delay) -> bool {
        let low = self.wait_for_state(true, delay);
        let high: u32 = self.wait_for_state(false, delay);
        high>low
    }

    /// Wait for the pin to change to the specified state (low = false, high = true)
    fn wait_for_state(&mut self, state: bool, delay: &mut Delay) -> u32 {
        let mut cycles = 0;
        //self.pin.set_as_input(esp_hal::gpio::Pull::Up);
        while self.pin.is_high() != state {
            cycles += 1;
            if cycles > self.max_cycles {
                return 0;
            }
            delay.delay_micros(1); // Short delay to avoid busy waiting
        }

        cycles
    }

   
}
