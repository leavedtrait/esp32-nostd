#![no_std]
#![no_main]

use dhtt11::DhttType;
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl, delay::Delay, gpio::{Io, Level, Output}, mcpwm::{
        operator::PwmPinConfig,
        timer::PwmWorkingMode,
        McPwm, PeripheralClockConfig,
    }, peripherals::Peripherals, prelude::*, rtc_cntl::Rtc, system::SystemControl
};
use esp_println::println;
use log::info;

mod dhtt11;
#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();

    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();
    esp_println::logger::init_logger_from_env();
    // let mut rtc = Rtc::new(peripherals.LPWR);
    // rtc.rwdt.set_timeout(2000.millis());

    //initialize io drive
    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    // Set GPIO2 as an output, and set its state high initially.
    let mut _led = Output::new(io.pins.gpio3, Level::High);
    // Initialize the Delay peripheral, and use it to toggle the LED state in a timely fashion
    let mut delay = Delay::new(&clocks);
    let mut array: [u32; 200] = [0; 200];

    // initialize peripheral
    let clock_cfg = PeripheralClockConfig::with_frequency(&clocks, 32.MHz()).unwrap();
    let mut mcpwm = McPwm::new(peripherals.MCPWM0, clock_cfg);

    mcpwm.operator0.set_timer(&mcpwm.timer0);
    let mut pwm_pin = mcpwm
        .operator0
        .with_pin_a(io.pins.gpio2, PwmPinConfig::UP_ACTIVE_HIGH);
    // start timer with timestamp values in the range of 0..=99 and a frequency
    // of 20 kHz
    let timer_clock_cfg = clock_cfg
        .timer_clock_with_frequency(50, PwmWorkingMode::Increase, 50.kHz())
        .unwrap();
    mcpwm.timer0.start(timer_clock_cfg);

    // pin will be high 50% of the time
    //pwm_pin.set_timestamp(50);
    /* `loop {}` or `panic!("...")` */
    // loop {
    //     for i in 0..array.len() {
    //         let x = i + 1;
    //         let midpoint = array.len() / 2;

    //         if x != array.len() {
    //             if i >= midpoint {
    //                 array[x] = array[i] - 1;
    //             } else if i < midpoint {
    //                 array[x] = array[i] + 1;
    //             }
    //         }
    //         //info!("Led off! for {} ms", array[i]);

    //         // esp_println::dbg!(led.set_high());
    //         // delay.delay_millis(array[i]);
    //         // esp_println::dbg!(led.set_low());
    //         delay.delay_millis(10);
    //         pwm_pin.set_timestamp(array[i].try_into().unwrap());
    //         info!("brightness {}%",array[i]);
    //         delay.delay_millis(10);
    //         // esp_println::dbg!(led.toggle());
    //         // info!("Led on! for {} ms", array[i]);
    //         // delay.delay_millis(array[i]);
    //         println!("index {} value {}", i, array[i])
    //     }
    // }
    //}
    println!("connecting...");
    let mut dhtsensor = io.pins.gpio25;

    let mut dht = dhtt11::Dht::new(&mut dhtsensor, DhttType::DHT11); // Using GPIO pin 4, DHT22 sensor type

    dht.begin(&mut delay);

    loop {
        for i in 0..array.len() {
            let x = i + 1;
            let midpoint = array.len() / 2;

            if x != array.len() {
                if i >= midpoint {
                    array[x] = array[i] - 1;
                } else if i < midpoint {
                    array[x] = array[i] + 1;
                }
            }
            pwm_pin.set_timestamp(array[i].try_into().unwrap());
            info!("brightness {}%", array[i]);
            delay.delay_millis(10);
        }
        if let Some(temp) = dht.read_temperature(&mut delay, false) {
            // Use the temperature value (Celsius)
            info!("temp {}", temp)
        }

        if let Some(humidity) = dht.read_humidity(&mut delay) {
            // Use the humidity value
            info!("humidity {}", humidity)
        }

        delay.delay_millis(2000); // Wait 2 seconds before next read
    }
}
