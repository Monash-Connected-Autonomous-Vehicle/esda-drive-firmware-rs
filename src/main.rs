use anyhow::Ok;
use esp_idf_hal::{gpio::*, timer};
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::task::*;
use std::f32::consts::PI;

const ENCODER_TICK_QUEUE_SIZE: usize = 7000;
const VELOCITY_SAMPLE_FREQUENCY: u64 = 50000;
const VELOCITY_SAMPLE_PERIOD: f32 = 1.0/VELOCITY_SAMPLE_FREQUENCY as f32;

const ENCODER_TICKS_PER_REVOLUTION: f32 = 1024.0;
const WHEEL_RADIUS: f32 = 0.13; // Unit: Metres

#[derive(Clone, Copy)]
enum EncoderID {
    Left,
    Right
}

#[derive(Clone, Copy)]
enum Direction {
    Clockwise,
    Anticlockwise
}

impl Direction {
    fn as_f32(self) -> f32 {
        match self {
            Self::Clockwise => 1.0,
            Self::Anticlockwise => -1.0
        }
    }
}

#[derive(Clone, Copy)]
struct EncoderTickEvent {
    source: EncoderID,
    direction: Direction
}

fn main() -> anyhow::Result<()>{
    // It is necessary to call this function once. Otherwise some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_svc::sys::link_patches();

    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();

    log::info!("Initialising ESDA Drive Firmware...");
    let peripherals = Peripherals::take()?;

    let mut encoder_left_driver_a = PinDriver::input(peripherals.pins.gpio18)?;
    let mut encoder_left_driver_d = PinDriver::input(peripherals.pins.gpio19)?;
    let mut encoder_right_driver_a = PinDriver::input(peripherals.pins.gpio34)?;
    let mut encoder_right_driver_d = PinDriver::input(peripherals.pins.gpio33)?;

    encoder_left_driver_a.set_pull(Pull::Down)?;
    encoder_left_driver_d.set_pull(Pull::Down)?;
    encoder_right_driver_a.set_pull(Pull::Down)?;
    encoder_right_driver_d.set_pull(Pull::Down)?;

    // Set encoder pin A interrupts to rising edge
    encoder_left_driver_a.set_interrupt_type(InterruptType::PosEdge)?;
    encoder_right_driver_a.set_interrupt_type(InterruptType::PosEdge)?;
    
    let encoder_tick_queue_master = queue::Queue::<EncoderTickEvent>::new(ENCODER_TICK_QUEUE_SIZE);
    let encoder_tick_queue_left = unsafe { queue::Queue::<EncoderTickEvent>::new_borrowed(encoder_tick_queue_master.as_raw()) };
    let encoder_tick_queue_right = unsafe { queue::Queue::<EncoderTickEvent>::new_borrowed(encoder_tick_queue_master.as_raw()) };
    let encoder_tick_queue_consume = unsafe { queue::Queue::<EncoderTickEvent>::new_borrowed(encoder_tick_queue_master.as_raw()) };

    let velocity_measure_timer_conf = timer::config::Config::new().auto_reload(true);
    let mut velocity_measure_timer = timer::TimerDriver::new(peripherals.timer00, &velocity_measure_timer_conf)?;
    velocity_measure_timer.set_alarm(velocity_measure_timer.tick_hz() / VELOCITY_SAMPLE_FREQUENCY)?;

    // Subscribe to each notifier
    // let notifier_left = notification.notifier();
    // let notifier_right = notification.notifier();
    unsafe {
        encoder_left_driver_a.subscribe(move || {
            encoder_tick_queue_left.send_back(EncoderTickEvent { 
                source: EncoderID::Left, 
                direction: match encoder_left_driver_d.get_level() {
                    Level::Low => Direction::Clockwise,
                    Level::High => Direction::Anticlockwise,
                }
            }, 0).unwrap();
        })?;
        encoder_right_driver_a.subscribe(move || {
            encoder_tick_queue_right.send_back(EncoderTickEvent { 
                source: EncoderID::Right, 
                direction: match encoder_right_driver_d.get_level() {
                    Level::Low => Direction::Clockwise,
                    Level::High => Direction::Anticlockwise,
                }
            }, 0).unwrap();
        })?;
    }

    unsafe {
        velocity_measure_timer.subscribe(move || {
            loop { 
                let mut left_ticks: f32 = 0.0;
                let mut right_ticks: f32 = 0.0;
                // Tally up all the ticks the encoders had in each direction
                while let Some((tick_event, should_yield)) = encoder_tick_queue_consume.recv_front(0) {
                    // Give way to higher priority tasks
                    if should_yield {
                        do_yield();
                    }
                    match tick_event.source {
                        EncoderID::Left => left_ticks+=1.0_f32*tick_event.direction.as_f32(),
                        EncoderID::Right => right_ticks+=1.0_f32*tick_event.direction.as_f32(),
                    }
                }
                
                let left_ticks = left_ticks as f32;
                let right_ticks = right_ticks as f32;
                let left_velocity = (left_ticks*WHEEL_RADIUS*PI)/(ENCODER_TICKS_PER_REVOLUTION*VELOCITY_SAMPLE_PERIOD);
                let right_velocity = (right_ticks*WHEEL_RADIUS*PI)/(ENCODER_TICKS_PER_REVOLUTION*VELOCITY_SAMPLE_PERIOD);
                log::info!("L: {left_ticks}t|{left_velocity}m/s R: {right_ticks}t|{right_velocity}m/s");
            }
        })?;
    }

    // Enable the velocity sample (queue processing) timer
    velocity_measure_timer.enable_interrupt()?;
    velocity_measure_timer.enable_alarm(true)?;
    velocity_measure_timer.enable(true)?;

    // Enable the encoder interrupts
    encoder_left_driver_a.enable_interrupt()?;
    encoder_right_driver_a.enable_interrupt()?;

    Ok(())
}
