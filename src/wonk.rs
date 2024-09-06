#![no_std]
#![no_main]

use core::iter::IntoIterator;
use core::usize;

use bsp::entry;
use bsp::hal;
use cortex_m::prelude::*;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::*;
use fugit::ExtU32;
use hal::gpio::{DynPinId, FunctionSioInput, Pin, PullUp};
use heapless::FnvIndexMap;
use heapless::Vec;
use keyberon::key_code::{KbHidReport, KeyCode};
use keyberon::keyboard;
use panic_probe as _;
use rp_pico::hal::pwm::B;
use rp_pico as bsp;
use rp_pico::pac;
#[allow(clippy::wildcard_imports)]
use usb_device::class_prelude::*;
use usb_device::prelude::*;
use usbd_human_interface_device::device::keyboard::BootKeyboard;
use usbd_human_interface_device::page::Keyboard;
use usbd_human_interface_device::prelude::UsbHidClassBuilder;
use usbd_human_interface_device::UsbHidError;

const BUTTON_COUNT: usize = 4;

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();

    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
    let clocks = hal::clocks::init_clocks_and_plls(
        bsp::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

    let sio = hal::Sio::new(pac.SIO);
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    info!("Starting");

    // USB setup
    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    let mut keyboard = UsbHidClassBuilder::new()
        .add_device(usbd_human_interface_device::device::keyboard::BootKeyboardConfig::default())
        .build(&usb_bus);

    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x1209, 0x0001))
        .strings(&[StringDescriptors::default()
            .manufacturer("Foxy Fabrications")
            .product("Steering Wheel Additional Buttons")
            .serial_number("TEST")])
        .unwrap()
        .build();

    // GPIO setup for buttons
    let mut input_pins: [Pin<DynPinId, FunctionSioInput, PullUp>; BUTTON_COUNT] = [
        pins.gpio2.into_pull_up_input().into_dyn_pin(),
        pins.gpio3.into_pull_up_input().into_dyn_pin(),
        pins.gpio4.into_pull_up_input().into_dyn_pin(),
        pins.gpio5.into_pull_up_input().into_dyn_pin(),
    ];

    let mut last_report: Vec<KeyCode, BUTTON_COUNT> = Vec::new();
    let mut input_count_down = timer.count_down();
    input_count_down.start(50.millis());

    loop {
        // Retrieve the BootKeyboard device from UsbHidClass
        let mut keyboard_device = usb_hid_class.device::<BootKeyboard<B>, _>();

        // Attempt to read a report from the keyboard device
        match keyboard_device.read_report() {
            Ok(report) => {
                // Process the report if it was successfully read
                for key in report.keys.iter() {
                    let keyboard_code = key_to_keyboard_code(*key);
                    // Handle key presses, log or take action based on the pressed key
                    println!("Key pressed: {}", keyboard_code);
                }
            }
            Err(e) => {
                // Handle read error if necessary (or just ignore it)
                println!("Failed to read report: {:?}", e);
            }
        }

        // Add some delay between polling to avoid tight looping
        delay(1); // Adjust the delay as necessary for your platform
    }
}

fn get_button_states(
    pins: &mut [Pin<DynPinId, FunctionSioInput, PullUp>],
) -> [(KeyCode, bool); BUTTON_COUNT] {
    let mut button_states: [(KeyCode, bool); BUTTON_COUNT] = [
        (KeyCode::A, false),
        (KeyCode::B, false),
        (KeyCode::C, false),
        (KeyCode::D, false),
    ];

    if pins[0].is_low().unwrap() {
        button_states[0].1 = true;
    }
    if pins[1].is_low().unwrap() {
        button_states[1].1 = true;
    }
    if pins[2].is_low().unwrap() {
        button_states[2].1 = true;
    }
    if pins[3].is_low().unwrap() {
        button_states[3].1 = true;
    }

    button_states
}

fn build_reports(button_states: &[(KeyCode, bool); BUTTON_COUNT]) -> KbHidReport {
    let mut report = KbHidReport::default();

    for &(keycode, pressed) in button_states.iter() {
        if pressed {
            report.pressed(keycode);
        }
    }

    report
}

fn key_to_keyboard_code(key: KeyCode) -> Keyboard {
    match key {
        KeyCode::A => Keyboard::A,
        KeyCode::B => Keyboard::B,
        KeyCode::C => Keyboard::C,
        KeyCode::D => Keyboard::D,
        KeyCode::E => Keyboard::E,
        KeyCode::F => Keyboard::F,
        KeyCode::G => Keyboard::G,
        KeyCode::H => Keyboard::H,
        KeyCode::I => Keyboard::I,
        KeyCode::J => Keyboard::J,
        KeyCode::K => Keyboard::K,
        KeyCode::L => Keyboard::L,
        KeyCode::M => Keyboard::M,
        KeyCode::N => Keyboard::N,
        KeyCode::O => Keyboard::O,
        KeyCode::P => Keyboard::P,
        KeyCode::Q => Keyboard::Q,
        KeyCode::R => Keyboard::R,
        KeyCode::S => Keyboard::S,
        KeyCode::T => Keyboard::T,
        KeyCode::U => Keyboard::U,
        KeyCode::V => Keyboard::V,
        KeyCode::W => Keyboard::W,
        KeyCode::X => Keyboard::X,
        KeyCode::Y => Keyboard::Y,
        KeyCode::Z => Keyboard::Z,
        // Add more mappings as necessary for other keys.
        _ => Keyboard::NoEventIndicated, // For unmapped keys
    }
}
