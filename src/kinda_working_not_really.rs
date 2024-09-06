#![no_std]
#![no_main]

use bsp::entry;
use usbd_human_interface_device::{device, prelude::UsbHidClassBuilder};
use core::fmt::Write;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::{InputPin, OutputPin};
use heapless::String;
use panic_probe as _;

use rp_pico::{
    self as bsp,
    hal::{usb::UsbBus, Timer},
};

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
    watchdog::Watchdog,
};

use keyberon::{
    debounce::Debouncer,
    hid::HidClass,
    key_code::{self, KbHidReport, KeyCode},
    keyboard::{Keyboard, Leds},
    layout::{Layers, Layout},
    matrix::Matrix,
};
use usb_device::class::UsbClass;
use usb_device::{
    bus::UsbBusAllocator,
    device::{StringDescriptors, UsbDeviceBuilder, UsbDeviceState, UsbVidPid},
};

struct NoLeds;

impl Leds for NoLeds {}

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Setup USB
    let usb_bus = UsbBusAllocator::new(UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    // Create the keyboard device
    let keyboard = Keyboard::new(NoLeds);

    // Setup keyberon HID class
    let mut hid = HidClass::new(keyboard, &usb_bus);

    let mut egboard = UsbHidClassBuilder::new()
		.add_interface(device::keyboard::NKROBootKeyboardConfig::default())
		.add_interface(device::mouse::WheelMouseConfig::default())
		.build(&usb_bus);

    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .strings(&[StringDescriptors::default()
            .manufacturer("FoxyFabrications")
            .product("USB Steering Wheel Buttons")
            .serial_number("TEST")])
        .unwrap()
        .device_class(0) // from: https://www.usb.org/defined-class-codes
        .build();

    // Configure Buttons and LED's
    let mut led_pin = pins.led.into_push_pull_output();
    let mut led_1 = pins.gpio6.into_push_pull_output();
    let mut led_2 = pins.gpio7.into_push_pull_output();
    let mut led_3 = pins.gpio8.into_push_pull_output();
    let mut led_4 = pins.gpio9.into_push_pull_output();

    let mut button_1 = pins.gpio2.into_pull_up_input();
    let mut button_2 = pins.gpio3.into_pull_up_input();
    let mut button_3 = pins.gpio4.into_pull_up_input();
    let mut button_4 = pins.gpio5.into_pull_up_input();

    loop {
        if usb_dev.poll(&mut [&mut hid]) {
            // Check if any buttons are pressed
            if button_1.is_low().unwrap() {
                // Send 'a' and light up LED 1
                led_1.set_high().unwrap();
                send_key(&mut hid, KeyCode::A);
            } else {
                led_1.set_low().unwrap();
            }

            if button_2.is_low().unwrap() {
                // Send 'b' and light up LED 2
                led_2.set_high().unwrap();
                send_key(&mut hid, KeyCode::B);
            } else {
                led_2.set_low().unwrap();
            }

            if button_3.is_low().unwrap() {
                // Send 'c' and light up LED 3
                led_3.set_high().unwrap();
                send_key(&mut hid, KeyCode::C);
            } else {
                led_3.set_low().unwrap();
            }

            if button_4.is_low().unwrap() {
                // Send 'e' and light up LED 4
                led_4.set_high().unwrap();
                send_key(&mut hid, KeyCode::D);
            } else {
                led_4.set_low().unwrap();
            }
        } else {
            led_1.set_low().unwrap();
            led_2.set_low().unwrap();
            led_3.set_low().unwrap();
            led_4.set_low().unwrap();
        }
    }
}

fn send_key(hid: &mut HidClass<UsbBus, Keyboard<NoLeds>>, key: KeyCode) {
    let mut report = KbHidReport::default();
    report.pressed(key);

    if hid.device_mut().set_keyboard_report(report.clone()) {
        while let Ok(0) = hid.write(report.as_bytes()) {}
    }
}
