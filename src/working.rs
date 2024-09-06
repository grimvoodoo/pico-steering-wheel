//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

use bsp::entry;
use core::fmt::Write;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::{InputPin, OutputPin};
use heapless::String;
use panic_probe as _;

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico::{
    self as bsp,
    hal::{usb::UsbBus, Timer},
};
// use sparkfun_pro_micro_rp2040 as bsp;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
    watchdog::Watchdog,
};

use keyberon::{key_code::KeyCode, keyboard};
use usb_device::{
    bus::UsbBusAllocator,
    class_prelude,
    device::{StringDescriptors, UsbDeviceBuilder, UsbVidPid},
};
use usbd_human_interface_device::prelude::*;
use usbd_human_interface_device::{
    device::{
        keyboard::{KeyboardLedsReport, NKROBootKeyboard, NKROBootKeyboardConfig},
        DeviceClass,
    },
    interface::UsbAllocatable,
};
use usbd_serial::SerialPort;

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
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

    // let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

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

    let usb_bus = UsbBusAllocator::new(UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    let mut serial = SerialPort::new(&usb_bus);

    let keyboard_config = NKROBootKeyboardConfig::default();
    // let keyboard = keyboard_config.allocate(&usb_bus);
    let mut hid_class = UsbHidClassBuilder::new()
        .add_device(keyboard_config)
        .build(&usb_bus);

    // Create a USB device with a fake VID and PID
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .strings(&[StringDescriptors::default()
            .manufacturer("Fake company")
            .product("Serial port")
            .serial_number("TEST")])
        .unwrap()
        .device_class(2) // from: https://www.usb.org/defined-class-codes
        .build();

    let mut said_hello = false;

    loop {
        // A welcome message at the beginning
        if !said_hello && timer.get_counter().ticks() >= 2_000_000 {
            said_hello = true;
            let _ = serial.write(b"Hello, World!\r\n");

            let time = timer.get_counter().ticks();
            let mut text: String<64> = String::new();
            writeln!(&mut text, "Current timer ticks: {}", time).unwrap();

            // This only works reliably because the number of bytes written to
            // the serial port is smaller than the buffers available to the USB
            // peripheral. In general, the return value should be handled, so that
            // bytes not transferred yet don't get lost.
            let _ = serial.write(text.as_bytes());
        }
        if !usb_dev.poll(&mut [&mut hid_class]) {
        } else {
            println!("poll succeeded")
        }
        if button_1.is_high().unwrap() {
            led_1.set_high().unwrap();
            led_pin.set_high().unwrap();
            println!("Button_1 Pressed!");
        } else {
            led_1.set_low().unwrap();
            led_pin.set_low().unwrap();
        }
        if button_2.is_high().unwrap() {
            led_2.set_high().unwrap();
            println!("Button_2 Pressed!");
        } else {
            led_2.set_low().unwrap();
        }
        if button_3.is_high().unwrap() {
            led_3.set_high().unwrap();
            println!("Button_3 Pressed!");
        } else {
            led_3.set_low().unwrap();
        }
        if button_4.is_high().unwrap() {
            led_4.set_high().unwrap();
            println!("Button_4 Pressed!");
        } else {
            led_4.set_low().unwrap();
        }
        // info!("on!");
        // led_pin.set_high().unwrap();
        // delay.delay_ms(500);
        // info!("off!");
        // led_pin.set_low().unwrap();
        // delay.delay_ms(500);
    }
}

// End of file
