// Import all the required crates
#![no_std]
#![no_main]

use core::default;

use bsp::entry;
use bsp::hal;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::*;
use heapless::Vec;
use panic_probe as _;
use rp_pico as bsp;
use rp_pico::pac;
#[allow(clippy::wildcard_imports)]
use usb_device::class_prelude::*;
use usb_device::prelude::*;
use usbd_human_interface_device::device::keyboard::BootKeyboard;
use usbd_human_interface_device::prelude::UsbHidClassBuilder;

#[derive(Copy, Clone, Debug, Eq, PartialEq, Format)]
pub enum Keyboard {
    A = 0x04,
    B = 0x05,
    C = 0x06,
    D = 0x07,
    NoEventIndicated = 0x00,
    ErrorRollOver = 0x01,
    POSTFail = 0x02,
    ErrorUndefined = 0x03,
    // Add your custom keycodes here
    JoystickButton1 = 0x7400,
    JoystickButton2 = 0x7401,
    JoystickButton3 = 0x7402,
    JoystickButton4 = 0x7403,
    JoystickButton5 = 0x7404,
    JoystickButton6 = 0x7405,
    JoystickButton7 = 0x7406,
    JoystickButton8 = 0x7407,
    JoystickButton9 = 0x7408,
    JoystickButton10 = 0x7409,
    JoystickButton11 = 0x7410,
    // You can keep adding custom hex codes like this
}

impl Default for Keyboard {
    fn default() -> Self {
        Keyboard::NoEventIndicated
    }
}

#[derive(Default)]
pub struct BootKeyboardReport {
    pub keys: [Keyboard; 6], // 6-key rollover support
}

impl BootKeyboardReport {
    pub fn new<K: IntoIterator<Item = Keyboard>>(keys: K) -> Self {
        let mut report = Self::default();

        let mut i = 0;
        for k in keys {
            if i < report.keys.len() {
                report.keys[i] = k;
                i += 1;
            }
        }

        report
    }

    pub fn write_report(&self) {
        // Mock function for sending the HID report
        println!("Sending HID report: {:?}", self.keys);
    }
}

const BUTTON_COUNT: usize = 4;

struct KeyBinding<P>
where
    P: InputPin,
{
    key: Keyboard,
    pin: P,
}

impl<P> KeyBinding<P>
where
    P: InputPin,
{
    pub fn is_pressed(&mut self) -> bool {
        self.pin.is_high().unwrap_or(false)
    }
}

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

    let mut led_pin = pins.gpio25.into_push_pull_output();
    let mut led_1 = pins.gpio6.into_push_pull_output();
    let mut led_2 = pins.gpio7.into_push_pull_output();
    let mut led_3 = pins.gpio8.into_push_pull_output();
    let mut led_4 = pins.gpio9.into_push_pull_output();

    let button_1 = pins.gpio2.into_pull_up_input();
    let button_2 = pins.gpio3.into_pull_up_input();
    let button_3 = pins.gpio4.into_pull_up_input();
    let button_4 = pins.gpio5.into_pull_up_input();

    let mut button_1 = KeyBinding {
        key: Keyboard::JoystickButton1,
        pin: button_1,
    };
    let mut button_2 = KeyBinding {
        key: Keyboard::JoystickButton2,
        pin: button_2,
    };
    let mut button_3 = KeyBinding {
        key: Keyboard::JoystickButton3,
        pin: button_3,
    };
    let mut button_4 = KeyBinding {
        key: Keyboard::JoystickButton4,
        pin: button_4,
    };

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
            .serial_number("PICO")])
        .unwrap()
        .build();

    info!("Starting");

    led_pin.set_high().unwrap();

    loop {
        if !usb_dev.poll(&mut [&mut keyboard]) {
            continue;
        }
        // Set the default that no keys are pressed at the beggining of the loop
        let mut no_keys_pressed = true;
        let mut pressed_buttons: Vec<Keyboard, BUTTON_COUNT> = Vec::new();

        // If any key is pressed then the no_keys_pressed state is set to false to allow the keys to be sent
        if button_1.is_pressed() {
            no_keys_pressed = false;
            pressed_buttons.push(button_1.key).unwrap();
            led_1.set_high().unwrap();
        }
        if button_2.is_pressed() {
            no_keys_pressed = false;
            pressed_buttons.push(button_2.key).unwrap();
            led_2.set_high().unwrap();
        }
        if button_3.is_pressed() {
            no_keys_pressed = false;
            pressed_buttons.push(button_3.key).unwrap();
            led_3.set_high().unwrap();
        }
        if button_4.is_pressed() {
            no_keys_pressed = false;
            pressed_buttons.push(button_4.key).unwrap();
            led_4.set_high().unwrap();
        }

        if no_keys_pressed {
            led_1.set_low().unwrap();
            led_2.set_low().unwrap();
            led_3.set_low().unwrap();
            led_4.set_low().unwrap();
            let report = BootKeyboardReport::new([Keyboard::NoEventIndicated; 6]);
            report.write_report();
        } else {
            let report = BootKeyboardReport::new(pressed_buttons);
            report.write_report();
        }
    }
}
