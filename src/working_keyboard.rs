// Import all the required crates
#![no_std]
#![no_main]

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
use usbd_human_interface_device::page::Keyboard;
use usbd_human_interface_device::prelude::UsbHidClassBuilder;

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
        key: Keyboard::F13,
        pin: button_1,
    };
    let mut button_2 = KeyBinding {
        key: Keyboard::F14,
        pin: button_2,
    };
    let mut button_3 = KeyBinding {
        key: Keyboard::F15,
        pin: button_3,
    };
    let mut button_4 = KeyBinding {
        key: Keyboard::F16,
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
            .serial_number("TEST")])
        .unwrap()
        .build();

    info!("Starting");

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
            keyboard
                .device::<BootKeyboard<'_, _>, _>()
                .write_report([Keyboard::NoEventIndicated; 6])
                .ok();
            // Send the clear down command so the keys are not left hanging
        } else {
            for button in pressed_buttons {
                keyboard
                    .device::<BootKeyboard<'_, _>, _>()
                    .write_report([button; 6])
                    .ok();
                continue;
                // Send the keystroke
            }
        }
    }
}
