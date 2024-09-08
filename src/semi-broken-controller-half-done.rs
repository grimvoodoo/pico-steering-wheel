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
use usbd_human_interface_device::page::Simulation;
use usbd_human_interface_device::prelude::UsbHidClassBuilder;

const BUTTON_COUNT: usize = 4;

// Descriptor based on the Logitech Gaming Keyboard
#[rustfmt::skip]
pub const LOGITECH_GAMING_KEYBOARD_REPORT_DESCRIPTOR: &[u8] = &[
    0x05, 0x01, //    Usage Page (Generic Desktop)
    0x09, 0x06, //    Usage (Keyboard)
    0xA1, 0x01, //    Collection (Application)
    0x05, 0x07, //        Usage Page (Keyboard/Keypad)
    0x19, 0xE0, //        Usage Minimum (Keyboard Left Control)
    0x29, 0xE7, //        Usage Maximum (Keyboard Right GUI)
    0x15, 0x00, //        Logical Minimum (0)
    0x25, 0x01, //        Logical Maximum (1)
    0x75, 0x01, //        Report Size (1)
    0x95, 0x08, //        Report Count (8)
    0x81, 0x02, //        Input (Data,Var,Abs,NWrp,Lin,Pref,NNul,Bit)
    0x95, 0x01, //        Report Count (1)
    0x75, 0x08, //        Report Size (8)
    0x81, 0x01, //        Input (Const,Ary,Abs)
    0x95, 0x05, //        Report Count (5)
    0x75, 0x01, //        Report Size (1)
    0x05, 0x08, //        Usage Page (LEDs)
    0x19, 0x01, //        Usage Minimum (Num Lock)
    0x29, 0x05, //        Usage Maximum (Kana)
    0x91, 0x02, //        Output (Data,Var,Abs,NWrp,Lin,Pref,NNul,NVol,Bit)
    0x95, 0x01, //        Report Count (1)
    0x75, 0x03, //        Report Size (3)
    0x91, 0x01, //        Output (Const,Ary,Abs,NWrp,Lin,Pref,NNul,NVol,Bit)
    0x95, 0x06, //        Report Count (6)
    0x75, 0x08, //        Report Size (8)
    0x15, 0x00, //        Logical Minimum (0)
    0x26, 0x97, 0x00, //        Logical Maximum (151)
    0x05, 0x07, //        Usage Page (Keyboard/Keypad)
    0x19, 0x00, //        Usage Minimum (Undefined)
    0x29, 0x97, //        Usage Maximum (Keyboard LANG8)
    0x81, 0x00, //        Input (Data,Ary,Abs)
    0xC0, //        End Collection
];

struct KeyBinding<P>
where
    P: InputPin,
{
    key: Simulation,
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
        key: Simulation::Ballast,
        pin: button_1,
    };
    let mut button_2 = KeyBinding {
        key: Simulation::Bicycle,
        pin: button_2,
    };
    let mut button_3 = KeyBinding {
        key: Simulation::BicycleCrank,
        pin: button_3,
    };
    let mut button_4 = KeyBinding {
        key: Simulation::Brake,
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
        .add_device(
            InterfaceBuilder::<InBytes8, OutBytes8, ReportSingle>::new(
                LOGITECH_GAMING_KEYBOARD_REPORT_DESCRIPTOR,
            )
            .unwrap()
            .description("Custom Keyboard")
            .idle_default(500.millis())
            .unwrap()
            .in_endpoint(10.millis())
            .unwrap()
            .with_out_endpoint(100.millis())
            .unwrap()
            .build(),
        )
        .build(&usb_bus);

    let mut simulator = UsbHidClassBuilder::new()
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
        let mut pressed_buttons: Vec<Simulation, BUTTON_COUNT> = Vec::new();

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
                .write_report([Simulation::NoEventIndicated; 6])
                .ok();
            // Send the clear down command so the keys are not left hanging
        } else {
            for button in pressed_buttons {
                simulator
                    .device::<BootKeyboard<'_, _>, _>()
                    .write_report([button; 6])
                    .ok();
                continue;
                // Send the keystroke
            }
        }
    }
}
