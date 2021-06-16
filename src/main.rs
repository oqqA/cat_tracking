#![no_std]
#![no_main]

use panic_halt as _;

use cortex_m_rt::entry;
use stm32f4xx_hal::{
    prelude::*, 
    stm32,
    otg_fs::{UsbBus, USB},
};
use usb_device::prelude::*;

fn u16_to_u8_arr_ascii(arr: &mut [u8], a: &mut u16) {
    let mut i = 5u8;
    while *a != 0u16 {
        i-=1u8;
        arr[i as usize]=(*a%10u16) as u8 + 0x30;
        *a/=10;
    }
}

static mut EP_MEMORY: [u32; 1024] = [0; 1024];

#[entry]
fn main() -> ! {
    let dp = stm32::Peripherals::take().unwrap();

    let gpioa = dp.GPIOA.split();
    let gpioc = dp.GPIOC.split();

    let mut led = gpioc.pc13.into_push_pull_output();
    led.set_high().unwrap();

    let rcc = dp.RCC.constrain();

    let clocks = rcc.cfgr
        .use_hse(25.mhz())
        .sysclk(48.mhz())
        .require_pll48clk()
        .freeze();

    let usb = USB {
        usb_global: dp.OTG_FS_GLOBAL,
        usb_device: dp.OTG_FS_DEVICE,
        usb_pwrclk: dp.OTG_FS_PWRCLK,
        pin_dm: gpioa.pa11.into_alternate_af10(),
        pin_dp: gpioa.pa12.into_alternate_af10(),
        hclk: clocks.hclk(),
    };

    let usb_bus = UsbBus::new(usb, unsafe { &mut EP_MEMORY });

    let mut serial = usbd_serial::SerialPort::new(&usb_bus);

    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .manufacturer("Fake company")
        .product("Serial port")
        .serial_number("TEST")
        .device_class(usbd_serial::USB_CLASS_CDC)
        .build();
    
    let mut flag1 = false;

    loop {
        if !usb_dev.poll(&mut [&mut serial]) {
            continue;
        }
        
        let mut buf = [0u8; 64];

        match serial.read(&mut buf) {
            Ok(count) if count > 0 => {
                // Echo back in upper case
                for c in buf[0..count].iter_mut() {
                    if 0x31 == *c {
                        led.set_high().unwrap();
                    }
                    if 0x32 == *c {
                        led.set_low().unwrap();
                    }
                    if 0x33 == *c {
                        flag1 = !flag1;
                    }
                }

                let mut write_offset = 0;
                while write_offset < count {
                    match serial.write(&buf[write_offset..count]) {
                        Ok(len) if len > 0 => {
                            write_offset += len;
                        }
                        _ => {}
                    }
                }
            }
            _ => {}
        }

        if flag1 {
            let mut millivolts = 54532u16;
            
            let mut arr = [0u8; 6];
            u16_to_u8_arr_ascii(&mut arr,&mut millivolts);
            arr[5] = 0x0a;

            let mut write_offset = 0;
            while write_offset < 6 {
                match serial.write(&arr) {
                    Ok(len) if len > 0 => {
                        write_offset += len;
                    }
                    _ => {}
                }
            }
        }
    }
}