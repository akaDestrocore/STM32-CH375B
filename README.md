
## TODO
- Y] USB host basic api
- Y] HID Host
- Y] HID Host Mouse
- Y] HID Host Keyboard
- Y] Optimize HID Host code
- Y] HID Device(composite device[mouse, keyboard])
- Y] Finish HID data (keyboard and mouse) transmission
- Y] HID data dynamic modification
- Y] HID Report Descriptor Clon

## Environment
1. Hardware
    - stm32-f4discovery
    - CH375 (USB Bus interface Module) [you need two, one for mouse, other for keyboard]

2. Develop Environment
    - Ubunutu
    - STM32CubeMX
    - openocd (For stlink debug progame)
    - gcc-arm-none-eabi
    - vscode (with Arm extensions)



## Project Structure
- Drivers
    - ch375: (include libraries,  usbhost, usbhid, hidkeyboard, HIDMouse_t)
    - ...
- Middlewares/ST/STM32_USB_Device_Library
    - Class/CustomHID

### CH375 Use
- CH375A
    - USART2
    - PE14     (Int Pin)
- CH375B
    - USART3
    - PE15     (Int Pin)
- Log Out
    - UART4
