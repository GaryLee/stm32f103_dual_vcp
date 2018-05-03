# stm32f103_dual_vcp

An example to show how to use dual CDC VCP USB interfaces. 

## Hardware configuration

MCU: STM32F103C6Tx(72MHz, LQFP48, 32KB Flash, 10KB RAM)

## Software Development Environment

- STM32CubeMX V4.24.0
- STM32Cube FW_F1 V1.6.0
- gcc-arm-none-eabi-7-2017-q4-major or above.

## Firmware Configureation

- Memory configuration:
    - Heap Size: 0x400
    - Stack Size: 0x800

- Perpherials
    - RCC 
        - High Speed Clock (HSE): Crystal/Ceramic Resonator
        - Low Speed Clock (LSE) : Crystal/Ceramic Resonator
    - USB
        - Device (FS) : Checked

    - USART1
        - Mode: Asynchronous

    - USART2
        - Mode: Asynchronous

- MiddleWares
    - USB_DEVICE
        - Class for FS IP: Communication Device Class (Virtual Port Com)

- Pin configuration
    - USB_DM: PA11
    - USB_PM: PA12
    - USART2_TX: PA2
    - USART2_RX: PA3
    - USART2_CTS: PA0(optional)
    - USART2_RTS: PA1(optional)
    - USART3_TX: PB10
    - USART3_RX: PB11
    - USART3_CTS: PB13(optional)
    - USART3_RTS: PB14(optional)

**NOTE**: Check stm32f103_dual_vcp.ioc for detial.
**NOTE**: If you encountered the multiple re-definition issues when compiling the source code, 
    check the Makefile and remove any duplicate files in C_SOURCES instruction.