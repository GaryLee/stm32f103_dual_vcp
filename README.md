# stm32f103_dual_vcp

An example to show how to use dual CDC VCP USB interfaces. 

## Hardware configuration

MCU: STM32F103C6Tx(72MHz, LQFP48, 32KB Flash, 10KB RAM)

## Software Development Environment

- STM32CubeMX V4.26.1
- STM32Cube FW_F1 V1.6.1
- gcc-arm-none-eabi-7-2017-q4-major

## Firmware Configureation

- Memory configuration:
    - Heap Size: 0x800
    - Stack Size: 0x800

- Perpherials
    - RCC 
        - High Speed Clock (HSE): Crystal/Ceramic Resonator
        - Low Speed Clock (LSE) : Crystal/Ceramic Resonator
    - USB
        - Device (FS)

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
    - USART1_TX: PA9
    - USART1_RX: PA10
    - USART2_TX: PA2
    - USART2_RX: PA3

**NOTE**: Check stm32f103_dual_vcp.ioc for detial.

**NOTE**: If you encountered the multiple re-definition issues when compiling 
    the source code, check the Makefile and remove any duplicate files in
    C_SOURCES instruction.

## Programming tips

### USB dual CDC VCP configuration.

To support dual class in single device, you have to add IAD(Interface Association Descriptor) to USBD_CDC_CfgFSDesc or USBD_CDC_CfgHsDesc depends on which 
full-speed or high-speed you are using.

USBD_CDC_CfgFSDesc and USBD_CDC_CfgHSDesc can be found in 
/src/stm32f103_dual_vcp/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c
/src/stm32f103_dual_vcp/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc/usbd_cdc.h

There are two template files /src/stm32f103_dual_vcp/usbd_cdc-dual_vcp-template.[ch]. 
Use them to overwrite usbd_cdc.[ch] files.

**NOTE**: usbd_cdc.[ch] are generated by STM32CubeMx. So, you have to repeat the overwriting action whenever you re-generate code from STM32CubeMx.

### USB event callback

There are several important USB callbacks function should be taken care of. 

#### UART configuration

There is a switch-case in usbd_cdc_if.c::CDC_Control_FS() and usbd_cdc_if.c::CDC_Control_HS(). 
It is used to configure UART property. The request is sent from host via USB.

STM32 has a predefined structure USBD_CDC_LineCodingTypeDef which can be used to represent 
the structure of line coding request.

```
    USBD_CDC_LineCodingTypeDef *line_coding = (USBD_CDC_LineCodingTypeDef *)pbuf;
```

