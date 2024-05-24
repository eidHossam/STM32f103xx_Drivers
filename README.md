# STM32f103xx MCU Drivers

## Overview

This repository hosts a collection of drivers for the STM32f103xx microcontroller unit (MCU). These drivers are designed to simplify the process of interfacing various peripherals and components with the STM32f103xx MCU, allowing developers to focus more on application logic rather than low-level hardware interaction.

## Supported Peripherals

The following peripherals are currently supported by the drivers in this repository:

1. **GPIO (General Purpose Input/Output)**

    - Provides functions for configuring and controlling GPIO pins.

2. **UART (Universal Asynchronous Receiver-Transmitter)**

    - Offers functions for UART communication, including initialization, transmission, and reception.

3. **SPI (Serial Peripheral Interface)**

    - Facilitates SPI communication with other devices using master or slave mode.

4. **I2C (Inter-Integrated Circuit)**

    - Offers functions for I2C communication, including initialization, master/slave mode configuration, data transmission, and reception.

5. **External Interrupts**

    - Supports configuration and handling of external interrupts (EXTI), allowing the MCU to respond to external events such as pin state changes.

6. **NVIC (Nested Vectored Interrupt Controller)**

    - Provides functions for configuring and managing interrupts, including enabling/disabling interrupts, setting priorities, and handling interrupt requests.

7. **SYSTICK Timer**
    - Implements functions for configuring and using the SYSTICK timer for generating periodic interrupts. This can be used for timekeeping, delays, and task scheduling.

## Getting Started

To get started using the drivers in your project, follow these steps:

1. **Clone the Repository**

2. **Include Headers:**

    - Copy the inc directory into your project's include path.
    - Include the necessary header files in your source files using #include "header.h".
    - **note**: make sure to add the Service and LIB folders to your project and them to your includes path.

3. **Link Source Files:**

    - Copy the src directory into your project's source directory.
    - Add the source files to your project's build system.

4. **Usage:**

    - Refer to the documentation and examples provided in the [Embedded Systems ](https://github.com/eidHossam/Master-Embedded-Systems) repository to understand how to use each driver.
