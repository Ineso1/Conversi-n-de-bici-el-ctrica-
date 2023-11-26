# Conversion-de-bici-electrica-
Con telemetría XD

# BLDC controller pinout

![image](https://github.com/Ineso1/Conversion-de-bici-electrica-/assets/84602829/58cb6a39-c17d-4182-9a79-42717728d032)

# BLDC Motor Controller with FreeRTOS
## BLDC_Controller_RTOS
This repository contains embedded software for controlling a Brushless DC (BLDC) motor using a microcontroller and FreeRTOS. It is designed to run on STMicroelectronics' hardware and uses the HAL library for hardware abstraction.

![image](https://github.com/Ineso1/Conversion-de-bici-electrica-/assets/84602829/c4f69a9d-6317-4949-9944-92ed20a11b43)

## Features

- Real-time motor control using Hall effect sensors
- ADC readings for throttle, voltage, and current
- PWM signal generation for motor phase control
- CAN communication for telemetry

### Prerequisites

- [STM32CubeMX](https://www.st.com/en/development-tools/stm32cubemx.html)
- [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html)
- [FreeRTOS](https://www.freertos.org/)

### Installation

1. Clone the repository:
   ```sh
   git clone [https://github.com/your-username/bldc-motor-controller.git](https://github.com/Ineso1/Conversion-de-bici-electrica-/tree/main/BLDC_Controller_RTOS_V1)

# Interrupt-Based BLDC Motor Controller
## BLDC_Controller_interrupction
This repository contains an interrupt-based software framework for controlling a Brushless DC (BLDC) motor using a microcontroller. The control logic is implemented in a modular way, with configuration and operation handled through a `Bldc` structure and associated functions.

![image](https://github.com/Ineso1/Conversion-de-bici-electrica-/assets/84602829/9b6637ee-9a81-46b2-9fa4-c4b9dd132fd4)

## Features

- Interrupt-driven motor control for real-time responsiveness.
- Structured `Bldc` object for easy configuration and management.
- Hall effect sensor processing for accurate motor control.
- ADC reading for throttle, voltage, and current sensing.
- PWM control for precise motor phase driving.

### Prerequisites

- A compatible microcontroller and development environment (e.g., STM32CubeIDE).
- Familiarity with embedded C programming and interrupt handling.

### Installation

1. Clone the repository:
   ```sh
   git clone [https://github.com/your-username/interrupt-bldc-controller.git](https://github.com/Ineso1/Conversion-de-bici-electrica-/tree/main/BLDC_Controller_interruption_V1)



# NucleoF303RE
https://www.st.com/en/evaluation-tools/nucleo-f303re.html

# MCP2515
https://github.com/eziya/STM32_SPI_MCP2515/tree/master

# BNO055
https://gitlab.pld.ttu.ee/iotcenter/bno055

