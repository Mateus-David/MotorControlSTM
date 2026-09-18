# MotorControlSTM



# MotorControlSTM

Closed-loop DC motor control firmware for the **STM32G431KB**, combining quadrature encoder feedback, a PID control loop, H-bridge PWM motor drive, an LCD1602 user interface, and persistent settings storage in flash.

## Overview

The system reads motor speed/position from a quadrature encoder, runs a PID controller to compute the required output, and drives the motor through an H-bridge via PWM. An LCD1602 display provides real-time feedback to the operator, and control buttons allow runtime adjustment of parameters, which are persisted to flash so they survive power cycles.

## Architecture

```mermaid
flowchart TD

subgraph group_app["Application Control"]
  node_main["Control Loop<br/>[main.c]"]
  node_interrupts["Interrupt Callbacks<br/>[stm32g4xx_it.c]"]
end

subgraph group_feedback["Motor Control"]
  node_pid["PID Controller<br/>[PID.c]"]
  node_encoder["Encoder Driver<br/>[motor_encoder.c]"]
  node_pwm["PWM Timer<br/>[main.c]"]
end

subgraph group_ui["User Interface"]
  node_lcd["LCD Driver<br/>[LCD1602.c]"]
end



subgraph group_platform["MCU Platform"]
  node_hal["HAL Runtime<br/>[stm32g4xx_hal.c]"]
  node_timhal["Timer HAL"]
  node_gpiohal["GPIO HAL"]
end

node_user(("Operator"))
node_buttons(("Control Buttons"))
node_motor["Motor"]
node_lcdhw["LCD Module"]
node_encoderhw(("Encoder Sensor"))

node_user -->|"presses"| node_buttons
node_buttons -->|"triggers"| node_interrupts
node_interrupts -->|"dispatches"| node_main
node_main -->|"reads counts"| node_encoder
node_encoderhw -->|"provides pulses"| node_encoder
node_main -->|"computes control"| node_pid
node_main -->|"sets compare"| node_pwm
node_pid -->|"returns output"| node_pwm
node_pwm -->|"drives PWM"| node_motor
node_main -->|"updates display"| node_lcd
node_lcd -->|"writes text"| node_lcdhw
node_main -->|"initializes"| node_hal
node_main -->|"configures timers"| node_timhal
node_main -->|"configures GPIO"| node_gpiohal

click node_main "https://github.com/mateus-david/motorcontrolstm/blob/main/Core/Src/main.c"
click node_interrupts "https://github.com/mateus-david/motorcontrolstm/blob/main/Core/Src/stm32g4xx_it.c"
click node_pid "https://github.com/mateus-david/motorcontrolstm/blob/main/Core/Src/PID.c"
click node_encoder "https://github.com/mateus-david/motorcontrolstm/blob/main/Core/Src/motor_encoder.c"
click node_pwm "https://github.com/mateus-david/motorcontrolstm/blob/main/Core/Src/main.c"
click node_lcd "https://github.com/mateus-david/motorcontrolstm/blob/main/Core/Src/LCD1602.c"
click node_hal "https://github.com/mateus-david/motorcontrolstm/blob/main/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal.c"
click node_timhal "https://github.com/mateus-david/motorcontrolstm/blob/main/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_tim.c"
click node_gpiohal "https://github.com/mateus-david/motorcontrolstm/blob/main/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_gpio.c"

classDef toneNeutral fill:#f8fafc,stroke:#334155,stroke-width:1.5px,color:#0f172a
classDef toneBlue fill:#dbeafe,stroke:#2563eb,stroke-width:1.5px,color:#172554
classDef toneAmber fill:#fef3c7,stroke:#d97706,stroke-width:1.5px,color:#78350f
classDef toneMint fill:#dcfce7,stroke:#16a34a,stroke-width:1.5px,color:#14532d
classDef toneRose fill:#ffe4e6,stroke:#e11d48,stroke-width:1.5px,color:#881337
classDef toneIndigo fill:#e0e7ff,stroke:#4f46e5,stroke-width:1.5px,color:#312e81
classDef toneTeal fill:#ccfbf1,stroke:#0f766e,stroke-width:1.5px,color:#134e4a
class node_main,node_interrupts toneBlue
class node_pid,node_encoder,node_pwm toneAmber
class node_lcd toneMint
class node_hal,node_timhal,node_gpiohal,node_user,node_buttons,node_motor,node_lcdhw,node_encoderhw toneIndigo
```

## Features

- **Encoder feedback** — quadrature decoding via `motor_encoder.c` for real-time speed/position sensing
- **PID control loop** — configurable PID controller (`PID.c`) computing the motor drive output each cycle
- **H-bridge motor drive** — PWM-based output stage controlling motor direction and speed
- **LCD1602 interface** — live status/parameter display via `LCD1602.c`
- **User input handling** — control buttons dispatched through interrupt callbacks (`stm32g4xx_it.c`)
- **Persistent settings** — parameters stored as records in flash, surviving power cycles

## Hardware

- MCU: STM32G431KB (Nucleo/custom board)
- Quadrature encoder (motor-mounted)
- H-bridge motor driver
- LCD1602 character display
- Push buttons for user input

## Project Structure

```
MotorControlSTM/
├── Core/
│   ├── Inc/                  # Application headers
│   └── Src/
│       ├── main.c            # Control loop, PWM setup, settings management
│       ├── stm32g4xx_it.c    # Interrupt callbacks (button inputs, timers)
│       ├── PID.c             # PID controller implementation
│       ├── motor_encoder.c   # Quadrature encoder driver
│       └── LCD1602.c         # LCD1602 driver
├── Drivers/                  # STM32G4xx HAL and CMSIS drivers
├── Debug/                    # Build output
├── MotorControl.ioc          # STM32CubeMX configuration
└── STM32G431KBTX_FLASH.ld    # Linker script
```

## Roadmap / TODO

- [ ] Tune PID gains
- [ ] Adjust LCD update/refresh rate
- [ ] Define initial system parameters (default settings)

