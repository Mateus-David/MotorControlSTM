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


```mermaid

graph TD
    %% Fonte de Alimentação e Terra Gerais
    subgraph Power_Supply ["Alimentação"]
        VCC["VCC (3.3V / 5V)"]
        GND["GND"]
    end

    %% Microcontrolador STM32
    subgraph STM32 ["Microcontrolador STM32"]
        %% Timers
        subgraph Perif_TIM ["Timers"]
            TIM1_CH1["TIM1 CH1 (PWM)"]
            TIM2_ENC["TIM2 CH1/CH2 (Encoder)"]
            TIM2_OC3["TIM2 CH3 (Output Compare)"]
            TIM3_IT["TIM3 (Base de Tempo / PID)"]
            TIM4_TIM["TIM4 (Base de Tempo)"]
            TIM6_IT["TIM6 (Detecção Long Press)"]
        end

        %% GPIO Out
        subgraph GPIO_Out ["Saídas GPIO"]
            LCD_DATA["PA (D4, D5, D6, D7)"]
            LCD_CTRL["PB (RS, RW, EN)"]
        end

        %% GPIO In / EXTI
        subgraph GPIO_In ["Entradas GPIO / EXTI"]
            EXTI0["EXTI0 (BT_CONFIG)"]
            EXTI4["EXTI4 (BT_DEC)"]
            EXTI_SET["EXTI (BT_SET)"]
        end

        %% Comunicação
        subgraph Comms ["Interface Serial"]
            USART1_TXRX["USART1 (TX / RX)"]
            BSP_COM["COM1 / BSP"]
        end
    end

    %% Periféricos Externos
    subgraph Interfaces_Entrada ["Interface de Usuário - Entradas"]
        BT1["Botão Config (BT_CONFIG)\nPull-Up Externo/Interno"]
        BT2["Botão Decremento/Mult (BT_DEC)\nPull-Up Externo/Interno"]
        BT3["Botão Set/Start-Stop (BT_SET)\nPull-Up Externo/Interno"]
    end

    subgraph Display_LCD ["Display LCD 1602 (Modo 4-bits)"]
        LCD_DISP["LCD 16x2"]
    end

    subgraph Driver_Motor ["Acionamento de Carga"]
        MOTOR_DRIVER["Driver do Motor DC\n(Ponte H / MOSFET)"]
        MOTOR["Motor DC"]
    end

    subgraph Feedback_Sensor ["Realimentação"]
        ENCODER["Encoder Quadratura\n(Canal A & Canal B)"]
    end

    subgraph Periferico_UART ["Comunicação Externa"]
        UART_DEV["Dispositivo Serial / PC\n(115200 / 9600 baud)"]
    end

    %% Conexões Elétricas e Sinalização
    
    %% Botões
    BT1 -->|Pull-Down na pressão| EXTI0
    BT2 -->|Pull-Down na pressão| EXTI4
    BT3 -->|Pull-Down na pressão| EXTI_SET
    GND --- BT1
    GND --- BT2
    GND --- BT3

    %% Display
    LCD_DATA -->|Dados 4-bits| LCD_DISP
    LCD_CTRL -->|Sinais de Controle| LCD_DISP
    VCC --- LCD_DISP
    GND --- LCD_DISP

    %% Actuação Motor
    TIM1_CH1 -->|Sinal PWM| MOTOR_DRIVER
    MOTOR_DRIVER -->|Tensão de Potência| MOTOR
    VCC --- MOTOR_DRIVER
    GND --- MOTOR_DRIVER

    %% Feedback Encoder
    MOTOR -.->|Eixo Mecânico| ENCODER
    ENCODER -->|Pulsos Canal A/B| TIM2_ENC
    VCC --- ENCODER
    GND --- ENCODER

    %% Serial
    USART1_TXRX <-->|Sinal TTL UART| UART_DEV
    BSP_COM <--> UART_DEV

    %% Estilização do Diagrama
    classDef stm32fill fill:#1e293b,stroke:#38bdf8,stroke-width:2px,color:#fff;
    classDef powerfill fill:#b91c1c,stroke:#f87171,stroke-width:1px,color:#fff;
    classDef extfill fill:#0f766e,stroke:#2dd4bf,stroke-width:1px,color:#fff;

    class STM32 stm32fill;
    class Power_Supply powerfill;
    class Interfaces_Entrada,Display_LCD,Driver_Motor,Feedback_Sensor,Periferico_UART extfill;



```


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

