/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    stm32g4xx_it.c
 * @brief   Interrupt Service Routines.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32g4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "PID.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define COUNTS_PER_REV    4000.0f     // 600 linhas x4 (TIM_ENCODERMODE_TI12)
#define ALPHA_FILTRO 0.05f
#define CONTROL_PERIOD_S 0.002f
#define WINDOW_SAMPLES 25

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
volatile uint8_t emergency_stop = 0;
float rpm_rampa = 0.0f; // Setpoint dinâmico (a rampa) que o PID vai perseguir
static int32_t pos_antiga = 0;

// --- Buffer circular para a média móvel de velocidade ---
static int32_t delta_buffer[WINDOW_SAMPLES] = {0};
static uint8_t buffer_index = 0;
static int32_t soma_delta_pulsos = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim6;
/* USER CODE BEGIN EV */


extern volatile encoder Encoder;
extern UART_HandleTypeDef huart1;
extern volatile float voltas_totais; //Mostrar no display
extern volatile uint32_t pwm; //Controle do pwm
extern volatile PIDController PID; // Parametros do PID
extern volatile float rpm_alvo; // ALVO
extern volatile float rpm_filtrado_pid;
extern volatile User_inputs parametros; // Param

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
 * @brief This function handles Non maskable interrupt.
 */
void NMI_Handler(void) {
	/* USER CODE BEGIN NonMaskableInt_IRQn 0 */

	/* USER CODE END NonMaskableInt_IRQn 0 */
	/* USER CODE BEGIN NonMaskableInt_IRQn 1 */
	while (1) {
	}
	/* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
 * @brief This function handles Hard fault interrupt.
 */
void HardFault_Handler(void) {
	/* USER CODE BEGIN HardFault_IRQn 0 */

	/* USER CODE END HardFault_IRQn 0 */
	while (1) {
		/* USER CODE BEGIN W1_HardFault_IRQn 0 */
		/* USER CODE END W1_HardFault_IRQn 0 */
	}
}

/**
 * @brief This function handles Memory management fault.
 */
void MemManage_Handler(void) {
	/* USER CODE BEGIN MemoryManagement_IRQn 0 */

	/* USER CODE END MemoryManagement_IRQn 0 */
	while (1) {
		/* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
		/* USER CODE END W1_MemoryManagement_IRQn 0 */
	}
}

/**
 * @brief This function handles Prefetch fault, memory access fault.
 */
void BusFault_Handler(void) {
	/* USER CODE BEGIN BusFault_IRQn 0 */

	/* USER CODE END BusFault_IRQn 0 */
	while (1) {
		/* USER CODE BEGIN W1_BusFault_IRQn 0 */
		/* USER CODE END W1_BusFault_IRQn 0 */
	}
}

/**
 * @brief This function handles Undefined instruction or illegal state.
 */
void UsageFault_Handler(void) {
	/* USER CODE BEGIN UsageFault_IRQn 0 */

	/* USER CODE END UsageFault_IRQn 0 */
	while (1) {
		/* USER CODE BEGIN W1_UsageFault_IRQn 0 */
		/* USER CODE END W1_UsageFault_IRQn 0 */
	}
}

/**
 * @brief This function handles System service call via SWI instruction.
 */
void SVC_Handler(void) {
	/* USER CODE BEGIN SVCall_IRQn 0 */

	/* USER CODE END SVCall_IRQn 0 */
	/* USER CODE BEGIN SVCall_IRQn 1 */

	/* USER CODE END SVCall_IRQn 1 */
}

/**
 * @brief This function handles Debug monitor.
 */
void DebugMon_Handler(void) {
	/* USER CODE BEGIN DebugMonitor_IRQn 0 */

	/* USER CODE END DebugMonitor_IRQn 0 */
	/* USER CODE BEGIN DebugMonitor_IRQn 1 */

	/* USER CODE END DebugMonitor_IRQn 1 */
}

/**
 * @brief This function handles Pendable request for system service.
 */
void PendSV_Handler(void) {
	/* USER CODE BEGIN PendSV_IRQn 0 */

	/* USER CODE END PendSV_IRQn 0 */
	/* USER CODE BEGIN PendSV_IRQn 1 */

	/* USER CODE END PendSV_IRQn 1 */
}

/**
 * @brief This function handles System tick timer.
 */
void SysTick_Handler(void) {
	/* USER CODE BEGIN SysTick_IRQn 0 */

	/* USER CODE END SysTick_IRQn 0 */
	HAL_IncTick();
	/* USER CODE BEGIN SysTick_IRQn 1 */

	/* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32G4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g4xx.s).                    */
/******************************************************************************/

/**
 * @brief This function handles EXTI line0 interrupt.
 */
void EXTI0_IRQHandler(void) {
	/* USER CODE BEGIN EXTI0_IRQn 0 */

	/* USER CODE END EXTI0_IRQn 0 */
	HAL_GPIO_EXTI_IRQHandler(BT_CONFIG_Pin);
	/* USER CODE BEGIN EXTI0_IRQn 1 */

	/* USER CODE END EXTI0_IRQn 1 */
}

/**
 * @brief This function handles EXTI line4 interrupt.
 */
void EXTI4_IRQHandler(void) {
	/* USER CODE BEGIN EXTI4_IRQn 0 */

	/* USER CODE END EXTI4_IRQn 0 */
	HAL_GPIO_EXTI_IRQHandler(BT_DEC_Pin);
	/* USER CODE BEGIN EXTI4_IRQn 1 */

	/* USER CODE END EXTI4_IRQn 1 */
}

/**
 * @brief This function handles EXTI line[9:5] interrupts.
 */
void EXTI9_5_IRQHandler(void) {
	/* USER CODE BEGIN EXTI9_5_IRQn 0 */

	/* USER CODE END EXTI9_5_IRQn 0 */
	HAL_GPIO_EXTI_IRQHandler(BT_SET_Pin);
	/* USER CODE BEGIN EXTI9_5_IRQn 1 */

	/* USER CODE END EXTI9_5_IRQn 1 */
}

/**
 * @brief This function handles TIM2 global interrupt.
 *        Often used here as an external trigger or safety limit switch (Emergency Stop).
 */
void TIM2_IRQHandler(void) {
	/* USER CODE BEGIN TIM2_IRQn 0 */

	// Check if Capture/Compare Channel 3 (CC3) triggered the interrupt
	if (TIM2->SR & TIM_SR_CC3IF) {
		// Clear the interrupt pending flag.
		// This is MANDATORY; otherwise, the MCU will re-enter the ISR infinitely.
		TIM2->SR &= ~TIM_SR_CC3IF;

		// Trigger the global emergency stop flag
		emergency_stop = 1;

		// --- Hardware-level fast shutdown (Commented out but kept for reference) ---
		// TIM_PWM->CCRx = 0;            // Force PWM duty cycle to 0 directly via register
		// TIM_PWM->CCER &= ~(TIM_CCER_CC1E << (canal * 4)); // Disable the PWM output channel directly
	}


	// Early return to bypass the HAL abstraction layer, reducing ISR latency
	return;

	/* USER CODE END TIM2_IRQn 0 */
	HAL_TIM_IRQHandler(&htim2);
	/* USER CODE BEGIN TIM2_IRQn 1 */

	/* USER CODE END TIM2_IRQn 1 */
}

/**
 * @brief This function handles TIM3 global interrupt.
 *        This acts as the main periodic Control Loop (e.g., running every 2ms).
 */
void TIM3_IRQHandler(void) {
	/* USER CODE BEGIN TIM3_IRQn 0 */

	// Check if the Update Interrupt Flag (UIF) is set (Timer period elapsed/overflow)
	if (TIM3->SR & TIM_SR_UIF) {
		// Fetch the latest target RPM.
		// (Non-static so it immediately reflects changes made in the main loop UI)
		float rpm_alvo = parametros.RPM;

		// Acknowledge the interrupt by clearing the UIF flag
		TIM3->SR &= ~TIM_SR_UIF;

		// Update raw encoder hardware readings
		update_encoder(&Encoder, &htim2);
		voltas_totais = Encoder.position / COUNTS_PER_REV;

		int32_t pos_atual = Encoder.position;

		// Calculate the exact number of pulses that occurred within this specific control period
		int32_t delta_pulsos = pos_atual - pos_antiga;

		// Save current position for the next ISR cycle
		pos_antiga = pos_atual;

		// --- Média móvel: tira o mais antigo da soma, entra o novo ---
		soma_delta_pulsos -= delta_buffer[buffer_index];
		soma_delta_pulsos += delta_pulsos;
		delta_buffer[buffer_index] = delta_pulsos;
		buffer_index = (buffer_index + 1) % WINDOW_SAMPLES;

		// RPM médio sobre a janela inteira (25 amostras = 50 ms)
		rpm_filtrado_pid = (soma_delta_pulsos / (float) COUNTS_PER_REV)
		                  * (60.0f / (WINDOW_SAMPLES * CONTROL_PERIOD_S));

//		// Calculate raw instantaneous RPM.
//		// Note: At low speeds, without high-resolution time-stamping, this calculation
//		// suffers from quantization error (causing jagged "jumps" in the reading).
//		float rpm_instantaneo = (delta_pulsos / (float) COUNTS_PER_REV)
//				* (60.0f / CONTROL_PERIOD_S);
//
//		// Apply Exponential Moving Average (EMA) Low-Pass Filter
//		// Formula: (New_Value * Alpha) + (Old_Value * (1 - Alpha))
//		// This smooths out the quantized instantaneous RPM for a stable PID feedback.
//		rpm_filtrado_pid = (ALPHA_FILTRO * rpm_instantaneo)
//				+ ((1.0f - ALPHA_FILTRO) * rpm_filtrado_pid);

		// --- DETERMINE ACTUAL TARGET ---
		// If emergency stop is triggered, override the user target and command 0 RPM.
		float alvo_atual = emergency_stop ? 0.0f : rpm_alvo;

		// --- RATE LIMITER (RAMP GENERATOR) ---
		// Prevents aggressive torque spikes by smoothly interpolating the setpoint over time.
		float periodo_ms = CONTROL_PERIOD_S * 1000.0f; // Control period in milliseconds (e.g., 2.0 ms)

		// Determine the reference max RPM to calculate the ramp slope.
		float max_rpm_referencia = rpm_alvo;
		if (max_rpm_referencia <= 0.0f)
			max_rpm_referencia = rpm_alvo; // Fallback to avoid div by zero/negative slope

		if (rpm_rampa < alvo_atual) {
			// ACCELERATION PHASE
			if (parametros.Rise_time > 0) {
				// Calculate how much RPM to add per control cycle
				float passo_subida = (max_rpm_referencia
						/ (float) parametros.Rise_time) * periodo_ms;
				rpm_rampa += passo_subida;

				// Clamp the ramped value to not overshoot the target
				if (rpm_rampa > alvo_atual)
					rpm_rampa = alvo_atual;
			} else {
				rpm_rampa = alvo_atual; // Step response (No ramp)
			}
		} else if (rpm_rampa > alvo_atual) {
			// DECELERATION PHASE (Uses standard Fall_time, even during emergency stops)
			if (parametros.Fall_time > 0) {
				// Calculate how much RPM to subtract per control cycle
				float passo_descida = (max_rpm_referencia
						/ (float) parametros.Fall_time) * periodo_ms;
				rpm_rampa -= passo_descida;

				// Clamp the ramped value to not undershoot the target
				if (rpm_rampa < alvo_atual)
					rpm_rampa = alvo_atual;
			} else {
				rpm_rampa = alvo_atual; // Step response (No ramp)
			}
		}

		// The PID controller runs continuously, tracking the dynamically generated ramp trajectory
		pwm = (uint32_t)PIDController_Update(&PID, rpm_rampa, rpm_filtrado_pid);

		// Apply the calculated PID control effort to the Timer 1 Capture/Compare Register (Duty Cycle)
		TIM1->CCR1 = pwm;

		// Early return to bypass HAL overhead
		return;
	}

	/* USER CODE END TIM3_IRQn 0 */
	HAL_TIM_IRQHandler(&htim3);
	/* USER CODE BEGIN TIM3_IRQn 1 */

	/* USER CODE END TIM3_IRQn 1 */
}
/**
 * @brief This function handles TIM6 global interrupt, DAC1 and DAC3 channel underrun error interrupts.
 */
void TIM6_DAC_IRQHandler(void) {
	/* USER CODE BEGIN TIM6_DAC_IRQn 0 */

	/* USER CODE END TIM6_DAC_IRQn 0 */
	HAL_TIM_IRQHandler(&htim6);
	/* USER CODE BEGIN TIM6_DAC_IRQn 1 */

	/* USER CODE END TIM6_DAC_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
