/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
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
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "PID.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define COUNTS_PER_REV    4000.0f     // 1000 linhas x4 (TIM_ENCODERMODE_TI12)
#define CONTROL_PERIOD_S  0.002f      // TIM3: 2 ms (CONTROL_PERIODO_US em main.c)
#define WINDOW_SAMPLES    25
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
volatile uint8_t emergency_stop = 0;

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

/* USER CODE BEGIN EV */
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

extern volatile encoder Encoder;
extern volatile float voltas_totais; //Mostrar no display
extern volatile uint32_t pwm; //Controle do pwm
extern volatile PIDController PID; // Parametros do PID
extern volatile float rpm_filtrado_pid;
extern volatile User_inputs parametros; // Param
extern volatile uint32_t voltas_tim;
extern int32_t pos_antiga;
extern float rpm_rampa; // Setpoint dinâmico (a rampa) que o PID vai perseguir
extern volatile uint8_t fall_end;
extern volatile uint8_t rise_ramp;
extern volatile Input_t input_prioritario;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/* USER CODE BEGIN 1 */

/* Os handlers abaixo ficam neste bloco de usuário porque o .ioc do F103 não
 * habilita TIM2/TIM3/EXTI15_10: assim a regeneração do CubeMX não os apaga.
 * Se habilitar essas interrupções no .ioc, apague estes handlers para não
 * duplicar os que o CubeMX passa a gerar. */

/**
 * @brief This function handles EXTI line[15:10] interrupts.
 *        Botões: PB12 (SEL), PB13 (CONFIG), PB14 (DEC), PB15 (STOP).
 */
void EXTI15_10_IRQHandler(void) {
	HAL_GPIO_EXTI_IRQHandler(BT_SEL_Pin);
	HAL_GPIO_EXTI_IRQHandler(BT_CONFIG_Pin);
	HAL_GPIO_EXTI_IRQHandler(BT_DEC_Pin);
	HAL_GPIO_EXTI_IRQHandler(BT_STOP_Pin);
}

/**
 * @brief This function handles TIM2 global interrupt.
 *        Canal 3 em output compare: fim da contagem de voltas (Modo 2).
 *
 * No F103 o TIM2 tem 16 bits, então o CCR3 só guarda os 16 bits baixos do
 * alvo (Target + Offset) * 4000, e o casamento acontece a cada 65536
 * contagens (~16,4 voltas). Para só disparar na volta certa, a posição
 * completa (32 bits) é reconstruída aqui e comparada com o alvo.
 */
void TIM2_IRQHandler(void) {

	// Check if Capture/Compare Channel 3 (CC3) triggered the interrupt
	if (TIM2->SR & TIM_SR_CC3IF) {
		// Clear the interrupt pending flag (MANDATORY, senão reentra na ISR)
		TIM2->SR &= ~TIM_SR_CC3IF;

		// Alvo completo em contagens (mesma conta do ATIV_INIT)
		int32_t alvo = (int32_t) ((parametros.Target_Counts
				+ parametros.Offset_Counts) * COUNTS_PER_REV + 0.5f);

		// Posição exata agora: pos_antiga (atualizada pelo TIM3 a cada 2 ms)
		// mais o quanto o contador de 16 bits andou desde então.
		int16_t d16 = (int16_t) ((uint16_t) TIM2->CNT - (uint16_t) pos_antiga);
		int32_t pos_agora = pos_antiga + d16;

		// Casamento "na volta certa": diferença próxima de 0 (ou maior).
		// Casamentos das voltas anteriores dão diferença <= -65536 + folga.
		if ((pos_agora - alvo) > -32768) {
			// Trigger the global stop flag (a ISR do TIM3 faz a rampa de descida)
			emergency_stop = 1;
		}
		return; // Early return to bypass the HAL abstraction layer
	}

	HAL_TIM_IRQHandler(&htim2);
}

/**
 * @brief This function handles TIM3 global interrupt.
 *        This acts as the main periodic Control Loop (running every 2ms).
 */
void TIM3_IRQHandler(void) {

	// Check if the Update Interrupt Flag (UIF) is set (Timer period elapsed/overflow)
	if (TIM3->SR & TIM_SR_UIF) {
		// Fetch the latest target RPM.
		// (Non-static so it immediately reflects changes made in the main loop UI)
		float rpm_alvo = parametros.RPM;

		// Acknowledge the interrupt by clearing the UIF flag
		TIM3->SR &= ~TIM_SR_UIF;

		// --- Posição de 32 bits a partir do TIM2 de 16 bits -----------------
		// Invariante: o ATIV_INIT zera TIM2->CNT e pos_antiga juntos, logo os
		// 16 bits baixos de pos_antiga sempre coincidem com o CNT anterior.
		// A diferença em int16_t é exata enquanto o motor andar menos de 32768
		// contagens em 2 ms (muito acima do que o motor faz).
		uint16_t cnt_atual = (uint16_t) TIM2->CNT;
		voltas_tim = cnt_atual;
		int16_t delta16 = (int16_t) (cnt_atual - (uint16_t) pos_antiga);
		int32_t pos_atual = pos_antiga + delta16;

		Encoder.position = pos_atual;
		voltas_totais = pos_atual / COUNTS_PER_REV;

		// Calculate the exact number of pulses that occurred within this specific control period
		int32_t delta_pulsos = delta16;

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
		if(rpm_filtrado_pid < 0) rpm_filtrado_pid =0;

		// --- DETERMINE ACTUAL TARGET ---
		// If the stop flag is set, override the user target and command 0 RPM.
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
			if (parametros.Rise_time > 0 && rise_ramp) {

				// Calculate how much RPM to add per control cycle
				float passo_subida = (max_rpm_referencia
						/ (float) parametros.Rise_time) * periodo_ms;
				rpm_rampa += passo_subida;

				// Clamp the ramped value to not overshoot the target
				// FIXME (herdado do G4, mantido de propósito na portagem):
				// faltam chaves no if abaixo, então 'rpm_rampa = alvo_atual'
				// executa SEMPRE e a rampa de subida vira degrau após o 1º
				// passo. Correção: if (rpm_rampa > alvo_atual) { rise_ramp = 0;
				// rpm_rampa = alvo_atual; }
				if (rpm_rampa > alvo_atual)
					rise_ramp = 0;
					rpm_rampa = alvo_atual;
			} else {
				rpm_rampa = alvo_atual; // Step response (No ramp)
			}
		} else if (rpm_rampa > alvo_atual && emergency_stop) {
			// DECELERATION PHASE (Uses standard Fall_time)
			// FIXME (herdado do G4): só desce com emergency_stop; no Modo 3, ao
			// diminuir o RPM com '-', rpm_rampa não acompanha o alvo para baixo.
			if (parametros.Fall_time > 0) {
				// Calculate how much RPM to subtract per control cycle
				float passo_descida = (max_rpm_referencia
						/ (float) parametros.Fall_time) * periodo_ms;
				rpm_rampa -= passo_descida;

				// Clamp the ramped value to not undershoot the target
				if (rpm_rampa < alvo_atual){
					rpm_rampa = alvo_atual;
					fall_end =1;
					input_prioritario = INPUT_RETURN;
				}
			} else {
				rpm_rampa = alvo_atual; // Step response (No ramp)
				fall_end =1;
				input_prioritario = INPUT_RETURN;
			}
		}

		// The PID controller runs continuously, tracking the dynamically generated ramp trajectory
		pwm = (uint32_t)PIDController_Update(&PID,rpm_rampa , rpm_filtrado_pid);

		// Apply the calculated PID control effort to the Timer 1 Capture/Compare Register (Duty Cycle)
		TIM1->CCR1 = pwm;

		// Early return to bypass HAL overhead
		return;
	}

	HAL_TIM_IRQHandler(&htim3);
}

/* USER CODE END 1 */
