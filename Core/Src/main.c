/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor_encoder.h"
#include "LCD1602.h"
#include "string.h"
#include "stdio.h"
#include "stdint.h"
#include "stdbool.h"
#include "stdlib.h"
#include "PID.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
	Start,
	Config_Cycles,
	Config_RPM,
	Config_Rising,
	Config_Falling,
	Config_Off_Cycles,
	PID_Calibration,
	Set_Defaults,
	Running,
	Initializing,
	Stopping
} DisplayState;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define COUNTS_PER_REV    4000.0f     // 600 linhas x4 (TIM_ENCODERMODE_TI12)
#define TARGET_REVS       10
#define TARGET_COUNT      ((uint32_t)(COUNTS_PER_REV * TARGET_REVS))  // 2.400.000
#define CONTROL_PERIOD_S 0.002f

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
volatile encoder Encoder;
volatile float voltas_totais = 0;
float media_rpm = 0;
volatile uint32_t pwm;
// PID
volatile PIDController PID;

volatile float rpm_filtrado_pid;

volatile uint8_t Flag_display = 0;
int increment[8] = { 1, 10, 100, 1000, -1, -10, -100, -1000 };
volatile uint8_t index_inc = 0;

static DisplayState state = Start; // Modes

/* --- Variáveis de Estado --- */
volatile User_inputs parametros;




volatile bool lcd_needs_update = true; // Começa como true para desenhar a primeira vez


extern volatile uint8_t emergency_stop;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	reset_encoder(&Encoder);
	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_USART1_UART_Init();
	MX_TIM1_Init();
	MX_TIM4_Init();
	MX_TIM6_Init();
	/* USER CODE BEGIN 2 */

	// 2º Inicializa a Flash e carrega os parâmetros salvos
	//  Storage_Init();

	HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);

	HAL_TIM_Base_Start(&htim4);
	lcd_init();
	lcd_put_cur(0, 0);
	lcd_send_string("Iniciado ");
	lcd_put_cur(1, 0);
	char buffer[16];

#define NUM_LEITURAS 100
	float historico_rpm[NUM_LEITURAS] = { 0 }; // Array preenchido com zeros
	float soma_rpm = 0.0f;
	uint8_t indice = 0;

	parametros.Fall_time = 2000;
	parametros.Offset_Counts = 5;
	parametros.RPM = 20;
	parametros.Rise_time = 1000;
	parametros.Target_Counts = 5000;

	// 1. Configuração dos Ganhos do Controlador (Estes valores precisarão ser ajustados na prática)
	PID.Kp = 16.0f;
	PID.Ki = 3.0f;
	PID.Kd = 0.0f;

	// 2. Constante de tempo do filtro passa-baixa da Derivada
	PID.tau = 0.01f;

	// 3. Tempo de Amostragem (Período)
	// Como o TIM3 roda a 1.000 Hz, o tempo entre as amostras é de 1 milissegundo.
	PID.T = 0.002f;

	// 4. Limites da Saída do Controlador (Sinal de Controle -> PWM)
	// Conforme configuramos, o TIM1 CH1 vai de 0 (parado) a 4800 (velocidade máxima).
	PID.limMin = 0.0f;
	PID.limMax = 100.0f;

	// 5. Limites do Integrador (Anti-Windup)
	// Impede que o erro integral cresça infinitamente caso o motor trave.
	PID.limMinInt = 0.0f;
	PID.limMaxInt = 100.0f;

	/* USER CODE END 2 */

	/* Initialize leds */
	BSP_LED_Init(LED_GREEN);

	/* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */
	BspCOMInit.BaudRate = 115200;
	BspCOMInit.WordLength = COM_WORDLENGTH_8B;
	BspCOMInit.StopBits = COM_STOPBITS_1;
	BspCOMInit.Parity = COM_PARITY_NONE;
	BspCOMInit.HwFlowCtl = COM_HWCONTROL_NONE;
	if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE) {
		Error_Handler();
	}

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	TIM1->CCR1 = 0;

	/* USER CODE BEGIN WHILE */

	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		/* ----------------------------------------------------------------------
		 * MOVING AVERAGE FILTER FOR RPM CALCULATION
		 * ---------------------------------------------------------------------- */
		// Note: The 'historico_rpm', 'soma_rpm', and 'indice' variables should be
		// declared as static or global before the while loop.
		// Convert raw encoder velocity (counts per control period) into RPM
		float rpm_atual = (Encoder.velocity / COUNTS_PER_REV)
				* (60.0f / CONTROL_PERIOD_S);

		// Update the running sum and the history buffer
		soma_rpm -= historico_rpm[indice]; // Remove the oldest reading from the running sum
		historico_rpm[indice] = rpm_atual; // Store the newest calculation in the buffer
		soma_rpm += historico_rpm[indice]; // Add the newest reading to the running sum

		// Calculate the filtered average RPM over NUM_LEITURAS (Number of readings)
		media_rpm = soma_rpm / NUM_LEITURAS;

		// Advance the circular buffer pointer
		indice++;
		if (indice >= NUM_LEITURAS) {
			indice = 0; // Wrap around to the beginning of the array
		}



		/* ----------------------------------------------------------------------
		 * SYSTEM STATE MACHINE & UI HANDLER
		 * ---------------------------------------------------------------------- */
		switch (state) {

		case Start: {
			static uint32_t ultimo_tempo_lcd = 0;
			static uint8_t tela_atual = 0;

			// Non-blocking delay: Update the LCD carousel every 1.5 seconds (1500 ms)
			if (HAL_GetTick() - ultimo_tempo_lcd > 1500) {
				ultimo_tempo_lcd = HAL_GetTick(); // Update timestamp

				lcd_put_cur(0, 0);
				lcd_send_string("Aperte SET:     "); // Prompt user. Trailing spaces clear previous display artifacts

				lcd_put_cur(1, 0);

				// Carousel logic: Cycle through system parameters to display on the second line
				switch (tela_atual) {
				case 0:
					sprintf(buffer, "Target: %d      ",
							(int)parametros.Target_Counts);
					break;
				case 1:
					sprintf(buffer, "RPM: %d         ", (int) parametros.RPM);
					break;
				case 2:
					sprintf(buffer, "Rise: %d        ",(int) parametros.Rise_time);
					break;
				case 3:
					sprintf(buffer, "Fall: %d        ",(int) parametros.Fall_time);
					break;
				case 4:
					sprintf(buffer, "Offset: %d      ",
							(int)parametros.Offset_Counts);
					break;
				}
				lcd_send_string(buffer);

				// Advance the carousel screen index, wrapping around after screen 4
				tela_atual++;
				if (tela_atual > 4)
					tela_atual = 0;
			}
			break;
		}

		case Config_Cycles: {
			// Event-driven update: Only redraw the LCD if a button interrupt flagged a change
			if (lcd_needs_update) {
				lcd_put_cur(0, 0);
				char sin = (increment[index_inc] > 0 ? '+' : '-');

				// First line: Parameter name, sign, and current increment magnitude
				snprintf(buffer, sizeof(buffer), "Target:%c%-6d", sin,
						abs(increment[index_inc]));
				lcd_send_string(buffer);

				// Second line: Current parameter value
				sprintf(buffer, "%d counts      ", (int)parametros.Target_Counts);
				lcd_put_cur(1, 0);
				lcd_send_string(buffer);

				lcd_needs_update = false; // Clear flag to prevent redundant I2C/SPI transmissions
			}
			break;
		}

		case Config_Rising: {
			if (lcd_needs_update) {
				lcd_put_cur(0, 0);
				char sin = (increment[index_inc] > 0 ? '+' : '-');
				snprintf(buffer, sizeof(buffer), "Rising:%c%-6d", sin,
						abs(increment[index_inc]));
				lcd_send_string(buffer);

				sprintf(buffer, "%d ms          ", (int)parametros.Rise_time);
				lcd_put_cur(1, 0);
				lcd_send_string(buffer);

				lcd_needs_update = false;
			}
			break;
		}

		case Config_Falling: {
			if (lcd_needs_update) {
				lcd_put_cur(0, 0);
				char sin = (increment[index_inc] > 0 ? '+' : '-');
				snprintf(buffer, sizeof(buffer), "Falling:%c%-6d", sin,
						abs(increment[index_inc]));
				lcd_send_string(buffer);

				sprintf(buffer, "%d ms          ",(int) parametros.Fall_time);
				lcd_put_cur(1, 0);
				lcd_send_string(buffer);

				lcd_needs_update = false;
			}
			break;
		}

		case Config_Off_Cycles: {
			if (lcd_needs_update) {
				lcd_put_cur(0, 0);
				char sin = (increment[index_inc] > 0 ? '+' : '-');
				snprintf(buffer, sizeof(buffer), "Offset:%c%-6d", sin,
						abs(increment[index_inc]));
				lcd_send_string(buffer);

				sprintf(buffer, "%d counts      ", (int)parametros.Offset_Counts);
				lcd_put_cur(1, 0);
				lcd_send_string(buffer);

				lcd_needs_update = false;
			}
			break;
		}

		case Running: {

//			if (emergency_stop) {
//				state = Stopping;
//			}
			static uint32_t ultimo_tempo_animacao = 0;
			static uint8_t frame_animacao = 0;
			char *pontos;

			// Verifica se já se passaram 250 milissegundos desde a última atualização
			if (HAL_GetTick() - ultimo_tempo_animacao >= 400) {
			    ultimo_tempo_animacao = HAL_GetTick(); // Reinicia o cronômetro

			// Define a posição da "bolinha" pulando
			switch(frame_animacao) {
			    case 0: pontos = "o.."; break; // Posição 1
			    case 1: pontos = ".o."; break; // Posição 2
			    case 2: pontos = "..o"; break; // Posição 3
			    case 3: pontos = ".o."; break; // Volta para a Posição 2
			}
			// Active execution state. Continuously overwrites display with live data
			lcd_put_cur(0, 0);
			sprintf(buffer, "On%s  |Cnt: %d ",pontos, (int)voltas_totais);
			lcd_send_string(buffer); // "Running..."

			// Avança para o próximo quadro (0, 1, 2, 3 e depois volta para 0)
			frame_animacao = (frame_animacao + 1) % 4;

			// Display live filtered RPM calculated at the top of the loop
			sprintf(buffer, "RPM: %d |Set:%d", (int) media_rpm, (int)parametros.Target_Counts);
			lcd_put_cur(1, 0);
			lcd_send_string(buffer);
			}
			// Ensure flag is cleared if we just transitioned from a config state
			lcd_needs_update = false;

			break;
		}

		case Initializing: {


			PIDController_Init(&PID);

			/* --- Peripheral Initialization Phase --- */

			// Start Timer 3 in interrupt mode (Likely used for the main control loop / PID execution time base)
			HAL_TIM_Base_Start_IT(&htim3);

			// Start Timer 2 Output Compare in interrupt mode (Often used to trigger precise velocity measurements)
			HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_3);

			// Start Timer 2 in Encoder interface mode to track quadrature encoder pulses
			HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

			// Initialize Motor PWM duty cycle (Capture/Compare Register 1) to 0% (stopped)
			TIM1->CCR1 = 0;

			// Start PWM signal generation on Timer 1 Channel 1
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

			state = Running;

			break;
		}
		case Stopping: {
			TIM1->CCR1 = 0;
			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
			HAL_TIM_Base_Stop_IT(&htim3);
			HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_3);
			HAL_TIM_Encoder_Stop(&htim2, TIM_CHANNEL_ALL);

			state = Start;
			emergency_stop = 0;
			break;
		}

		default:
			break;
		}

	}
}
	/* USER CODE END 3 */

	/**
	 * @brief System Clock Configuration
	 * @retval None
	 */
	void SystemClock_Config(void) {
		RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
		RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

		/** Configure the main internal regulator output voltage
		 */
		HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

		/** Initializes the RCC Oscillators according to the specified parameters
		 * in the RCC_OscInitTypeDef structure.
		 */
		RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
		RCC_OscInitStruct.HSIState = RCC_HSI_ON;
		RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
		RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
		RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
		RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
		RCC_OscInitStruct.PLL.PLLN = 12;
		RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
		RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
		RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
		if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
			Error_Handler();
		}

		/** Initializes the CPU, AHB and APB buses clocks
		 */
		RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
				| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
		RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
		RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
		RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
		RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

		if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3)
				!= HAL_OK) {
			Error_Handler();
		}
	}

	/**
	 * @brief TIM1 Initialization Function
	 * @param None
	 * @retval None
	 */
	static void MX_TIM1_Init(void) {

		/* USER CODE BEGIN TIM1_Init 0 */

		/* USER CODE END TIM1_Init 0 */

		TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
		TIM_MasterConfigTypeDef sMasterConfig = { 0 };
		TIM_OC_InitTypeDef sConfigOC = { 0 };
		TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

		/* USER CODE BEGIN TIM1_Init 1 */

		/* USER CODE END TIM1_Init 1 */
		htim1.Instance = TIM1;
		htim1.Init.Prescaler = 4799;
		htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
		htim1.Init.Period = 99;
		htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
		htim1.Init.RepetitionCounter = 0;
		htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
		if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
			Error_Handler();
		}
		sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
		if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
			Error_Handler();
		}
		if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
			Error_Handler();
		}
		sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
		sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
		sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
		if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
				!= HAL_OK) {
			Error_Handler();
		}
		sConfigOC.OCMode = TIM_OCMODE_PWM1;
		sConfigOC.Pulse = 0;
		sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
		sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
		sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
		sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
		sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
		if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1)
				!= HAL_OK) {
			Error_Handler();
		}
		sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
		sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
		sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
		sBreakDeadTimeConfig.DeadTime = 0;
		sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
		sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
		sBreakDeadTimeConfig.BreakFilter = 0;
		sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
		sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
		sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
		sBreakDeadTimeConfig.Break2Filter = 0;
		sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
		sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
		if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig)
				!= HAL_OK) {
			Error_Handler();
		}
		/* USER CODE BEGIN TIM1_Init 2 */

		/* USER CODE END TIM1_Init 2 */
		HAL_TIM_MspPostInit(&htim1);

	}

	/**
	 * @brief TIM2 Initialization Function
	 * @param None
	 * @retval None
	 */
	static void MX_TIM2_Init(void) {

		/* USER CODE BEGIN TIM2_Init 0 */

		/* USER CODE END TIM2_Init 0 */

		TIM_Encoder_InitTypeDef sConfig = { 0 };
		TIM_MasterConfigTypeDef sMasterConfig = { 0 };
		TIM_OC_InitTypeDef sConfigOC = { 0 };

		/* USER CODE BEGIN TIM2_Init 1 */

		/* USER CODE END TIM2_Init 1 */
		htim2.Instance = TIM2;
		htim2.Init.Prescaler = 0;
		htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
		htim2.Init.Period = 4294967295;
		htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
		htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
		if (HAL_TIM_OC_Init(&htim2) != HAL_OK) {
			Error_Handler();
		}
		sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
		sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
		sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
		sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
		sConfig.IC1Filter = 5;
		sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
		sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
		sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
		sConfig.IC2Filter = 5;
		if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK) {
			Error_Handler();
		}
		sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
		sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
		if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
				!= HAL_OK) {
			Error_Handler();
		}
		sConfigOC.OCMode = TIM_OCMODE_TIMING;
		sConfigOC.Pulse = 0;
		sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
		sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
		if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3)
				!= HAL_OK) {
			Error_Handler();
		}
		/* USER CODE BEGIN TIM2_Init 2 */
		TIM2->CCR3 = parametros.Target_Counts;

		/* USER CODE END TIM2_Init 2 */

	}

	/**
	 * @brief TIM3 Initialization Function
	 * @param None
	 * @retval None
	 */
	static void MX_TIM3_Init(void) {

		/* USER CODE BEGIN TIM3_Init 0 */

		/* USER CODE END TIM3_Init 0 */

		TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
		TIM_MasterConfigTypeDef sMasterConfig = { 0 };

		/* USER CODE BEGIN TIM3_Init 1 */

		/* USER CODE END TIM3_Init 1 */
		htim3.Instance = TIM3;
		htim3.Init.Prescaler = 95;
		htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
		htim3.Init.Period = 1999;
		htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
		htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
		if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
			Error_Handler();
		}
		sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
		if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
			Error_Handler();
		}
		sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
		sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
		if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
				!= HAL_OK) {
			Error_Handler();
		}
		/* USER CODE BEGIN TIM3_Init 2 */

		/* USER CODE END TIM3_Init 2 */

	}

	/**
	 * @brief TIM4 Initialization Function
	 * @param None
	 * @retval None
	 */
	static void MX_TIM4_Init(void) {

		/* USER CODE BEGIN TIM4_Init 0 */

		/* USER CODE END TIM4_Init 0 */

		TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
		TIM_MasterConfigTypeDef sMasterConfig = { 0 };

		/* USER CODE BEGIN TIM4_Init 1 */

		/* USER CODE END TIM4_Init 1 */
		htim4.Instance = TIM4;
		htim4.Init.Prescaler = 0;
		htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
		htim4.Init.Period = 65535;
		htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
		htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
		if (HAL_TIM_Base_Init(&htim4) != HAL_OK) {
			Error_Handler();
		}
		sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
		if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK) {
			Error_Handler();
		}
		sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
		sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
		if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig)
				!= HAL_OK) {
			Error_Handler();
		}
		/* USER CODE BEGIN TIM4_Init 2 */

		/* USER CODE END TIM4_Init 2 */

	}

	/**
	 * @brief TIM6 Initialization Function
	 * @param None
	 * @retval None
	 */
	static void MX_TIM6_Init(void) {

		/* USER CODE BEGIN TIM6_Init 0 */

		/* USER CODE END TIM6_Init 0 */

		TIM_MasterConfigTypeDef sMasterConfig = { 0 };

		/* USER CODE BEGIN TIM6_Init 1 */

		/* USER CODE END TIM6_Init 1 */
		htim6.Instance = TIM6;
		htim6.Init.Prescaler = 9599;
		htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
		htim6.Init.Period = 14999;
		htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
		if (HAL_TIM_Base_Init(&htim6) != HAL_OK) {
			Error_Handler();
		}
		sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
		sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
		if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig)
				!= HAL_OK) {
			Error_Handler();
		}
		/* USER CODE BEGIN TIM6_Init 2 */

		/* USER CODE END TIM6_Init 2 */

	}

	/**
	 * @brief USART1 Initialization Function
	 * @param None
	 * @retval None
	 */
	static void MX_USART1_UART_Init(void) {

		/* USER CODE BEGIN USART1_Init 0 */

		/* USER CODE END USART1_Init 0 */

		/* USER CODE BEGIN USART1_Init 1 */

		/* USER CODE END USART1_Init 1 */
		huart1.Instance = USART1;
		huart1.Init.BaudRate = 9600;
		huart1.Init.WordLength = UART_WORDLENGTH_8B;
		huart1.Init.StopBits = UART_STOPBITS_1;
		huart1.Init.Parity = UART_PARITY_NONE;
		huart1.Init.Mode = UART_MODE_TX_RX;
		huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
		huart1.Init.OverSampling = UART_OVERSAMPLING_16;
		huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
		huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
		huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
		if (HAL_UART_Init(&huart1) != HAL_OK) {
			Error_Handler();
		}
		if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8)
				!= HAL_OK) {
			Error_Handler();
		}
		if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8)
				!= HAL_OK) {
			Error_Handler();
		}
		if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK) {
			Error_Handler();
		}
		/* USER CODE BEGIN USART1_Init 2 */

		/* USER CODE END USART1_Init 2 */

	}

	/**
	 * @brief GPIO Initialization Function
	 * @param None
	 * @retval None
	 */
	static void MX_GPIO_Init(void) {
		GPIO_InitTypeDef GPIO_InitStruct = { 0 };
		/* USER CODE BEGIN MX_GPIO_Init_1 */

		/* USER CODE END MX_GPIO_Init_1 */

		/* GPIO Ports Clock Enable */
		__HAL_RCC_GPIOA_CLK_ENABLE();
		__HAL_RCC_GPIOB_CLK_ENABLE();

		/*Configure GPIO pin Output Level */
		HAL_GPIO_WritePin(GPIOA, D4_Pin | D5_Pin | D7_Pin | D6_Pin,
				GPIO_PIN_RESET);

		/*Configure GPIO pin Output Level */
		HAL_GPIO_WritePin(GPIOB, EN_Pin | RW_Pin | RS_Pin, GPIO_PIN_RESET);

		/*Configure GPIO pins : D4_Pin D5_Pin D7_Pin D6_Pin */
		GPIO_InitStruct.Pin = D4_Pin | D5_Pin | D7_Pin | D6_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		/*Configure GPIO pin : BT_SET_Pin */
		GPIO_InitStruct.Pin = BT_SET_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		HAL_GPIO_Init(BT_SET_GPIO_Port, &GPIO_InitStruct);

		/*Configure GPIO pin : BT_CONFIG_Pin */
		GPIO_InitStruct.Pin = BT_CONFIG_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		HAL_GPIO_Init(BT_CONFIG_GPIO_Port, &GPIO_InitStruct);

		/*Configure GPIO pin : BT_DEC_Pin */
		GPIO_InitStruct.Pin = BT_DEC_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		HAL_GPIO_Init(BT_DEC_GPIO_Port, &GPIO_InitStruct);

		/*Configure GPIO pins : EN_Pin RW_Pin RS_Pin */
		GPIO_InitStruct.Pin = EN_Pin | RW_Pin | RS_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

		/* EXTI interrupt init*/
		HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(EXTI0_IRQn);

		HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(EXTI4_IRQn);

		HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

		/* USER CODE BEGIN MX_GPIO_Init_2 */

		/* USER CODE END MX_GPIO_Init_2 */
	}

	/* USER CODE BEGIN 4 */

	/**
	 * @brief Timer Period Elapsed Callback. Handles the timeout/long-press logic for TIM6.
	 * @param htim: Pointer to a TIM_HandleTypeDef structure that contains
	 *              the configuration information for the specified TIM module.
	 * @retval None
	 */
	void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

		// Check if the interrupt was triggered by Timer 6 (used for long-press detection)
		if (htim->Instance == TIM6) {

			// Stop the timer so it acts as a one-shot event rather than continuously firing
			HAL_TIM_Base_Stop_IT(&htim6);

			// Circularly advance the multiplier index.
			// Keeps the value bounded between 0 and 7 (assuming an array of 8 increment sizes).
			index_inc = (index_inc + 1) % 8;

			// Flag the main loop to refresh the display, updating the UI with the new multiplier
			lcd_needs_update = true;

			// --- Reset the EXTI Line 4 (Button) Edge Triggers ---
			// By doing this here, the long-press forces the button state machine back
			// to its default state (waiting for a new press), effectively ignoring the physical release.

			EXTI->RTSR1 &= ~EXTI_FTSR1_FT4; // Disable rising edge detection on Line 4 (abort release detection)
			EXTI->FTSR1 |= EXTI_RTSR1_RT4; // Enable falling edge detection on Line 4 (ready for next press)
		}
	}

	/**
	 * @brief GPIO Interrupt Function. Handles button presses for Config, Decrement, and Set.
	 * @param GPIO_Pin: Specifies the pins connected to the EXTI line.
	 * @retval None
	 */
	void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
		/* ----------------------------------------------------------------------
		 * CONFIGURATION BUTTON HANDLING
		 * ---------------------------------------------------------------------- */
		if (GPIO_Pin == BT_CONFIG_Pin) {
			static uint32_t ultimo_aperto_conf = 0;

			// Software debounce: ensure at least 200ms have passed since the last valid press
			if (HAL_GetTick() - ultimo_aperto_conf > 200) {
				ultimo_aperto_conf = HAL_GetTick(); // Update the debounce timestamp

				// Ignore configuration inputs if the system is currently executing (Running state)
				if (state == Running)
					return;

				// Flag to trigger the main loop to refresh the display
				lcd_needs_update = true;

				// State machine transition: cycle through the configuration menus
				if (state == Config_Off_Cycles) {
					state = Start; // Wrap around to the initial state if at the end of the menu
					return;
				}
				state += 1; // Advance to the next configuration state
			}

			/* ----------------------------------------------------------------------
			 * DECREMENT / MULTIPLIER BUTTON HANDLING
			 * ---------------------------------------------------------------------- */
		} else if (GPIO_Pin == BT_DEC_Pin) {
			if (state == Running)
							return;
			static uint32_t last_press_dec = 0;

			// Software debounce: 50ms threshold for the decrement button
			if (HAL_GetTick() - last_press_dec > 50) {
				last_press_dec = HAL_GetTick(); // Update the debounce timestamp

				lcd_needs_update = true;

				// Direct register read: Check if Pin 4 on the port is LOW (Button Pressed - Active Low)
				if (!(BT_DEC_GPIO_Port->IDR & GPIO_IDR_ID4)) {

					// Button was PRESSED
					// Dynamically reconfigure EXTI triggers to catch the release (Rising Edge)
					EXTI->FTSR1 &= ~EXTI_FTSR1_FT4; // Disable falling edge detection on Line 4
					EXTI->RTSR1 |= EXTI_RTSR1_RT4; // Enable rising edge detection on Line 4

					// Reset and start Timer 6 (Likely used for long-press detection or dynamic increment speed)
					__HAL_TIM_SET_COUNTER(&htim6, 0);
					HAL_TIM_Base_Start_IT(&htim6);
				} else {

					// Button was RELEASED
					HAL_TIM_Base_Stop_IT(&htim6); // Stop the long-press timer

					// Restore EXTI triggers to catch the next press (Falling Edge)
					EXTI->RTSR1 &= ~EXTI_FTSR1_FT4; // Disable rising edge detection on Line 4
					EXTI->FTSR1 |= EXTI_RTSR1_RT4; // Enable falling edge detection on Line 4

					// --- APPLY MULTIPLIER LOGIC ---
					// Add the selected increment value to the corresponding parameter based on the current state
					if (state == Config_Cycles) {
						parametros.Target_Counts += increment[index_inc];
					} else if (state == Config_Rising) {
						parametros.Rise_time += increment[index_inc];
					} else if (state == Config_Falling) {
						parametros.Fall_time += increment[index_inc];
					} else if (state == Config_Off_Cycles) {
						parametros.Offset_Counts += increment[index_inc];
					}
				}
			}

			/* ----------------------------------------------------------------------
			 * SET / START-STOP BUTTON HANDLING
			 * ---------------------------------------------------------------------- */
		} else if (GPIO_Pin == BT_SET_Pin) {
			static uint32_t last_press_set = 0;

			// Software debounce: 50ms threshold for the SET button
			if (HAL_GetTick() - last_press_set > 50) {
				last_press_set = HAL_GetTick(); // Update the debounce timestamp

				// Toggle system state between Start (Idle/Ready) and Running (Active)
				if (state == Start) {
					state = Initializing;
				} else if (state == Running) {
					state = Start;
				}

				// Flag the LCD to update its UI with the new execution state
				lcd_needs_update = true;
			}
		}
	}



	void SendChar(char c) {
		while (!(USART3->ISR & USART_ISR_TXE_TXFNF)) {
		}
		USART3->TDR = c;
	}
	/* USER CODE END 4 */

	/* USER CODE BEGIN Header */
	/**
	 ******************************************************************************
	 * @file           : main.c
	 * @brief          : Main program body
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

	/**
	 * @}
	 */

	/**
	 * @}
	 */

	/**
	 * @brief  This function is executed in case of error occurrence.
	 * @retval None
	 */
	void Error_Handler(void) {
		/* USER CODE BEGIN Error_Handler_Debug */
		/* User can add his own implementation to report the HAL error return state */
		__disable_irq();
		while (1) {
		}
		/* USER CODE END Error_Handler_Debug */
	}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
