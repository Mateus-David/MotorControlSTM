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

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* --- Base de tempo (ver SystemClock_Config) ---------------------------------
 * SYSCLK = 48 MHz (HSI/2 x12). APB1 /2 -> PCLK1 = 24 MHz; como o prescaler do
 * APB1 é diferente de 1, TIM2/3/4 recebem 2 x PCLK1 = 48 MHz. TIM1 (APB2 /1)
 * também recebe 48 MHz. Se mudar o clock, só ajuste TIM_CLK_HZ: os prescalers
 * abaixo são recalculados (e os _Static_assert avisam se a divisão não fechar). */
#define TIM_CLK_HZ          48000000UL

/* PWM do motor (TIM1 CH1): 20 kHz com 600 passos. 600 é o mesmo fundo de escala
 * do PID (limMax), então os ganhos que você sintonizou no G4 continuam válidos. */
#define PWM_FREQ_HZ         20000UL
#define PWM_PASSOS          600UL
#define TIM1_PRESCALER      ((TIM_CLK_HZ / (PWM_FREQ_HZ * PWM_PASSOS)) - 1UL)

/* Malha de controle (TIM3): contador a 1 MHz, estouro a cada 2 ms (500 Hz).
 * Precisa bater com CONTROL_PERIOD_S em stm32f1xx_it.c e com PID.T abaixo. */
#define CONTROL_PERIODO_US  2000UL
#define TIM3_PRESCALER      ((TIM_CLK_HZ / 1000000UL) - 1UL)

/* TIM4: contador livre usado pela biblioteca do LCD (no G4: prescaler 0).
 * Confira em LCD1602.c que o atraso em µs não assume um clock fixo de 96 MHz. */
#define TIM4_PRESCALER      0UL

/* Buffer de uma linha do LCD 16x2: 16 colunas + '\0'. */
#define LCD_LINHA_BUF       17u

_Static_assert((TIM_CLK_HZ % (PWM_FREQ_HZ * PWM_PASSOS)) == 0,
		"TIM_CLK_HZ nao e multiplo de PWM_FREQ_HZ * PWM_PASSOS");
_Static_assert((TIM_CLK_HZ % 1000000UL) == 0,
		"TIM_CLK_HZ nao e multiplo de 1 MHz");

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

volatile encoder Encoder;
volatile float voltas_totais = 0;
float media_rpm = 0;
volatile uint32_t pwm;

//Máquina de estados
volatile TelaConfig_t tela_conf_global = TELA_INICIAR;
volatile EstadoAtiv_t estado_ativ_global = ATIV_INIT;

//Estado de subida ou descida
volatile uint8_t rise_ramp = 0;

//debug
volatile uint32_t voltas_tim;
// PID
volatile PIDController PID;
int32_t pos_antiga = 0;
float rpm_rampa = 0.0f; // Setpoint dinâmico (a rampa) que o PID vai perseguir
volatile uint8_t fall_end = 0;

volatile float rpm_filtrado_pid;

/* Legado do G4 (sem uso na máquina atual; apague se nada mais referenciar). */
volatile uint8_t Flag_display = 0;
int increment[8] = { 1, 10, 100, 1000, -1, -10, -100, -1000 };
volatile uint8_t index_inc = 0;
volatile bool lcd_needs_update = true;

/* --- Variáveis de Estado --- */
volatile User_inputs parametros;

extern volatile uint8_t emergency_stop; /* definida em stm32f1xx_it.c */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define FILA_TAM 8u                       /* potência de 2 */
static volatile Input_t fila[FILA_TAM];
static volatile uint8_t fila_ini = 0u;    /* só o laço principal escreve */
static volatile uint8_t fila_fim = 0u;    /* só a ISR escreve            */
volatile Input_t input_prioritario = INPUT_NENHUM; //Input parada de contagem

/* chamar na ISR, no lugar de Maquina_executar(INPUT_x) */
bool input_empilhar(Input_t in)
{
    uint8_t prox = (uint8_t)((fila_fim + 1u) & (FILA_TAM - 1u));
    if (prox == fila_ini) {
        return false;                     /* cheia: descarta */
    }
    fila[fila_fim] = in;
    fila_fim = prox;
    return true;
}

/* substitui a input_consumir() anterior */
static Input_t input_consumir(void)
{
	if (input_prioritario != INPUT_NENHUM) {
	        uint32_t primask = __get_PRIMASK();
	        __disable_irq();
	        Input_t in = input_prioritario;     /* lê e limpa de forma atômica */
	        input_prioritario = INPUT_NENHUM;
	        __set_PRIMASK(primask);
	        return in;
	    }
    if (fila_ini == fila_fim) {
        return INPUT_NENHUM;
    }
    Input_t in = fila[fila_ini];
    fila_ini = (uint8_t)((fila_ini + 1u) & (FILA_TAM - 1u));
    return in;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  /* Interrupções dos timers (o CubeMX não habilitou no .ioc do F103).
   * Prioridade menor = mais urgente: o disparo da contagem (TIM2) pode
   * interromper a malha de controle (TIM3), que pode interromper os botões. */
  HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
  HAL_NVIC_SetPriority(TIM3_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);

  HAL_TIM_Base_Start(&htim4);   /* base de tempo do LCD: antes do lcd_init() */
  lcd_init();
  lcd_put_cur(0, 0);
  lcd_send_string("Iniciado ");
  lcd_put_cur(1, 0);
  char buffer[LCD_LINHA_BUF];    /* 16 colunas + '\0' */
  char linha[32];

#define NUM_LEITURAS 10
  float historico_rpm[NUM_LEITURAS] = { 0 }; // Array preenchido com zeros
  float soma_rpm = 0.0f;
  uint8_t indice = 0;

  parametros.Fall_time = 2000;
  parametros.Offset_Counts = 2;
  parametros.RPM = 20;
  parametros.Rise_time = 2000;
  parametros.Target_Counts = 1;
  parametros.leitura = 1;
  parametros.concluida = 0;

  // 1. Ganhos do controlador (os mesmos sintonizados no G4)
  PID.Kp = 12.6f;
  PID.Ki = 68.04f;
  PID.Kd = 0.0f;

  // 2. Constante de tempo do filtro passa-baixa da derivada
  PID.tau = 0.01f;

  // 3. Tempo de amostragem: o TIM3 estoura a cada 2 ms (500 Hz)
  PID.T = 0.002f;

  // 4. Limites da saída (sinal de controle -> CCR1 do TIM1): 0 a PWM_PASSOS
  PID.limMin = 0.0f;
  PID.limMax = (float) PWM_PASSOS;

  // 5. Limites do integrador (anti-windup): 80 % do fundo de escala
  PID.limMinInt = 0.0f;
  PID.limMaxInt = 0.8f * (float) PWM_PASSOS;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  TIM1->CCR1 = 0;
  Fase_t fase_atual;
  fase_atual = Maquina_executar(INPUT_NENHUM); /* desenha a tela inicial */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* ----------------------------------------------------------------------
     * Média móvel do RPM (só para exibição)
     * ---------------------------------------------------------------------- */
    float rpm_atual = rpm_filtrado_pid;

    soma_rpm -= historico_rpm[indice]; // tira a leitura mais antiga da soma
    historico_rpm[indice] = rpm_atual; // guarda a mais nova
    soma_rpm += historico_rpm[indice]; // e soma

    media_rpm = soma_rpm / NUM_LEITURAS;

    indice++;
    if (indice >= NUM_LEITURAS) {
      indice = 0;
    }
    if (fase_atual == FASE_ATIVACAO) {
      /* snprintf + "%-16.16s": nunca estoura o buffer e apaga o resto da linha */
      snprintf(linha, sizeof(linha), "RPM:%.1f|%.1f", media_rpm, parametros.RPM);
      snprintf(buffer, sizeof(buffer), "%-16.16s", linha);
      lcd_put_cur(1, 0);
      lcd_send_string(buffer);
    }
    Input_t input = input_consumir();
    if (input != INPUT_NENHUM) fase_atual = Maquina_executar(input);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  * HSI (8 MHz) /2 -> PLL x12 -> SYSCLK = 48 MHz (sem cristal externo).
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  * APB1 tem limite de 36 MHz, então /2. 48 MHz exige 1 wait state na flash.
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function (PWM do motor, CH1 em PA8)
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = TIM1_PRESCALER;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = PWM_PASSOS - 1UL;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function (encoder em PA0/PA1 + OC CH3)
  * @note  No F103 o TIM2 é de 16 bits (no G4 era de 32). A contagem de 32 bits
  *        é reconstruída na ISR do TIM3 (ver stm32f1xx_it.c).
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  /* TI12 = quadratura x4 (4000 contagens/volta), filtros 10, como no G4.
   * Não chamamos HAL_TIM_OC_Init() antes: o Encoder_Init é quem aciona o
   * MspInit (clock + GPIO PA0/PA1) quando o handle ainda está em RESET. */
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* Canal 3 em output compare "timing": só gera a interrupção de fim de contagem */
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function (malha de controle: 2 ms)
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */
  /* O .ioc do F103 não tinha o TIM3, então o MspInit gerado não liga o clock. */
  __HAL_RCC_TIM3_CLK_ENABLE();
  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = TIM3_PRESCALER;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = CONTROL_PERIODO_US - 1UL;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function (contador livre do LCD)
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */
  __HAL_RCC_TIM4_CLK_ENABLE();
  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = TIM4_PRESCALER;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, D7_Pin|D6_Pin|D5_Pin|D4_Pin
                          |EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RW_Pin|RS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : D7_Pin D6_Pin D5_Pin D4_Pin
                           EN_Pin */
  GPIO_InitStruct.Pin = D7_Pin|D6_Pin|D5_Pin|D4_Pin
                          |EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : RW_Pin RS_Pin */
  GPIO_InitStruct.Pin = RW_Pin|RS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : BT_SEL_Pin BT_CONFIG_Pin BT_DEC_Pin BT_STOP_Pin
   * Botões ativos em nível baixo (ligados ao GND), como no G4:
   * interrupção na borda de descida com pull-up interno. */
  GPIO_InitStruct.Pin = BT_SEL_Pin|BT_CONFIG_Pin|BT_DEC_Pin|BT_STOP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init: PB12..PB15 compartilham o vetor EXTI15_10 */
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
 * @brief GPIO Interrupt Function. Handles button presses for Config, Decrement, Select and Stop.
 * @param GPIO_Pin: Specifies the pins connected to the EXTI line.
 * @retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

	/* ----------------------------------------------------------------------
	 * CONFIG -> '+'
	 * ---------------------------------------------------------------------- */
	if (GPIO_Pin == BT_CONFIG_Pin) {

		static uint32_t ultimo_aperto_conf = 0;

		// Software debounce
		if (HAL_GetTick() - ultimo_aperto_conf > 100) {
			ultimo_aperto_conf = HAL_GetTick(); // Update debounce timestamp

			input_empilhar(INPUT_MAIS);
		}

	/* ----------------------------------------------------------------------
	 * DEC -> '-'
	 * ---------------------------------------------------------------------- */
	} else if (GPIO_Pin == BT_DEC_Pin) {

		static uint32_t last_press_dec = 0;

		// Software debounce: 50ms threshold
		if (HAL_GetTick() - last_press_dec > 50) {
			last_press_dec = HAL_GetTick();

			input_empilhar(INPUT_MENOS);
		}

	/* ----------------------------------------------------------------------
	 * SEL (antigo SET) -> Enter
	 * ---------------------------------------------------------------------- */
	} else if (GPIO_Pin == BT_SEL_Pin) {

		static uint32_t last_press_sel = 0;

		// Software debounce: 50ms threshold
		if (HAL_GetTick() - last_press_sel > 50) {
			last_press_sel = HAL_GetTick(); // (no G4 faltava esta atualização)
			input_empilhar(INPUT_ENTER);
		}

	/* ----------------------------------------------------------------------
	 * STOP -> Return
	 * ---------------------------------------------------------------------- */
	} else if (GPIO_Pin == BT_STOP_Pin) {

		static uint32_t last_press_stop = 0;

		if (HAL_GetTick() - last_press_stop > 50) {
			last_press_stop = HAL_GetTick(); // Update the debounce timestamp
			input_empilhar(INPUT_RETURN);
		}
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
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
