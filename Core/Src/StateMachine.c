/**
 * @file    maquina_ttl.c
 * @brief   Máquina de estados em cascata: Rotina de modo -> Rotina de
 *          configuração -> Rotina de ativação (baseada em maquinattl.drawio).
 *
 * Arquitetura
 * -----------
 *  - Três rotinas, uma por bloco tracejado do diagrama.
 *  - Cada rotina é NÃO-BLOQUEANTE: executa UM passo por chamada e devolve
 *    imediatamente. Todo o estado fica em variáveis 'static' (state pattern).
 *  - O botão pressionado chega pela variável global 'input_atual' (escrita
 *    pela ISR/driver de botões) e é CONSUMIDO (zerado) pela rotina que o lê,
 *    de modo que um mesmo clique nunca é tratado duas vezes.
 *  - Cascata: Rotina_de_modo() -> Modo_t
 *             Rotina_de_configuracao(Modo_t) -> Config_t
 *             Rotina_de_ativacao(const Config_t *) -> AtivStatus_t
 *    A função Maquina_executar() (chamada no super-loop) apenas encadeia
 *    as três, guardando em qual fase o sistema está.
 *
 * Simulação no PC (opcional):
 *    gcc -Wall -Wextra -DMAQUINA_TTL_SIM maquina_ttl.c -o sim && ./sim
 */

#include "StateMachine.h"

/* ========================================================================== */
/*  Limites e valores padrão dos parâmetros (ajuste conforme o hardware)       */
/* ========================================================================== */
#define CFG_RPM_PADRAO          1000.0f
#define CFG_RPM_MIN             0u
#define CFG_RPM_MAX             6000.0f
#define CFG_RPM_PASSO           0.1f

#define CFG_VOLTAS_PADRAO       10u
#define CFG_VOLTAS_MIN          1u
#define CFG_VOLTAS_MAX          9999u
#define CFG_VOLTAS_PASSO        0.1f

#define CFG_TEMPO_PADRAO_MS     2000u
#define CFG_TEMPO_MIN_MS        0u
#define CFG_TEMPO_MAX_MS        60000u
#define CFG_TEMPO_PASSO_MS      0.1f

#define CFG_OFFSET_PADRAO_MS     2000u
#define CFG_OFFSET_MIN_MS        0u
#define CFG_OFFSET_MAX_MS        60000u
#define CFG_OFFSET_PASSO_MS      0.1f

/* ========================================================================== */
/*  Variável global de entrada                                                 */
/* ========================================================================== */
extern volatile User_inputs parametros;
extern volatile PIDController PID;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim6;

extern volatile uint8_t emergency_stop;

extern volatile encoder Encoder;
extern UART_HandleTypeDef huart1;
extern volatile float voltas_totais; //Mostrar no display
extern volatile uint32_t pwm; //Controle do pwm
extern volatile PIDController PID; // Parametros do PID
extern volatile float rpm_alvo; // ALVO
extern volatile float rpm_filtrado_pid;
extern volatile User_inputs parametros; // Param
extern volatile uint32_t voltas_tim;
extern int32_t pos_antiga;
extern float rpm_rampa; // Setpoint dinâmico (a rampa) que o PID vai perseguir
extern volatile uint8_t fall_end;
extern volatile uint8_t rise_ramp;
extern volatile Input_t input_prioritario;

/* ========================================================================== */
/*  Const VARS                                                */
/* ========================================================================== */

volatile float *campos[] = { &parametros.modo, &parametros.RPM,
		&parametros.Target_Counts, &parametros.Rise_time, &parametros.Fall_time,
		&parametros.leitura, &parametros.Offset_Counts };

char const grandeza[7][7] = { "", //MODO
		"RPM", //RPM
		"Counts", //TARGET
		"Seg", //RISE
		"Seg", // FALL
		"", // leitura
		"Counts" //Offset
		};

char buffer[17];
/* ========================================================================== */
/*  FUNÇÃO 1 - Rotina de modo                                                  */
/* ========================================================================== */

/**
 * @brief  Navega entre os 3 modos.
 *         + : próximo (3 -> 1 com wrap)   - : anterior (1 -> 3 com wrap)
 *         Enter : confirma e encerra a rotina.
 * @return MODO_NENHUM enquanto o usuário navega; o modo escolhido no passo
 *         em que Enter é pressionado.
 */
Modo_t Rotina_de_modo(Input_t Input) {
	static const char display_txt[3][20] = { "Modo 1", "Modo 2", "Modo 3" };

	static Modo_t modo = MODO_1;
	static bool primeira_vez = true;

	Modo_t selecionado = MODO_NENHUM;
	bool atualizar = primeira_vez;
	primeira_vez = false;

	if (Input == INPUT_MAIS) {
		modo = (Modo_t) ((modo + 1) % 3);
		atualizar = true;
	} else if (Input == INPUT_MENOS) {
		modo = (Modo_t) ((modo + 2) % 3);
		atualizar = true;
	} else if (Input == INPUT_ENTER) {
		selecionado = modo;
		primeira_vez = true;

	}

	if (atualizar) {
		lcd_put_cur(1, 0);
		snprintf(buffer, sizeof(buffer), "%-16s",
				display_txt[modo]);
		lcd_send_string(buffer);
	}

	return selecionado;
}

/* ========================================================================== */
/*  FUNÇÃO 2 - Rotina de configuração                                          */
/* ========================================================================== */

/** Soma/subtrai 'passo' a 'valor' saturando em [min, max] (sem underflow). */
static float ajustar_valor(float valor, bool incrementa, float passo, float min,
		float max) {
	if (incrementa) {
		valor = ((max - valor) > passo) ? (valor + passo) : max;
	} else {
		valor = ((valor - min) > passo) ? (valor - passo) : min;
	}
	return valor;
}

/** Aplica + ou - ao parâmetro da tela atual (estado CFG_AJUSTANDO). */
static void ajustar_parametro(TelaConfig_t tela, bool incrementa) {
	switch (tela) {
	case TELA_RPM:
		parametros.RPM = ajustar_valor(parametros.RPM, incrementa,
		CFG_RPM_PASSO, CFG_RPM_MIN, CFG_RPM_MAX);
		break;
	case TELA_VOLTAS:
		parametros.Target_Counts = ajustar_valor(parametros.Target_Counts,
				incrementa,
				CFG_VOLTAS_PASSO, CFG_VOLTAS_MIN, CFG_VOLTAS_MAX);
		break;
	case TELA_TEMPO_DESCIDA:
		parametros.Fall_time = ajustar_valor(parametros.Fall_time, incrementa,
		CFG_TEMPO_PASSO_MS, CFG_TEMPO_MIN_MS, CFG_TEMPO_MAX_MS);
		break;
	case TELA_TEMPO_SUBIDA:
		parametros.Rise_time = ajustar_valor(parametros.Rise_time, incrementa,
		CFG_TEMPO_PASSO_MS, CFG_TEMPO_MIN_MS, CFG_TEMPO_MAX_MS);
		break;
	case TELA_LER_A_AB: /* opção binária: + ou - alterna       */
		parametros.leitura = (parametros.leitura == 1) ? 0 : 1;
		break;
	case TELA_OFFSET_COUNTS:
		parametros.Offset_Counts = ajustar_valor(parametros.Offset_Counts,
				incrementa,
				CFG_OFFSET_PASSO_MS, CFG_OFFSET_MIN_MS, CFG_OFFSET_MAX_MS);
	case TELA_INICIAR: /* não é parâmetro                     */
	default:
		break;
	}
}

/**
 * @brief  Menu de parâmetros.
 *  CFG_NAVEGANDO : + / - percorrem a lista (sem wrap; ver diagrama).
 *                  Enter em "Iniciar" -> CFG_CONFIRMANDO;
 *                  Enter em outro item -> CFG_AJUSTANDO.
 *  CFG_AJUSTANDO : + / - alteram o parâmetro; Return volta à lista.
 *  CFG_CONFIRMANDO: Return volta a "Iniciar"; Enter finaliza a configuração.
 *
 * Os valores ficam retidos entre execuções (static), então na próxima
 * rodada o usuário parte dos últimos valores usados.
 *
 * @param  modo  Modo selecionado pela Rotina_de_modo().
 * @return Cópia da configuração; 'concluida' == true apenas no passo em que
 *         o usuário confirmou.
 */
EstadoConfig_t Rotina_de_configuracao(Input_t Input) {

	static const char display_txt[7][20] = { "Iniciar", "RPM", "Voltas",
			"T. Subida", "T. Descida", "Encoder", "Offset" };

	static EstadoConfig_t estado = CFG_NAVEGANDO;
	static TelaConfig_t tela_atual = TELA_INICIAR; // Tela atual

	static bool primeira_vez = true;
	bool atualizar = primeira_vez;
	primeira_vez = false;

	parametros.concluida = 0;

	switch (estado) {

	/* ----------------------------------------------------------------- */
	case CFG_NAVEGANDO:

		switch (Input) {
		case INPUT_MAIS: /* desce na lista   */
			tela_atual = (TelaConfig_t) ((tela_atual + 1) % 7);
			atualizar = 1;

			break;
		case INPUT_MENOS: /* sobe na lista    */
			tela_atual = (TelaConfig_t) ((tela_atual + 6) % 7);
			atualizar = 1;

			break;
		case INPUT_ENTER:
			if (tela_atual != TELA_INICIAR) {
				estado = CFG_AJUSTANDO;
				estado =
						(tela_atual == TELA_INICIAR) ?
								CFG_CONFIRMANDO : CFG_AJUSTANDO;
				lcd_put_cur(0, 0);
				snprintf(buffer, sizeof(buffer), "%-16s",
						display_txt[tela_atual]);
				lcd_send_string(buffer);
				lcd_put_cur(1, 0);
				snprintf(buffer, sizeof(buffer), "%-10.1f%-6s",
						*campos[tela_atual], grandeza[tela_atual]);
				lcd_send_string(buffer);
			} else {
				atualizar = 0;
				estado = CFG_CONFIRMANDO;
				lcd_put_cur(0, 0);
				lcd_send_string("Confirmar Inicio");
				lcd_put_cur(1, 0);
				if (parametros.modo == 0)
					lcd_send_string("Modo 1");
				else if (parametros.modo == 1)
					lcd_send_string("Modo 2");
				else
					lcd_send_string("Modo 3");
				break;
			}
		default:
			break;
		}
		if (atualizar) {
			lcd_put_cur(0, 0);
			snprintf(buffer, sizeof(buffer), "Param: %-9.1f",
					*campos[tela_atual]);
			lcd_send_string(buffer);
			lcd_put_cur(1, 0);
			lcd_send_string(display_txt[tela_atual]);
			atualizar = 0;

		}
		break;

		/* ----------------------------------------------------------------- */
	case CFG_AJUSTANDO:
		switch (Input) {
		case INPUT_MAIS:
			ajustar_parametro(tela_atual, true);
			atualizar = 1;
			break;
		case INPUT_MENOS:
			ajustar_parametro(tela_atual, false);
			atualizar = 1;
			break;
		case INPUT_RETURN:
			estado = CFG_NAVEGANDO;
			lcd_put_cur(0, 0);
			snprintf(buffer, sizeof(buffer), "Param: %-10.1f",
					*campos[tela_atual]);
			lcd_send_string(buffer);
			lcd_put_cur(1, 0);
			lcd_send_string("A                 ");
			tela_atual = TELA_INICIAR;
			break;
		default:
			break;
		}
		if (atualizar) {
			lcd_put_cur(1, 0);
			snprintf(buffer, sizeof(buffer), "%-9.1f%-6s",
					*campos[tela_atual ], grandeza[tela_atual]);
			lcd_send_string(buffer);

			atualizar = 0;
		}

		break;

		/* ----------------------------------------------------------------- */
	case CFG_CONFIRMANDO:
		switch (Input) {
		case INPUT_RETURN: /* volta ao Iniciar */
			estado = CFG_NAVEGANDO;
			tela_atual = TELA_INICIAR;
			lcd_put_cur(0, 0);
			snprintf(buffer, sizeof(buffer), "Param:%-10.1f", *campos[0]);
			lcd_send_string(buffer);
			lcd_put_cur(1, 0);
			lcd_send_string("A                 ");

			break;
		case INPUT_ENTER: /* finaliza   ADICIONAR COISA LA DO PARAMETROS  . CONCLUIDA       */
			parametros.concluida =1;
			estado = CFG_NAVEGANDO; /* prepara próxima  */
			tela_atual = TELA_INICIAR; /* rodada           */
			break;
		default:
			break;
		}
		break;

	default: /* recuperação      */
		estado = CFG_NAVEGANDO;
		tela_atual = TELA_INICIAR;
		break;
	}
	return estado;
}


/**
 * @brief  Executa a operação com os parâmetros recebidos.
 *
 *  Premissas (o diagrama não detalha esta rotina; ajuste se necessário):
 *   PARADO        : Enter = partir; Return = encerrar a ativação.
 *   RAMPA_SUBIDA  : rampa linear 0 -> RPM alvo em 'tempo_subida_ms'.
 *   REGIME        : mantém o RPM; no Modo 3, + e - alteram o RPM em tempo real.
 *   RAMPA_DESCIDA : rampa linear até 0 em 'tempo_descida_ms', volta a PARADO.
 *
 * @param  cfg  Configuração gerada pela Rotina_de_configuracao().
 * @return ATIV_EM_EXECUCAO enquanto ativa; ATIV_CONCLUIDA ao encerrar.
 */
EstadoAtiv_t Rotina_de_ativacao(Input_t Input) {
static EstadoAtiv_t estado = ATIV_INIT;

switch (estado) {

/* ----------------------------------------------------------------- */
case ATIV_INIT:
	PIDController_Init(&PID);

	/* --- Peripheral Initialization Phase --- */

	// Start Timer 2 Output Compare in interrupt mode (Often used to trigger precise velocity measurements)
	TIM2->CCR3 = (parametros.Target_Counts + parametros.Offset_Counts) * 4000;
	TIM2->CNT = 0;
	pos_antiga = 0;
	rpm_rampa = 0;
	fall_end = 0;
	if (parametros.modo == MODO_2) { //Se estiver no modo de contagem
		HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_3);
	}
//	HAL_TIM_OC_Start(&htim2, TIM_CHANNEL_3);
	// Start Timer 2 in Encoder interface mode to track quadrature encoder pulses
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	HAL_Delay(10);
	update_encoder(&Encoder, &htim2);

	// Start Timer 3 in interrupt mode (Likely used for the main control loop / PID execution time base)
	HAL_TIM_Base_Start_IT(&htim3);

	// Initialize Motor PWM duty cycle (Capture/Compare Register 1) to 0% (stopped)
	TIM1->CCR1 = 0;

	// Start PWM signal generation on Timer 1 Channel 1
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	//static float save_rpm = parametros.RPM;
	rise_ramp = 1;
	estado = ATIV_RODANDO;
	break;

	/* ----------------------------------------------------------------- */
case ATIV_PARANDO:
	TIM1->CCR1 = 0;
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);

	HAL_TIM_OC_Stop_IT(&htim2, TIM_CHANNEL_3);

	HAL_TIM_OC_Stop_IT(&htim2, TIM_CHANNEL_3);
	HAL_TIM_Encoder_Stop(&htim2, TIM_CHANNEL_ALL);
	emergency_stop = 0;
	parametros.concluida = 0;

	estado = ATIV_INIT;
	break;

	/* ----------------------------------------------------------------- */
case ATIV_RODANDO:
	if (Input == INPUT_RETURN || fall_end) {
		estado = ATIV_PARANDO;
		break;
	}
	if (parametros.modo == MODO_3) {
		if (Input == INPUT_MAIS) {
			parametros.RPM = ajustar_valor(parametros.RPM, 1,
			CFG_RPM_PASSO, CFG_RPM_MIN, CFG_RPM_MAX);
		} else if (Input == INPUT_MENOS) {
			parametros.RPM = ajustar_valor(parametros.RPM, 0,
			CFG_RPM_PASSO, CFG_RPM_MIN, CFG_RPM_MAX);
		}

	}

	break;
}
return estado;
}
/* ========================================================================== */
/*  Encadeamento em cascata (chamar periodicamente no super-loop)              */
/* ========================================================================== */

/**
 * @brief  Um passo da máquina completa. Não bloqueia.
 *         modo -> Rotina_de_configuracao(modo) -> Rotina_de_ativacao(&cfg)
 */
Fase_t Maquina_executar(Input_t Input) {
static Fase_t fase = FASE_MODO;
static Modo_t modo = MODO_NENHUM;

switch (fase) {
case FASE_MODO:
	lcd_put_cur(0, 0);
	lcd_send_string("Selecione o modo:     "); // Prompt user. Trailing spaces clear previous display artifacts

	modo = Rotina_de_modo(Input);
	if (modo != MODO_NENHUM) {
		parametros.modo = modo;
		fase = FASE_CONFIG;
		lcd_put_cur(0, 0);
		snprintf(buffer, sizeof(buffer), "Param:%-10.1f", *campos[0]);
		lcd_send_string(buffer);
		lcd_put_cur(1, 0);
		lcd_send_string("A                 ");

	}

	break;

case FASE_CONFIG:

	Rotina_de_configuracao(Input);
	if (parametros.concluida ) {
		fase = FASE_ATIVACAO;
		lcd_put_cur(0, 0);
		sprintf(buffer, "Rodando  |Cnt: %d ", (int)voltas_totais);
		lcd_send_string(buffer);
		Rotina_de_ativacao(INPUT_NENHUM);
	}
	break;

case FASE_ATIVACAO:

	if (Rotina_de_ativacao(Input) == ATIV_PARANDO) {
		Rotina_de_ativacao(INPUT_NENHUM);
		parametros.concluida = 0;

		fase = FASE_MODO;
		Maquina_executar(INPUT_NENHUM);

	}
	break;

default:
	fase = FASE_MODO;
	break;
}
return fase;
}
