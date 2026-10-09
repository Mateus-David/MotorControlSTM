/**
 * @file    maquina_ttl.c
 * @brief   Máquina de estados em cascata: Rotina de modo -> Rotina de
 *          configuração -> Rotina de ativação (baseada em maquinattl.drawio).
 *
 * Arquitetura
 * -----------
 *  - Três rotinas, uma por bloco tracejado do diagrama, encadeadas por
 *    Maquina_executar() através da variável de fase.
 *  - A máquina é ORIENTADA A EVENTO: só é chamada quando há um input do
 *    usuário. Por isso, nas transições de fase ela se auto-chama com
 *    INPUT_NENHUM para "avançar" o estado e desenhar a nova tela sem esperar
 *    outro clique.
 *  - Todo estado das rotinas fica em variáveis 'static' (state pattern).
 *  - O botão chega por parâmetro (Input_t). Quem chama Maquina_executar()
 *    deve limpar o botão depois de cada chamada, senão o mesmo clique é
 *    tratado de novo na próxima.
 *  - Eventos que NÃO vêm de botão (fall_end e emergency_stop, ambos escritos
 *    pela ISR) só são percebidos quando a máquina roda. Para isso existe
 *    Maquina_evento_pendente(): o loop principal deve fazer
 *
 *        if (Maquina_evento_pendente()) Maquina_executar(INPUT_NENHUM);
 *
 * Uso no boot
 * -----------
 *  Chame Maquina_executar(INPUT_NENHUM) uma vez na inicialização para
 *  desenhar a tela de seleção de modo (senão o primeiro clique do usuário
 *  seria processado sem ele ter visto a tela).
 *
 * Requisitos de StateMachine.h (assumidos, verificados por _Static_assert)
 * -----------------------------------------------------------------------
 *  Modo_t       : MODO_1 = 0, MODO_2 = 1, MODO_3 = 2, MODO_NENHUM fora de 0..2
 *  TelaConfig_t : INICIAR, RPM, VOLTAS, TEMPO_SUBIDA, TEMPO_DESCIDA,
 *                 LER_A_AB, OFFSET_COUNTS (nessa ordem, de 0 a 6)
 *  User_inputs  : campos modo, RPM, Target_Counts, Rise_time, Fall_time,
 *                 leitura e Offset_Counts do tipo float (campos[] aponta
 *                 para eles como 'volatile float *').
 */

#include "StateMachine.h"
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

/* ========================================================================== */
/*  Limites e passos dos parâmetros (ajuste conforme o hardware)               */
/* ========================================================================== */
/* ATENÇÃO: os tempos são exibidos como "Seg" no LCD, mas as macros chamam-se
 * *_MS com máximo 60000. Confirme em que unidade a ISR consome Rise_time e
 * Fall_time e acerte o rótulo (grandeza[]) ou os limites.
 * Os *_PADRAO não são usados neste arquivo: use-os onde 'parametros' é
 * inicializado (ex.: main.c). */
#define CFG_RPM_PADRAO          1000.0f
#define CFG_RPM_MIN             0.0f
#define CFG_RPM_MAX             6000.0f
#define CFG_RPM_PASSO           0.1f

#define CFG_VOLTAS_PADRAO       10.0f
#define CFG_VOLTAS_MIN          1.0f
#define CFG_VOLTAS_MAX          9999.0f
#define CFG_VOLTAS_PASSO        0.1f

#define CFG_TEMPO_PADRAO_MS     2000.0f
#define CFG_TEMPO_MIN_MS        0.0f
#define CFG_TEMPO_MAX_MS        60000.0f
#define CFG_TEMPO_PASSO_MS      0.1f

#define CFG_OFFSET_PADRAO_MS    2000.0f
#define CFG_OFFSET_MIN_MS       0.0f
#define CFG_OFFSET_MAX_MS       60000.0f
#define CFG_OFFSET_PASSO_MS     0.1f

/** Contagens do encoder por volta, em quadratura x4 (leitura AB).
 *  Se 'leitura' selecionar só o canal A, este fator muda (x1 ou x2). */
#define ENCODER_CONTAGENS_POR_VOLTA   4000.0f

#define NUM_MODOS               3u
#define NUM_TELAS               7u
#define LCD_COLUNAS             16u

///* Se algum destes asserts falhar, a ordem dos enums em StateMachine.h não
// * bate com a ordem usada em campos[], grandeza[] e nomes_tela[]. */
//_Static_assert(MODO_1 == 0 && MODO_2 == 1 && MODO_3 == 2,
//		"Modo_t deve ser MODO_1=0, MODO_2=1, MODO_3=2");
//_Static_assert((int)MODO_NENHUM > 2 || (int)MODO_NENHUM < 0,
//		"MODO_NENHUM nao pode coincidir com um modo valido");
//_Static_assert(TELA_INICIAR == 0 && TELA_RPM == 1 && TELA_VOLTAS == 2
//		&& TELA_TEMPO_SUBIDA == 3 && TELA_TEMPO_DESCIDA == 4
//		&& TELA_LER_A_AB == 5 && TELA_OFFSET_COUNTS == 6,
//		"Ordem de TelaConfig_t diferente de campos[]/nomes_tela[]");

/* ========================================================================== */
/*  Variáveis compartilhadas com ISRs / outros arquivos                        */
/* ========================================================================== */
extern volatile User_inputs parametros;  /* parâmetros do usuário            */
extern volatile PIDController PID;       /* parâmetros do PID                */
extern volatile encoder Encoder;

extern TIM_HandleTypeDef htim1;          /* PWM do motor (CH1)               */
extern TIM_HandleTypeDef htim2;          /* encoder + output compare (CH3)   */
extern TIM_HandleTypeDef htim3;          /* base de tempo do loop do PID     */

extern volatile uint8_t emergency_stop;  /* ISR -> máquina: parada de emerg. */
extern volatile uint8_t fall_end;        /* ISR -> máquina: rampa de descida */
extern volatile uint8_t rise_ramp;       /* máquina -> ISR: inicia a subida  */
extern volatile float voltas_totais;     /* mostrar no display               */
extern int32_t pos_antiga;
extern float rpm_rampa;                  /* setpoint dinâmico perseguido pelo PID */

/* ========================================================================== */
/*  Tabelas de texto e acesso aos parâmetros                                   */
/* ========================================================================== */

/** Parâmetros na mesma ordem de TelaConfig_t (índice = tela). */
static volatile float *const campos[NUM_TELAS] = { &parametros.modo,
		&parametros.RPM, &parametros.Target_Counts, &parametros.Rise_time,
		&parametros.Fall_time, &parametros.leitura,
		&parametros.Offset_Counts };

/** Unidade exibida ao lado do valor (máx. 6 caracteres + '\0'). */
static const char grandeza[NUM_TELAS][7] = { "", /* Iniciar (mostra o modo) */
"RPM", /* RPM      */
"Counts", /* Voltas   */
"Seg", /* Subida   */
"Seg", /* Descida  */
"", /* Encoder  */
"Counts" /* Offset   */
};

/** Nome de cada tela na lista de configuração. */
static const char *const nomes_tela[NUM_TELAS] = { "Iniciar", "RPM", "Voltas",
		"T. Subida", "T. Descida", "Encoder", "Offset" };

static const char *const nomes_modo[NUM_MODOS] = { "Modo 1", "Modo 2",
		"Modo 3" };

/* ========================================================================== */
/*  LCD 16x2: helpers (todas as linhas são completadas com espaços)            */
/* ========================================================================== */

/** Buffer de formatação: 16 colunas + '\0'. Sempre use snprintf com ele. */
static char buffer[LCD_COLUNAS + 1u];

/** Escreve 'texto' na linha, truncando em 16 e completando com espaços para
 *  apagar o que havia antes (evita "RPMDescida"). */
static void lcd_linha(uint8_t linha, const char *texto) {
	lcd_put_cur(linha, 0);
	snprintf(buffer, sizeof(buffer), "%-16.16s", texto);
	lcd_send_string(buffer);
}

/** Desenha a tela de configuração correspondente ao estado atual. */
static void desenhar_config(EstadoConfig_t estado, TelaConfig_t tela) {
	switch (estado) {
	case CFG_NAVEGANDO: /* "Param: valor" / nome do item */
		lcd_put_cur(0, 0);
		snprintf(buffer, sizeof(buffer), "Param: %-9.1f",
				(double) *campos[tela]);
		lcd_send_string(buffer);
		lcd_linha(1, nomes_tela[tela]);
		break;

	case CFG_AJUSTANDO: /* nome / "valor    unidade" */
		lcd_linha(0, nomes_tela[tela]);
		lcd_put_cur(1, 0);
		snprintf(buffer, sizeof(buffer), "%-10.1f%-6s", (double) *campos[tela],
				grandeza[tela]);
		lcd_send_string(buffer);
		break;

	case CFG_CONFIRMANDO: { /* confirmação + modo escolhido */
		uint32_t m = (uint32_t) parametros.modo;
		if (m >= NUM_MODOS) {
			m = NUM_MODOS - 1u;
		}
		lcd_linha(0, "Confirmar Inicio");
		lcd_linha(1, nomes_modo[m]);
		break;
	}
	default:
		break;
	}
}

/** Tela exibida enquanto o motor roda. A atualização da contagem durante a
 *  execução precisa ser feita fora da máquina (ela só roda com input). */
static void mostrar_rodando(void) {
	lcd_put_cur(0, 0);
	snprintf(buffer, sizeof(buffer), "Rodando|Cnt:%-4d", (int) voltas_totais);
	lcd_send_string(buffer);
}

/* ========================================================================== */
/*  FUNÇÃO 1 - Rotina de modo                                                  */
/* ========================================================================== */

/**
 * @brief  Navega entre os 3 modos.
 *         + : próximo (3 -> 1 com wrap)   - : anterior (1 -> 3 com wrap)
 *         Enter : confirma e encerra a rotina.
 *         INPUT_NENHUM só redesenha se for a primeira chamada da rodada.
 * @return MODO_NENHUM enquanto o usuário navega; o modo escolhido no passo
 *         em que Enter é pressionado.
 */
Modo_t Rotina_de_modo(Input_t Input) {
	static Modo_t modo = MODO_1;
	static bool primeira_vez = true; /* true => desenha a tela ao entrar */

	Modo_t selecionado = MODO_NENHUM;
	bool atualizar = primeira_vez;
	primeira_vez = false;

	switch (Input) {
	case INPUT_MAIS:
		modo = (Modo_t) ((modo + 1u) % NUM_MODOS);
		atualizar = true;
		break;
	case INPUT_MENOS:
		modo = (Modo_t) ((modo + NUM_MODOS - 1u) % NUM_MODOS);
		atualizar = true;
		break;
	case INPUT_ENTER:
		selecionado = modo;
		primeira_vez = true; /* próxima rodada redesenha            */
		atualizar = false; /* a próxima fase assume o display     */
		break;
	default:
		break;
	}

	if (atualizar) {
		lcd_linha(0, "Selecione o modo");
		lcd_linha(1, nomes_modo[modo]);
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
	case TELA_LER_A_AB: /* opção binária: + ou - alterna */
		parametros.leitura = (parametros.leitura > 0.5f) ? 0.0f : 1.0f;
		break;
	case TELA_OFFSET_COUNTS:
		parametros.Offset_Counts = ajustar_valor(parametros.Offset_Counts,
				incrementa,
				CFG_OFFSET_PASSO_MS, CFG_OFFSET_MIN_MS, CFG_OFFSET_MAX_MS);
		break;
	case TELA_INICIAR: /* não é parâmetro */
	default:
		break;
	}
}

/**
 * @brief  Menu de parâmetros.
 *  CFG_NAVEGANDO  : + / - percorrem a lista (COM wrap, em módulo 7).
 *                   Enter em "Iniciar" -> CFG_CONFIRMANDO;
 *                   Enter em outro item -> CFG_AJUSTANDO.
 *  CFG_AJUSTANDO  : + / - alteram o parâmetro; Return volta à lista, no item
 *                   "Iniciar" (para ficar no item editado, remova a linha
 *                   'tela_atual = TELA_INICIAR' deste case).
 *  CFG_CONFIRMANDO: Return volta a "Iniciar"; Enter finaliza a configuração.
 *
 * Os valores ficam em 'parametros' e são retidos entre rodadas, então o
 * usuário parte dos últimos valores usados.
 *
 * A conclusão é sinalizada em parametros.concluida (== 1 só na chamada em
 * que o Enter de confirmação foi tratado). A flag é zerada a cada chamada.
 *
 * @param  Input  Botão pressionado (INPUT_NENHUM só redesenha, se preciso).
 * @return Estado interno do menu (a conclusão NÃO é indicada por ele).
 */
EstadoConfig_t Rotina_de_configuracao(Input_t Input) {
	static EstadoConfig_t estado = CFG_NAVEGANDO;
	static TelaConfig_t tela_atual = TELA_INICIAR;
	static bool primeira_vez = true; /* true => desenha a tela ao entrar */

	bool atualizar = primeira_vez;
	primeira_vez = false;

	parametros.concluida = 0;

	switch (estado) {

	/* ----------------------------------------------------------------- */
	case CFG_NAVEGANDO:
		switch (Input) {
		case INPUT_MAIS: /* desce na lista */
			tela_atual = (TelaConfig_t) ((tela_atual + 1u) % NUM_TELAS);
			atualizar = true;
			break;
		case INPUT_MENOS: /* sobe na lista */
			tela_atual = (TelaConfig_t) ((tela_atual + NUM_TELAS - 1u)
					% NUM_TELAS);
			atualizar = true;
			break;
		case INPUT_ENTER:
			estado = (tela_atual == TELA_INICIAR) ? CFG_CONFIRMANDO :
														CFG_AJUSTANDO;
			atualizar = true;
			break;
		default:
			break;
		}
		break;

		/* ----------------------------------------------------------------- */
	case CFG_AJUSTANDO:
		switch (Input) {
		case INPUT_MAIS:
			ajustar_parametro(tela_atual, true);
			atualizar = true;
			break;
		case INPUT_MENOS:
			ajustar_parametro(tela_atual, false);
			atualizar = true;
			break;
		case INPUT_RETURN:
			estado = CFG_NAVEGANDO;
			tela_atual = TELA_INICIAR;
			atualizar = true;
			break;
		default:
			break;
		}
		break;

		/* ----------------------------------------------------------------- */
	case CFG_CONFIRMANDO:
		switch (Input) {
		case INPUT_RETURN: /* volta ao Iniciar */
			estado = CFG_NAVEGANDO;
			tela_atual = TELA_INICIAR;
			atualizar = true;
			break;
		case INPUT_ENTER: /* finaliza a configuração */
			parametros.concluida = 1;
			estado = CFG_NAVEGANDO; /* prepara a próxima rodada          */
			tela_atual = TELA_INICIAR;
			primeira_vez = true; /* próxima rodada redesenha          */
			atualizar = false; /* a fase de ativação assume o LCD   */
			break;
		default:
			break;
		}
		break;

		/* ----------------------------------------------------------------- */
	default: /* recuperação */
		estado = CFG_NAVEGANDO;
		tela_atual = TELA_INICIAR;
		atualizar = true;
		break;
	}

	if (atualizar) {
		desenhar_config(estado, tela_atual);
	}
	return estado;
}

/* ========================================================================== */
/*  FUNÇÃO 3 - Rotina de ativação                                              */
/* ========================================================================== */

/**
 * @brief  Liga/desliga o motor e supervisiona a execução.
 *
 *  ATIV_INIT    : zera PID/rampa/encoder, liga os periféricos e vai para
 *                 ATIV_RODANDO no mesmo passo. O TIM3 (ISR do PID, que também
 *                 conduz as rampas) é o ÚLTIMO a ser ligado.
 *  ATIV_RODANDO : encerra quando houver Return, fall_end (fim da rampa de
 *                 descida, vindo da ISR) ou emergency_stop. No Modo 3, + e -
 *                 alteram o RPM em tempo real.
 *  ATIV_PARANDO : desliga tudo (TIM3 primeiro) e volta para ATIV_INIT, pronto
 *                 para a próxima rodada.
 *
 *  Quem chama (Maquina_executar) enxerga ATIV_PARANDO no retorno do passo em
 *  que a parada foi decidida e chama a rotina de novo com INPUT_NENHUM para
 *  executar o case ATIV_PARANDO.
 *
 * @param  Input  Botão pressionado.
 * @return Estado ao final do passo.
 */
EstadoAtiv_t Rotina_de_ativacao(Input_t Input) {
	static EstadoAtiv_t estado = ATIV_INIT;

	switch (estado) {

	/* ----------------------------------------------------------------- */
	case ATIV_INIT:
		/* A ISR do TIM3 está parada aqui, então descartar o 'volatile' é
		 * seguro. (Alternativa: declarar o parâmetro de PIDController_Init
		 * como 'volatile PIDController *'.) */
		PIDController_Init((PIDController *) &PID);

		/* Estado compartilhado com a ISR, ajustado ANTES de ligar os timers. */
		rpm_rampa = 0;
		pos_antiga = 0;
		fall_end = 0;

		/* Encoder (TIM2): zera a contagem e programa a parada por contagem.
		 * Soma 0.5f para arredondar: o passo de 0.1 acumula erro de float e a
		 * conversão direta para uint32_t truncaria (erro de 1 contagem). */
		TIM2->CNT = 0;
		TIM2->CCR3 = (uint32_t) ((parametros.Target_Counts
				+ parametros.Offset_Counts) * ENCODER_CONTAGENS_POR_VOLTA
				+ 0.5f);
		if (parametros.modo == MODO_2) { /* modo de contagem */
			HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_3);
		}
		HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
		HAL_Delay(10); /* espera única na partida; ver observação na entrega */
		update_encoder(&Encoder, &htim2);

		/* PWM (TIM1) começa em 0 % para o PID assumir a partir daí. */
		TIM1->CCR1 = 0;
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

		/* Pede a rampa de subida e só então liga o loop de controle (TIM3). */
		rise_ramp = 1;
		HAL_TIM_Base_Start_IT(&htim3);

		estado = ATIV_RODANDO;
		break;

		/* ----------------------------------------------------------------- */
	case ATIV_PARANDO:
		HAL_TIM_Base_Stop_IT(&htim3); /* 1º: a ISR não mexe mais no PWM */
		TIM1->CCR1 = 0;
		HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);

		HAL_TIM_OC_Stop_IT(&htim2, TIM_CHANNEL_3);
		HAL_TIM_Encoder_Stop(&htim2, TIM_CHANNEL_ALL);

		emergency_stop = 0; /* motor já está parado: libera nova partida */
		parametros.concluida = 0;

		estado = ATIV_INIT;
		break;

		/* ----------------------------------------------------------------- */
	case ATIV_RODANDO:
		if (Input == INPUT_RETURN || fall_end ) {
			estado = ATIV_PARANDO;
			break;
		}
		if (parametros.modo == MODO_3) { /* controle contínuo de velocidade */
			if (Input == INPUT_MAIS) {
				parametros.RPM = ajustar_valor(parametros.RPM, true,
				CFG_RPM_PASSO, CFG_RPM_MIN, CFG_RPM_MAX);
			} else if (Input == INPUT_MENOS) {
				parametros.RPM = ajustar_valor(parametros.RPM, false,
				CFG_RPM_PASSO, CFG_RPM_MIN, CFG_RPM_MAX);
			}
		}
		break;

	default: /* estado inválido: o mais seguro é parar o motor */
		estado = ATIV_PARANDO;
		break;
	}
	return estado;
}

/* ========================================================================== */
/*  Encadeamento em cascata                                                    */
/* ========================================================================== */

static Fase_t fase_atual = FASE_MODO;

/**
 * @brief  Um passo da máquina completa (chamar a cada input do usuário).
 *         Nas trocas de fase a própria máquina chama a rotina seguinte com
 *         INPUT_NENHUM para desenhar a nova tela / executar a partida.
 */
Fase_t Maquina_executar(Input_t Input) {
	switch (fase_atual) {

	case FASE_MODO: {
		Modo_t modo = Rotina_de_modo(Input);
		if (modo != MODO_NENHUM) {
			parametros.modo = modo;
			fase_atual = FASE_CONFIG;
			Rotina_de_configuracao(INPUT_NENHUM); /* desenha a 1ª tela */
		}
		break;
	}

	case FASE_CONFIG:
		Rotina_de_configuracao(Input);
		if (parametros.concluida) {
			fase_atual = FASE_ATIVACAO;
			mostrar_rodando();
			Rotina_de_ativacao(INPUT_NENHUM); /* executa ATIV_INIT */
		}
		break;

	case FASE_ATIVACAO:
		if (Rotina_de_ativacao(Input) == ATIV_PARANDO) {
			Rotina_de_ativacao(INPUT_NENHUM); /* executa ATIV_PARANDO */
			fase_atual = FASE_MODO;
			Rotina_de_modo(INPUT_NENHUM); /* redesenha a seleção de modo */
		}
		break;

	default:
		fase_atual = FASE_MODO;
		Rotina_de_modo(INPUT_NENHUM);
		break;
	}
	return fase_atual;
}

/**
 * @brief  Indica que há um evento vindo da ISR que a máquina precisa tratar
 *         mesmo sem input do usuário (fim da rampa de descida ou parada de
 *         emergência com o motor em operação).
 *         Declare o protótipo em StateMachine.h.
 */
bool Maquina_evento_pendente(void) {
	return (fase_atual == FASE_ATIVACAO) && (fall_end || emergency_stop);
}
