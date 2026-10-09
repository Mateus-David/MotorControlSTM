/*
 * StateMachine.h
 *
 *  Created on: 30 de set. de 2026
 *      Author: padil
 */

#ifndef INC_STATEMACHINE_H_
#define INC_STATEMACHINE_H_

//INCLUDES

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "main.h"
#include <stm32f1xx_it.h>
#include "PID.h"
#include "LCD1602.h"
#include "motor_encoder.h"



/* Exported types ------------------------------------------------------------*/

/* ========================================================================== */
/*  Tipos                                                                      */
/* ========================================================================== */

/** Botões do sistema. */
typedef enum {
    INPUT_NENHUM = 0,   /**< Nenhum botão pendente */
    INPUT_MAIS,         /**< Botão +      */
    INPUT_MENOS,        /**< Botão -      */
    INPUT_ENTER,        /**< Botão Enter  */
    INPUT_RETURN        /**< Botão Return */
} Input_t;

/** Modos de operação (Rotina de modo). */
typedef enum {
    MODO_NENHUM = -1,   /**< Ainda não selecionado (rotina em andamento) */
    MODO_1 = 0,         /**< Botão Partir e parar                        */
    MODO_2,             /**< Botão Partir e parar por contagem           */
    MODO_3,             /**< Botão Partir e controle de vel. contínuo    */
    MODO_QTD
} Modo_t;

/** Telas (itens) do menu de configuração, na ordem do diagrama. */
typedef enum {
    TELA_INICIAR = 0,
    TELA_RPM,
    TELA_VOLTAS,
    TELA_TEMPO_DESCIDA,
    TELA_TEMPO_SUBIDA,
    TELA_LER_A_AB,
	TELA_OFFSET_COUNTS,
    TELA_QTD
} TelaConfig_t;

/** Sub-estados da Rotina de configuração. */
typedef enum {
    CFG_NAVEGANDO = 0,  /**< Percorrendo a lista com + e -                    */
    CFG_AJUSTANDO,      /**< "Botão + ou - para alterar o parâmetro"          */
    CFG_CONFIRMANDO,     /**< Tela "Confirma"                                  */
	CFG_COMPLETO
} EstadoConfig_t;

/** Leitura do encoder: só canal A ou A+B (quadratura). */
typedef enum {
    LEITURA_A = 0,
    LEITURA_AB
} Leitura_t;

/** Todos os parâmetros da máquina (saída da Rotina de configuração). */
typedef struct {
    Modo_t    modo;              /**< Modo escolhido na Rotina de modo         */
    uint32_t  rpm;               /**< Velocidade alvo [rpm]                    */
    uint32_t  voltas;            /**< Nº de voltas (usado no Modo 2)           */
    uint32_t  tempo_descida_ms;  /**< Duração da rampa de descida [ms]         */
    uint32_t  tempo_subida_ms;   /**< Duração da rampa de subida [ms]          */
    Leitura_t leitura;           /**< Leitura A ou AB                          */
    bool      concluida;         /**< true no passo em que o usuário confirmou */
} Config_t;

/** Sub-estados da Rotina de ativação. */
typedef enum {
    ATIV_INIT = 0,      /**< Inicializa periféricos/variáveis                  */
    ATIV_PARANDO,        /**< Pronto; aguarda "partir"                          */
    ATIV_RODANDO,  /**< Acelerando até o RPM alvo                         */
    ATIV_REGIME,        /**< Mantendo o RPM alvo                               */
    ATIV_RAMPA_DESCIDA  /**< Desacelerando até parar                           */
} EstadoAtiv_t;

typedef enum {
    FASE_MODO = 0,
    FASE_CONFIG,
    FASE_ATIVACAO
} Fase_t;

/* Functions ------------------------------------------------------------*/

Modo_t Rotina_de_modo(Input_t Input);
static float ajustar_valor(float valor, bool incrementa, float passo, float min, float max);
static void ajustar_parametro(TelaConfig_t tela, bool incrementa);
EstadoConfig_t Rotina_de_configuracao( Input_t Input);
EstadoAtiv_t Rotina_de_ativacao(Input_t Input);
Fase_t Maquina_executar(Input_t Input);
#endif /* INC_STATEMACHINE_H_ */
