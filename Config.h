#pragma once

#include <stdint.h>

// Ajuste primeiro as constantes deste arquivo.
// Leia LEIA_PRIMEIRO.md antes de energizar o carrinho.
namespace cfg {

// =====================================================
// MODO DE EXECUCAO / PARTIDA
// =====================================================

constexpr bool COMPETICAO = true;

constexpr uint32_t BUSCA_PARTIDA_MS = 5000;
constexpr uint32_t PAUSA_TENTATIVA_MS = 3000;
constexpr int QUADROS_PARTIDA = 15;
constexpr uint32_t ESPERA_PARTIDA_MS = 5000;
constexpr uint32_t TEMPO_MAX_PROVA_MS = 600000;

// =====================================================
// REENCONTRO DA FITA
// =====================================================

// Forca relativa usada quando a fita some.
constexpr float ESFORCO_REENCONTRO = 0.90f;

// O reencontro usa pulsos curtos para mover, olhar novamente e corrigir.
constexpr uint32_t REENCONTRO_MOVE_MS = 180;
constexpr uint32_t REENCONTRO_PAUSA_MS = 70;

// Por quanto tempo a ultima direcao conhecida da fita continua valida.
constexpr uint32_t MEMORIA_FITA_MS = 8000;

// Desativado por padrao: ao perder a fita, procura imediatamente na direcao
// memorizada da curva em vez de primeiro recuar.
constexpr bool RETORNO_ATIVO = false;
constexpr uint32_t RETORNO_MS = 400;
constexpr uint32_t RETORNO_PAUSA_MS = 150;

// =====================================================
// CURVAS
// =====================================================

constexpr bool CURVAS_EM_PASSOS = false;
constexpr float ESFORCO_CURVA_FECHADA = 0.70f;

// Curva forte normal: reduz bastante a roda interna para realmente fechar o raio.
// Curva fechada/cotovelo: permite parar a roda interna durante o pivot.
constexpr float LIMIAR_CURVA_FORTE = 0.55f;
constexpr float FRACAO_INTERNA_CURVA_FORTE = 0.25f;
constexpr float FRACAO_INTERNA_CURVA_FECHADA = 0.0f;

// Transicao entre curvas fechadas de sentidos opostos, sem esperar uma reta.
constexpr bool MODO_CURVAS_EM_S = false;
constexpr int S_QUADROS_TROCA = 2;
constexpr uint32_t S_PAUSA_TROCA_MS = 100;

constexpr float ZONA_MORTA_ERRO = 0.030f;

// Menor Y inicia o giro mais cedo; maior Y espera o cotovelo se aproximar.
constexpr int COTOVELO_PERTO_Y = 48;

// =====================================================
// ORIENTACAO / PINOS
// =====================================================

// Na montagem atual, PWM positivo girava os dois lados para tras.
// Invertemos ambos para que os comandos positivos do seguidor sejam avanco.
constexpr bool INVERTER_DIREITA = false;
constexpr bool INVERTER_ESQUERDA = false;
constexpr bool ESPELHAR_CAMERA = false;
constexpr bool VIRAR_CAMERA_VERTICAL = false;

constexpr int IN1 = 12;  // direita
constexpr int IN2 = 13;  // direita
constexpr int IN3 = 14;  // esquerda
constexpr int IN4 = 15;  // esquerda

// =====================================================
// PWM / MOTORES
// =====================================================

constexpr int PWM_HZ = 1000;

// Piso mecanico: abaixo disso o motor pode apenas fazer ruido sem partir.
constexpr int PWM_MIN_ESQ = 120;
constexpr int PWM_MIN_DIR = 120;

constexpr int PWM_MAX = 255;
constexpr int PWM_RETA = 120;
constexpr int PWM_CURVA = 140;
constexpr int PWM_CAUTELA = 120;
constexpr int PWM_REENCONTRO = 135;
constexpr int PWM_CRUZAMENTO = 120;
constexpr int PWM_GIRO = 155;

// Impulso curto e moderado: suficiente para vencer a inercia sem dar um salto.
constexpr int PWM_PARTIDA = 1600;
constexpr uint32_t IMPULSO_MS = 60;
                                                                                    
// Pequena pausa ao trocar do avanco para a curva fechada.
constexpr uint32_t ACOMODAR_GIRO_MS = 25;

// Reducao aplicada conforme aumenta o risco de curva a frente.
constexpr float REDUCAO_ANTECIPACAO = 0.30f;
constexpr uint32_t MEMORIA_CURVA_MS = 180;

// Protecoes de mudanca de sentido e expiracao de comando.
constexpr uint32_t REVERSAO_MS = 60;
constexpr uint32_t COMANDO_EXPIRA_MS = 300;
constexpr uint32_t FRAME_MAX_IDADE_MS = 220;

// =====================================================
// VISAO
// =====================================================

constexpr int W = 160;
constexpr int H = 120;

constexpr int N = 23;
constexpr int Y[N] = {
    112, 108, 104, 100, 96, 92, 88, 84,
    80,  76,  72,  68,  64, 60, 56, 52,
    48,  44,  40,  36,  32, 28, 24
};

constexpr int MIN_LARGURA = 3;
constexpr int MIN_CONTRASTE = 28;

// Inclui faixas distantes para antecipar cotovelos e curvas graduais.
constexpr int ROI_TOPO_Y = 24;
constexpr float FRACAO_LIMIAR_AUTO = 0.20f;
constexpr float LARGURA_PERTO = 12.0f;
constexpr float LARGURA_LONGE = 5.0f;
constexpr int LIMIAR_FIXO = -1;
constexpr float CENTRO_CAMERA = 79.5f;

constexpr float KP = 1.65f;
constexpr float KD = 0.045f;

// Prioriza o trecho proximo nas curvas em S.
constexpr float PESO_DIRECAO = 0.60f;

// Esforcos relativos. Os tetos PWM e speedScale regulam a saida real.
constexpr float BASE_RETA = 0.12f;
constexpr float BASE_CURVA = 0.10f;

// =====================================================
// CONTROLE POR PASSOS
// =====================================================

// Quando false, reta e curvas normais sao continuas.
// RECOVERY continua usando pulsos curtos para procurar a pista.
constexpr bool PASSOS_ATIVOS = false;

constexpr uint32_t RETA_MOVE_MS = 140;
constexpr uint32_t RETA_PAUSA_MS = 55;

constexpr uint32_t CURVA_MOVE_MS = RETA_MOVE_MS;
constexpr uint32_t CURVA_PAUSA_MS = 30;

constexpr uint32_t GIRO_MOVE_MS = 140;
constexpr uint32_t GIRO_PAUSA_MS = 40;

constexpr uint32_t CRUZ_MOVE_MS = 140;
constexpr uint32_t CRUZ_PAUSA_MS = 30;

// =====================================================
// LIMITES DE SEGURANCA
// =====================================================

constexpr uint32_t CRUZAMENTO_MAX_MS = 3000;
constexpr uint32_t GIRO_MAX_MS = 6000;

}  // namespace cfg
