/*
 * WALL-E FORCE | ROBO SUMO AUTONOMO | ESP32-WROOM-32 | ESP-IDF 5.x
 * 1 HC-SR04 frontal + 3 TCRT5000 + ponte H TB6612FNG.
 * Referencia didatica: pulso TRIG 10us, largura do ECHO e timeout 25ms.
 * Versao experimental: ao detectar alvo, ataca por 5s SEM checar bordas.
 * Maquina de estados sem delays de movimentacao.
 * IMPORTANTISSIMO: HC-SR04 convencional VCC=5V; ECHO -> divisor para 3,3V.
 * Os TCRT5000 devem fornecer saida logica de 3,3 V ao ESP32.
 * Verificar polaridade real dos motores e nivel da borda antes de competir.
 */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "esp_err.h"

// ========================= 1. PINAGEM ============================
#define TRIG_FRENTE 32
#define ECHO_FRENTE 34
#define TCRT_ESQUERDO 36
#define TCRT_DIREITO 39
#define TCRT_TRASEIRO 18
#define AIN1 25
#define AIN2 26
#define PWMA 27
#define BIN1 23
#define BIN2 22
#define PWMB 21

// ========================= 2. PARAMETROS ========================
#define NIVEL_BORDA 0              // AJUSTAR conforme sensores na faixa branca
#define INVERTER_MOTOR_ESQUERDO 1  // confirmar com rodas suspensas
#define INVERTER_MOTOR_DIREITO 1   // confirmar com rodas suspensas
#define IGNORAR_BORDAS_NO_ATAQUE 1 // durante os 5s: bordas ignoradas (RISCO DE SAIDA)
#define GIRO_BUSCA_UM_MOTOR 0      // 0: giro no eixo com motores em sentidos opostos
#define DISTANCIA_ATAQUE_CM 35     // evita detectar objetos fora do dojo
#define DISTANCIA_LIBERAR_CM 40
#define LEITURAS_LONGE_SAIR 2
#define TOLERANCIA_ECHO_INVALIDO_MS 100
#define INTERVALO_PING_MS 65
#define TIMEOUT_ECHO_US 25000
#define VELOCIDADE_ATAQUE 255
#define VELOCIDADE_BUSCA 150
#define VELOCIDADE_GIRO 180
#define VELOCIDADE_RECUO 220
#define FREQUENCIA_PWM 20000
#define TEMPO_RECUO_MS 110 // ajustar fisicamente conforme arena
#define TEMPO_AVANCO_TRASEIRO_MS 110
#define TEMPO_GIRO_EVASAO_MS 140
#define PASSO_LOOP_MS 10
#define LOG_CADA_MS 350
#define PARTIDA_SEGUNDOS 5
#define TEMPO_ATAQUE_CEGO_MS 3000 

// ========================= 3. ESTADOS ===========================
typedef enum
{
    BUSCANDO,
    ATACANDO,
    EVADINDO,
    PARADO_SEGURANCA
} estado_t;
typedef enum
{
    MOV_PARAR,
    MOV_ATAQUE,
    MOV_RECUAR,
    MOV_AVANCO_EVASAO,
    MOV_GIRO_DIREITA,
    MOV_GIRO_ESQUERDA
} movimento_t;
typedef enum
{
    EV_RE,
    EV_GIRO,
    EV_AVANCO,
    EV_FIM
} fase_t;
typedef struct
{
    bool esq, dir, tras;
} bordas_t;

static estado_t estado = BUSCANDO;
static movimento_t movimento_atual = MOV_PARAR;
static fase_t fase = EV_FIM;
static bool evasao_para_direita = true;
static int64_t fim_fase_us = 0;
static int64_t proximo_ping_us = 0;
static int64_t ultima_deteccao_us = 0;
static int64_t ultimo_log_us = 0;
static int64_t fim_ataque_cegous = 0;
static int distancia_frente_cm = -1;
static unsigned leituras_longe = 0;
static TaskHandle_t tarefa_principal = NULL;
static const gpio_num_t pinos_ir[3] = {
    (gpio_num_t)TCRT_ESQUERDO, (gpio_num_t)TCRT_DIREITO,
    (gpio_num_t)TCRT_TRASEIRO};

static int64_t agora_us(void) { return esp_timer_get_time(); }
static int64_t ms_us(int ms) { return (int64_t)ms * 1000LL; }

// Funcao da estrutura ensinada pelo professor.
void delay_ms(int ms) { vTaskDelay(pdMS_TO_TICKS(ms)); }

// ========================= 4. MOTORES ===========================
static void motor(gpio_num_t in1, gpio_num_t in2, ledc_channel_t canal, int vel)
{
    if (vel > 255)
        vel = 255;
    if (vel < -255)
        vel = -255;
    // Desliga PWM antes de mudar sentido; so ocorre ao mudar movimento.
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, canal, 0));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, canal));
    ESP_ERROR_CHECK(gpio_set_level(in1, vel > 0));
    ESP_ERROR_CHECK(gpio_set_level(in2, vel < 0));
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, canal, vel < 0 ? -vel : vel));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, canal));
}

static void mover(movimento_t comando)
{
    if (comando == movimento_atual)
        return;
    int e = 0, d = 0;
    switch (comando)
    {
    case MOV_ATAQUE:
        e = VELOCIDADE_ATAQUE;
        d = VELOCIDADE_ATAQUE;
        break;
    case MOV_RECUAR:
        e = -VELOCIDADE_RECUO;
        d = -VELOCIDADE_RECUO;
        break;
    case MOV_AVANCO_EVASAO:
        e = VELOCIDADE_RECUO;
        d = VELOCIDADE_RECUO;
        break;
    case MOV_GIRO_DIREITA:
        if (GIRO_BUSCA_UM_MOTOR)
        {
            e = (estado == EVADINDO) ? VELOCIDADE_GIRO : VELOCIDADE_BUSCA;
            d = 0;
        }
        else
        {
            e = (estado == EVADINDO) ? VELOCIDADE_GIRO : VELOCIDADE_BUSCA;
            d = -e;
        }
        break;
    case MOV_GIRO_ESQUERDA:
        if (GIRO_BUSCA_UM_MOTOR)
        {
            e = 0;
            d = (estado == EVADINDO) ? VELOCIDADE_GIRO : VELOCIDADE_BUSCA;
        }
        else
        {
            d = (estado == EVADINDO) ? VELOCIDADE_GIRO : VELOCIDADE_BUSCA;
            e = -d;
        }
        break;
    case MOV_PARAR:
    default:
        break;
    }
    if (INVERTER_MOTOR_ESQUERDO)
        e = -e;
    if (INVERTER_MOTOR_DIREITO)
        d = -d;
    motor((gpio_num_t)AIN1, (gpio_num_t)AIN2, LEDC_CHANNEL_0, e);
    motor((gpio_num_t)BIN1, (gpio_num_t)BIN2, LEDC_CHANNEL_1, d);
    movimento_atual = comando;
    printf("MOTOR -> %s | E=%d D=%d\n",
           comando == MOV_ATAQUE ? "ATAQUE" : comando == MOV_RECUAR      ? "RE"
                                          : comando == MOV_AVANCO_EVASAO ? "AVANCO EVASAO"
                                          : comando == MOV_GIRO_DIREITA  ? "GIRO DIREITA"
                                          : comando == MOV_GIRO_ESQUERDA ? "GIRO ESQUERDA"
                                                                         : "PARAR",
           e, d);
}

static void configurar_motores(void)
{
    gpio_num_t dir_pinos[] = {(gpio_num_t)AIN1, (gpio_num_t)AIN2,
                              (gpio_num_t)BIN1, (gpio_num_t)BIN2};
    for (unsigned i = 0; i < 4; ++i)
    {
        ESP_ERROR_CHECK(gpio_set_direction(dir_pinos[i], GPIO_MODE_OUTPUT));
        ESP_ERROR_CHECK(gpio_set_level(dir_pinos[i], 0));
    }
    ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE, .timer_num = LEDC_TIMER_0, .duty_resolution = LEDC_TIMER_8_BIT, .freq_hz = FREQUENCIA_PWM, .clk_cfg = LEDC_AUTO_CLK};
    ESP_ERROR_CHECK(ledc_timer_config(&timer));
    ledc_channel_config_t a = {
        .gpio_num = PWMA, .speed_mode = LEDC_LOW_SPEED_MODE, .channel = LEDC_CHANNEL_0, .timer_sel = LEDC_TIMER_0, .duty = 0, .hpoint = 0};
    ledc_channel_config_t b = {
        .gpio_num = PWMB, .speed_mode = LEDC_LOW_SPEED_MODE, .channel = LEDC_CHANNEL_1, .timer_sel = LEDC_TIMER_0, .duty = 0, .hpoint = 0};
    ESP_ERROR_CHECK(ledc_channel_config(&a));
    ESP_ERROR_CHECK(ledc_channel_config(&b));
    movimento_atual = (movimento_t)-1;
    mover(MOV_PARAR);
}

// ========================= 5. BORDAS / INTERRUPCOES =============
static bordas_t ler_bordas(void)
{
    return (bordas_t){
        .esq = gpio_get_level((gpio_num_t)TCRT_ESQUERDO) == NIVEL_BORDA,
        .dir = gpio_get_level((gpio_num_t)TCRT_DIREITO) == NIVEL_BORDA,
        .tras = gpio_get_level((gpio_num_t)TCRT_TRASEIRO) == NIVEL_BORDA};
}
static bool tem_borda(bordas_t b) { return b.esq || b.dir || b.tras; }
static bool bordas_opostas(bordas_t b) { return b.tras && (b.esq || b.dir); }

// ISR so notifica: nunca imprime, mede ou controla motores dentro dela.
static void isr_borda(void *contexto)
{
    uint32_t mascara = 1U << (uintptr_t)contexto;
    BaseType_t acordou = pdFALSE;
    if (tarefa_principal)
    {
        xTaskNotifyFromISR(tarefa_principal, mascara, eSetBits, &acordou);
        if (acordou == pdTRUE)
            portYIELD_FROM_ISR();
    }
}

static void configurar_infravermelhos(void)
{
    tarefa_principal = xTaskGetCurrentTaskHandle();
    // GPIO36/39 nao tem pull-up interno: saida dos modulos deve ser estavel.
    // Interrupcao por flanco + leitura periodica do nivel: evita
    // perder uma borda persistente, mesmo sem novo flanco.
    gpio_int_type_t tipo = (NIVEL_BORDA == 0) ? GPIO_INTR_NEGEDGE : GPIO_INTR_POSEDGE;
    for (unsigned i = 0; i < 3; ++i)
    {
        ESP_ERROR_CHECK(gpio_set_direction(pinos_ir[i], GPIO_MODE_INPUT));
        ESP_ERROR_CHECK(gpio_set_intr_type(pinos_ir[i], tipo));
    }
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    for (unsigned i = 0; i < 3; ++i)
    {
        ESP_ERROR_CHECK(gpio_isr_handler_add(pinos_ir[i], isr_borda, (void *)(uintptr_t)i));
    }
}

// ========================= 6. HC-SR04 ===========================
static void configurar_ultrassonico(void)
{
    ESP_ERROR_CHECK(gpio_set_direction((gpio_num_t)TRIG_FRENTE, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_direction((gpio_num_t)ECHO_FRENTE, GPIO_MODE_INPUT));
    ESP_ERROR_CHECK(gpio_set_level((gpio_num_t)TRIG_FRENTE, 0));
}

// Conforme os slides: TRIG 10us; mede ECHO alto, timeout nos dois loops.
// NUNCA fazer gpio_set_level(ECHO_FRENTE, 0): ECHO e uma ENTRADA!
// Retorna -1 quando a leitura e invalida ou quando uma borda exige atencao.
static int ler_HC_SR04(bool monitorar_borda)
{
    if (gpio_get_level((gpio_num_t)ECHO_FRENTE) != 0)
        return -1;
    ESP_ERROR_CHECK(gpio_set_level((gpio_num_t)TRIG_FRENTE, 0));
    esp_rom_delay_us(2);
    ESP_ERROR_CHECK(gpio_set_level((gpio_num_t)TRIG_FRENTE, 1));
    esp_rom_delay_us(10);
    ESP_ERROR_CHECK(gpio_set_level((gpio_num_t)TRIG_FRENTE, 0));
    int64_t inicio = agora_us();
    while (gpio_get_level((gpio_num_t)ECHO_FRENTE) == 0)
    {
        if (monitorar_borda && tem_borda(ler_bordas()))
            return -1;
        if (agora_us() - inicio >= TIMEOUT_ECHO_US)
            return -1;
    }
    int64_t subida = agora_us();
    while (gpio_get_level((gpio_num_t)ECHO_FRENTE) == 1)
    {
        if (monitorar_borda && tem_borda(ler_bordas()))
            return -1;
        if (agora_us() - subida >= TIMEOUT_ECHO_US)
            return -1;
    }
    int64_t duracao = agora_us() - subida;
    int cm = (int)(duracao * 343LL / 20000LL); // cm, pulso em us
    return (cm >= 2 && cm <= 400) ? cm : -1;
}

// ========================= 7. EVASAO ============================
static void iniciar_evasao(bordas_t b)
{
    estado = EVADINDO;
    leituras_longe = 0;
    ultima_deteccao_us = 0;
    if (b.tras && !b.esq && !b.dir)
    {
        printf("BORDA TRASEIRA -> AVANCAR\n");
        fase = EV_AVANCO;
        fim_fase_us = agora_us() + ms_us(TEMPO_AVANCO_TRASEIRO_MS);
        mover(MOV_AVANCO_EVASAO);
    }
    else
    {
        printf("BORDA FRONTAL -> RECUAR\n");
        evasao_para_direita = !b.dir; // ESQ -> DIR; DIR -> ESQ
        fase = EV_RE;
        fim_fase_us = agora_us() + ms_us(TEMPO_RECUO_MS);
        mover(MOV_RECUAR);
    }
}

static void executar_evasao(void)
{
    bordas_t b = ler_bordas();
    if (bordas_opostas(b) ||
        (fase == EV_RE && b.tras) ||
        (fase == EV_AVANCO && (b.esq || b.dir)))
    {
        // Nao continuar manobra se uma borda aparecer no sentido do movimento.
        estado = PARADO_SEGURANCA;
        fase = EV_FIM;
        mover(MOV_PARAR);
        printf("EVASAO CANCELADA: BORDA NO SENTIDO DO MOVIMENTO\n");
        return;
    }
    if (agora_us() < fim_fase_us)
        return;
    if (fase == EV_RE)
    {
        fase = EV_GIRO;
        fim_fase_us = agora_us() + ms_us(TEMPO_GIRO_EVASAO_MS);
        mover(evasao_para_direita ? MOV_GIRO_DIREITA : MOV_GIRO_ESQUERDA);
    }
    else if (fase == EV_GIRO || fase == EV_AVANCO)
    {
        fase = EV_FIM;
        // Apos evasao, nao volta ao combate ate sair da faixa.
        if (tem_borda(b))
        {
            estado = PARADO_SEGURANCA;
            mover(MOV_PARAR);
            printf("BORDA PERSISTENTE -> PARADO, VERIFICAR POSICAO\n");
        }
        else
        {
            estado = BUSCANDO;
            mover(MOV_GIRO_DIREITA);
        }
    }
}

// ========================= 8. COMBATE ===========================
static void controlar_robo(void)
{
    int64_t t = agora_us();

    // ATAQUE TEMPORIZADO: deliberadamente nao consulta IR nem ultrassonico
    // durante 5s. A ISR ainda pode notificar, mas o motor NAO e interrompido.
    // ATENCAO: dojo de 60cm: pode atravessar a faixa e sair da arena!
    if (estado == ATACANDO)
    {
        if (t < fim_ataque_cegous)
            return;

        // Ao finalizar a janela de 5s, examina PRIMEIRO as bordas.
        printf("ATAQUE 5s ENCERRADO -> VERIFICANDO BORDAS\n");
        estado = BUSCANDO;
        bordas_t b_pos_ataque = ler_bordas();
        if (tem_borda(b_pos_ataque))
        {
            if (bordas_opostas(b_pos_ataque))
            {
                estado = PARADO_SEGURANCA;
                mover(MOV_PARAR);
            }
            else
                iniciar_evasao(b_pos_ataque);
            return;
        }
        // Sem borda: para o ataque e volta a procurar.
        // Proxima leitura valida pode iniciar outra janela de 5s.
        mover(MOV_GIRO_DIREITA);
        proximo_ping_us = 0;
        return;
    }

    bordas_t b = ler_bordas();
    if (estado == PARADO_SEGURANCA)
    {
        if (!tem_borda(b))
        {
            printf("FAIXA LIBERADA -> BUSCA\n");
            estado = BUSCANDO;
            proximo_ping_us = 0;
            mover(MOV_GIRO_DIREITA);
        }
        return;
    }
    if (estado == EVADINDO)
    {
        executar_evasao();
        return;
    }

    // Durante a BUSCA, a borda continua com prioridade.
    if (tem_borda(b))
    {
        if (bordas_opostas(b))
        {
            estado = PARADO_SEGURANCA;
            mover(MOV_PARAR);
        }
        else
            iniciar_evasao(b);
        return;
    }

    if (t < proximo_ping_us)
        return;
    distancia_frente_cm = ler_HC_SR04(true);
    t = agora_us();
    proximo_ping_us = t + ms_us(INTERVALO_PING_MS);

    // Borda que surgiu durante a medicao tem prioridade antes do ataque.
    b = ler_bordas();
    if (tem_borda(b))
        return;

    if (distancia_frente_cm >= 2 && distancia_frente_cm <= DISTANCIA_ATAQUE_CM)
    {
        printf("ALVO %d cm -> ATAQUE CONTINUO POR 5s\n", distancia_frente_cm);
        estado = ATACANDO;
        fim_ataque_cegous = t + ms_us(TEMPO_ATAQUE_CEGO_MS);
        mover(MOV_ATAQUE);
        return;
    }

    mover(MOV_GIRO_DIREITA);
    if (t - ultimo_log_us >= ms_us(LOG_CADA_MS))
    {
        printf("FRENTE=%d cm | ESTADO=BUSCANDO\n", distancia_frente_cm);
        ultimo_log_us = t;
    }
}

// ========================= 9. ESTRUTURA DA AULA ================
void setup(void)
{
    configurar_motores();
    configurar_ultrassonico();
    configurar_infravermelhos();
    printf("WALL-E FORCE | TESTE ATAQUE 5s | HC-SR04 FRONTAL + 3 IR\n");
    for (int i = PARTIDA_SEGUNDOS; i > 0; --i)
    {
        printf("INICIO EM %d...\n", i);
        delay_ms(1000);
    }
    estado = BUSCANDO;
    proximo_ping_us = 0;
    printf("COMBATE INICIADO\n");
}

void loop(void)
{
    while (1)
    {
        controlar_robo();
        // ISR acorda o loop; polling de 10ms confirma bordas persistentes.
        uint32_t notificacoes = 0;
        (void)xTaskNotifyWait(0, UINT32_MAX, &notificacoes,
                              pdMS_TO_TICKS(PASSO_LOOP_MS));
    }
}

void app_main(void)
{
    setup();
    loop();
}
