/***
 * @autor: Mariana da Silva Lima Santos
 * Painel de controle interativo para um RU (Restaurante Universitário). 
 * 
 * O projeto utiliza o microcontrolador Raspberry Pi Pico com o sistema operacional FreeRTOS,
 * um display OLED SSD1306, LEDs RGB e um buzzer.
 * 
 * O painel permite o controle de entrada e saída de usuários,
 * além de fornecer feedback visual e sonoro sobre o estado do RU.
 */
#include <stdio.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "semphr.h"

#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"

#include "lib/ssd1306.h"
#include "lib/font.h"

#define I2C_PORT i2c1
#define I2C_SDA 14
#define I2C_SCL 15
#define address 0x3C
#define BUZZER_A 21
#define BUZZER_B 10
#define buttonA 5
#define buttonB 6
#define joystickPB 22
#define MAX_SIZE 20 // Capacidade máxima do RU
#define led_pin_r 13
#define led_pin_b 12
#define led_pin_g 11
#define PWM_DIVISER 50
#define PWM_WRAP 4000 


// Prototipação
void vTaskEntrada(void *pvParameters);
void vTaskSaida(void *pvParameters);
void vTaskReset(void *pvParameters);
void gpio_irq_handler(uint gpio, uint32_t events);
void gpio_init_all();
void buzzer_short_beep();
void buzzer_double_beep();
void update_display();
void update_led_rgb();
void _pwm_init(int pin);

// Variáveis globais
volatile uint8_t usuarios = 0;
ssd1306_t ssd; 

// Semáforos e Mutex
SemaphoreHandle_t xSemContador;
SemaphoreHandle_t xSemReset;
SemaphoreHandle_t xMutexDisplay;

int main()
{
    stdio_init_all();
    gpio_init_all();

    // Inicializações
    xSemContador = xSemaphoreCreateCounting(MAX_SIZE, MAX_SIZE);
    xSemReset = xSemaphoreCreateBinary();
    xMutexDisplay = xSemaphoreCreateMutex();

    // Configuração da interrupção
    gpio_set_irq_enabled_with_callback(joystickPB, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);

    // Criação das tarefas
    xTaskCreate(vTaskEntrada, "Entrada", 2048, NULL, 1, NULL);
    xTaskCreate(vTaskSaida, "Saida", 2048, NULL, 1, NULL);
    xTaskCreate(vTaskReset, "Reset", 2048, NULL, 2, NULL);

    // Inicializa led RGB e display OLED
    update_led_rgb();
    update_display();

    vTaskStartScheduler();
    panic_unsupported();
}

void _pwm_init(int pin) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(pin);
    pwm_set_clkdiv(slice_num, PWM_DIVISER);
    pwm_set_wrap(slice_num, PWM_WRAP);

    pwm_set_gpio_level(pin, 0);
    pwm_set_enabled(slice_num, true);
}

void gpio_init_all() {
    // Inicializa os pinos do display
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
    
    // Inicializa o display OLED
    ssd1306_init(&ssd, WIDTH, HEIGHT, false, address, I2C_PORT); // Inicializa o display
    ssd1306_config(&ssd);                                         // Configura o display
    ssd1306_send_data(&ssd);                                      // Envia os dados para o display

    // Limpa o display. O display inicia com todos os pixels apagados.
    ssd1306_fill(&ssd, false);
    ssd1306_send_data(&ssd);
    
    // Inicializa LEDs RGB
    gpio_init(led_pin_r);
    gpio_set_dir(led_pin_r, GPIO_OUT); // Configura o pino como saída
    gpio_init(led_pin_g);
    gpio_set_dir(led_pin_g, GPIO_OUT);
    gpio_init(led_pin_b);
    gpio_set_dir(led_pin_b, GPIO_OUT); 

    // Inicializa Buzzer com PWM
    _pwm_init(BUZZER_A);
    
    // Botões
    gpio_init(buttonA);
    gpio_set_dir(buttonA, GPIO_IN); // Configura o pino do botão A como entrada
    gpio_pull_up(buttonA);
    gpio_init(buttonB);
    gpio_set_dir(buttonB, GPIO_IN); // Configura o pino do botão B como entrada
    gpio_pull_up(buttonB);
    gpio_init(joystickPB);
    gpio_set_dir(joystickPB, GPIO_IN);
    gpio_pull_up(joystickPB);
}

// Funções de Hardware
void buzzer_short_beep() {
    uint slice_num = pwm_gpio_to_slice_num(BUZZER_A);

    // Ativa duty cycle de 50%
    pwm_set_gpio_level(BUZZER_A, PWM_WRAP / 2);
    //sleep_ms(150); // duração do beep
    vTaskDelay(pdMS_TO_TICKS(150));

    // Silencia o buzzer
    pwm_set_gpio_level(BUZZER_A, 0);
}

void buzzer_double_beep() {
    buzzer_short_beep();
    vTaskDelay(pdMS_TO_TICKS(100));
    //sleep_ms(100);  // Pausa entre os beeps
    buzzer_short_beep();
}

void update_display() {
    xSemaphoreTake(xMutexDisplay, portMAX_DELAY);
    char buffer[20];
    snprintf(buffer, 20, "Pessoas no RU: %d\n", usuarios);
    ssd1306_draw_string(&ssd, buffer, 0, 4);
    snprintf(buffer, 20, "Cota restante: %d\n", MAX_SIZE - usuarios);
    ssd1306_draw_string(&ssd, buffer, 0, 24);
    ssd1306_send_data(&ssd);
    printf("Usuários no RU: %d\n", usuarios);
    printf("Cota restante: %d\n", MAX_SIZE - usuarios);
    xSemaphoreGive(xMutexDisplay);
}

void update_led_rgb() {
    gpio_put(led_pin_r, 0);
    gpio_put(led_pin_g, 0);
    gpio_put(led_pin_b, 0);

    if (usuarios == 0)
        gpio_put(led_pin_b, 1); // Azul
    else if (usuarios < MAX_SIZE - 1)
        gpio_put(led_pin_g, 1); // Verde
    else if (usuarios == MAX_SIZE - 1)
        gpio_put(led_pin_r, 1), gpio_put(led_pin_g, 1); // Amarelo
    else
        gpio_put(led_pin_r, 1); // Vermelho
}

// Tarefa de entrada
void vTaskEntrada(void *pvParameters) {
    while (1) {
        if (gpio_get(buttonA) == 0) {
            vTaskDelay(pdMS_TO_TICKS(20)); // debounce inicial
            if (gpio_get(buttonA) == 0) {  // Confirma se ainda está pressionado
                if (xSemaphoreTake(xSemContador, 0) == pdTRUE) {
                    usuarios++;
                    update_led_rgb();
                    update_display();
                } else {
                    printf("Capacidade máxima atingida!\n");
                    buzzer_short_beep();
                }
                while (gpio_get(buttonA) == 0) {
                    vTaskDelay(pdMS_TO_TICKS(10)); // Espera até botão ser solto
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// Tarefa de saída
void vTaskSaida(void *pvParameters) {
    while (1) {
        if (gpio_get(buttonB) == 0 && usuarios > 0) {
            vTaskDelay(pdMS_TO_TICKS(20)); // debounce inicial
            if (gpio_get(buttonB) == 0) {
                usuarios--;
                xSemaphoreGive(xSemContador);
                update_led_rgb();
                update_display();
                vTaskDelay(pdMS_TO_TICKS(300));
                while (gpio_get(buttonB) == 0) {
                    vTaskDelay(pdMS_TO_TICKS(10)); // Espera até botão ser solto
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// Tarefa de reset
void vTaskReset(void *pvParameters) {
    while (1) {
        if (xSemaphoreTake(xSemReset, portMAX_DELAY) == pdTRUE) {
            // Calcula quantas permissões faltam para encher o semáforo até MAX
            for (uint8_t i = 0; i < usuarios; i++) {
                xSemaphoreGive(xSemContador);
            }
            usuarios = 0;
            if (xSemaphoreTake(xMutexDisplay, portMAX_DELAY) == pdTRUE) {
                printf("Reset realizado!\n");
                printf("Usuários: %d\n", usuarios);
                xSemaphoreGive(xMutexDisplay);
            }
            update_led_rgb();
            update_display();
            buzzer_double_beep();
        }
    }
}

// Interrupção do botão de reset (joystick)
void gpio_irq_handler(uint gpio, uint32_t events) {
    static uint32_t last_time = 0;
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    if (current_time - last_time < 200) {
        // Ignora interrupções dentro de 200ms (debounce)
        return;
    }
    last_time = current_time;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (gpio == joystickPB) {
        xSemaphoreGiveFromISR(xSemReset, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}