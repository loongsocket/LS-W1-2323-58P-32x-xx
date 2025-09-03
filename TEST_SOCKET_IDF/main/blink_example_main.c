/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/uart.h"
#include "driver/ledc.h"

#include "esp_log.h"
#include "sdkconfig.h"

#include "led_strip.h"

//v3

// ==============================
// Configuración / Defines
// ==============================

static const char *TAG = "example";

// Botones en GPIO del ESP32
#define BUTTON1_GPIO 36
#define BUTTON2_GPIO 34
#define BUTTON3_GPIO 35
#define BUTTON4_GPIO 32
#define BUTTON5_GPIO 33
#define BUTTON6_GPIO 2

// SPI Pins (bus VSPI)
#define PIN_MISO 19
#define PIN_MOSI 23
#define PIN_CLK  18
#define CS_MCP3208   26
#define CS_MCP23S17  25

// Pin INT del MCP23S17 hacia el ESP32
// OJO: GPIO39 no tiene pull-up interno -> usar pull-up externo (~10k a 3V3)
#define PIN_NUM_MCP_INT  GPIO_NUM_39

// UART1
#define UART_PORT UART_NUM_1
#define UART_TX 4
#define UART_RX 5
#define UART_BUF_SIZE 1024

// ADC MCP3208
#define MCP3208_MAX 4095.0f
#define VREF        2.048f
#define R1 6200.0f
#define R2 10000.0f

// Opcodes MCP23S17 (SPI, A2..A0 = 0; si no, ajusta)
#define MCP23S17_OPCODE_WRITE 0x40
#define MCP23S17_OPCODE_READ  0x41

// Registros MCP23S17 (BANK=0)
#define IODIRA   0x00
#define IODIRB   0x01
#define IPOLA    0x02
#define IPOLB    0x03
#define GPINTENA 0x04
#define GPINTENB 0x05
#define DEFVALA  0x06
#define DEFVALB  0x07
#define INTCONA  0x08
#define INTCONB  0x09
#define IOCON    0x0A  // (también 0x0B)
#define GPPUA    0x0C
#define GPPUB    0x0D
#define INTFA    0x0E
#define INTFB    0x0F
#define INTCAPA  0x10
#define INTCAPB  0x11
#define GPIOA    0x12
#define GPIOB    0x13
#define OLATA    0x14
#define OLATB    0x15

// Bits útiles de IOCON (BANK=0)
#define IOCON_BANK   (1<<7)
#define IOCON_MIRROR (1<<6)  // une INTA/INTB
#define IOCON_SEQOP  (1<<5)
#define IOCON_DISSLW (1<<4)
#define IOCON_HAEN   (1<<3)  // habilita direcciones HW en SPI
#define IOCON_ODR    (1<<2)  // INT open-drain
#define IOCON_INTPOL (1<<1)  // ignóralo si ODR=1

// PWM

#define PWM_FREQ_HZ        5000                      // 5 kHz
#define PWM_DUTY_RES       LEDC_TIMER_10_BIT         // 10 bits (0–1023)
#define PWM_TIMER          LEDC_TIMER_0
#define PWM_MODE           LEDC_LOW_SPEED_MODE

// LED del ejemplo (usa menuconfig)
#define BLINK_GPIO CONFIG_BLINK_GPIO

// ==============================
// Handles globales
// ==============================

spi_device_handle_t spi_adc;
spi_device_handle_t spi_gpio;

static uint8_t s_led_state = 0;
static QueueHandle_t gpio_evt_queue = NULL;

// pwm

// Pines
static const int pwm_pins[] = {22, 21, 17, 16, 2};
// Canales
static const ledc_channel_t pwm_channels[] = {
    LEDC_CHANNEL_0, LEDC_CHANNEL_1, LEDC_CHANNEL_2,
    LEDC_CHANNEL_3, LEDC_CHANNEL_4
};
static const int pwm_count = sizeof(pwm_pins)/sizeof(pwm_pins[0]);

// ==============================
// LED (dos variantes)
// ==============================

#ifdef CONFIG_BLINK_LED_STRIP
static led_strip_handle_t led_strip;

static void blink_led(void)
{
    if (s_led_state) {
        //led_strip_set_pixel(led_strip, 0, 127, 0, 127); // magenta 50%
        led_strip_set_pixel(led_strip, 0, 64, 0, 64); // magenta 25%
        led_strip_refresh(led_strip);
    } else {
        led_strip_clear(led_strip);
    }
}

static void configure_led(void)
{
    ESP_LOGI(TAG, "Example configured to blink addressable LED!");
    led_strip_config_t strip_config = {
        .strip_gpio_num = BLINK_GPIO,
        .max_leds = 1,
    };
#if CONFIG_BLINK_LED_STRIP_BACKEND_RMT
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000,
        .flags.with_dma = false,
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
#elif CONFIG_BLINK_LED_STRIP_BACKEND_SPI
    led_strip_spi_config_t spi_config = {
        .spi_bus = SPI2_HOST,
        .flags.with_dma = true,
    };
    ESP_ERROR_CHECK(led_strip_new_spi_device(&strip_config, &spi_config, &led_strip));
#else
#error "unsupported LED strip backend"
#endif
    led_strip_clear(led_strip);
}

#elif CONFIG_BLINK_LED_GPIO

static void blink_led(void)
{
    gpio_set_level(BLINK_GPIO, s_led_state);
}

static void configure_led(void)
{
    ESP_LOGI(TAG, "Example configured to blink GPIO LED!");
    gpio_reset_pin(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
}

#else
#error "unsupported LED type"
#endif

// ==============================
// Botones locales (GPIO del ESP32)
// ==============================

static void init_buttons(void) {
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << BUTTON1_GPIO) |
                        (1ULL << BUTTON2_GPIO) |
                        (1ULL << BUTTON3_GPIO) |
                        (1ULL << BUTTON4_GPIO) |
                        (1ULL << BUTTON5_GPIO) |
                        (1ULL << BUTTON6_GPIO),
        .pull_up_en = GPIO_PULLUP_DISABLE,   // estos GPIO no tienen pull-up interno
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
}

// ==============================
// UART1
// ==============================

static void init_uart(void) {
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, UART_BUF_SIZE, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT, UART_TX, UART_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

// ==============================
// SPI bus + dispositivos
// ==============================

static void init_spi_bus(void) {
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_MISO,
        .mosi_io_num = PIN_MOSI,
        .sclk_io_num = PIN_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1
    };
    ESP_ERROR_CHECK(spi_bus_initialize(VSPI_HOST, &buscfg, SPI_DMA_CH_AUTO));

     // Sesgo de MISO en idle (pull-up interno ~45–50 kΩ)
    ESP_ERROR_CHECK(gpio_set_pull_mode(PIN_MISO, GPIO_PULLUP_ONLY));
    // o equivalente:
    // ESP_ERROR_CHECK(gpio_pullup_en(PIN_MISO));
}

static void add_spi_devices(void) {
    // MCP3208
    spi_device_interface_config_t devcfg_adc = {
        .clock_speed_hz = 1 * 1000 * 1000,
        //.clock_speed_hz = 200*1000,
        .mode = 0,
        .spics_io_num = CS_MCP3208,
        .queue_size = 1,
         .cs_ena_posttrans = 2,         // 1–3 ciclos de SCLK suele bastar
        .cs_ena_pretrans  = 1,         // opcional: afirmar CS antes de clockear
    };
    ESP_ERROR_CHECK(spi_bus_add_device(VSPI_HOST, &devcfg_adc, &spi_adc));

    // MCP23S17
    spi_device_interface_config_t devcfg_gpio = {
        .clock_speed_hz = 1 * 1000 * 1000,
        //.clock_speed_hz = 200*1000,
        .mode = 0,
        .spics_io_num = CS_MCP23S17,
        .queue_size = 1,
         .cs_ena_posttrans = 2,         // 1–3 ciclos de SCLK suele bastar
        .cs_ena_pretrans  = 1,         // opcional: afirmar CS antes de clockear
    };
    ESP_ERROR_CHECK(spi_bus_add_device(VSPI_HOST, &devcfg_gpio, &spi_gpio));
}

// ==============================
// MCP3208
// ==============================

static uint16_t read_mcp3208(uint8_t channel) {
    uint8_t tx_data[3] = {
        0x06 | ((channel & 0x04) >> 2),
        (channel & 0x03) << 6,
        0x00
    };
    uint8_t rx_data[3] = {0};

    spi_transaction_t t = {
        .length = 3 * 8,
        .tx_buffer = tx_data,
        .rx_buffer = rx_data,
    };
    ESP_ERROR_CHECK(spi_device_transmit(spi_adc, &t));
    return (uint16_t)(((rx_data[1] & 0x0F) << 8) | rx_data[2]);
}

// ==============================
// MCP23S17 - R/W registro
// ==============================

static void mcp23s17_write_register(uint8_t reg, uint8_t data) {
    uint8_t tx_data[3] = { MCP23S17_OPCODE_WRITE, reg, data };
    spi_transaction_t t = {
        .length = 8 * 3,
        .tx_buffer = tx_data,
        .rx_buffer = NULL,
    };
    ESP_ERROR_CHECK(spi_device_transmit(spi_gpio, &t));
}

static uint8_t mcp23s17_read_register(uint8_t reg) {
    uint8_t tx_data[3] = { MCP23S17_OPCODE_READ, reg, 0x00 };
    uint8_t rx_data[3] = {0};
    spi_transaction_t t = {
        .length = 8 * 3,
        .tx_buffer = tx_data,
        .rx_buffer = rx_data,
    };
    //ESP_ERROR_CHECK(spi_device_polling_transmit(spi_gpio, &t));
    ESP_ERROR_CHECK(spi_device_transmit(spi_gpio, &t));
    return rx_data[2];
}

// ==============================
// MCP23S17 - Configuración
// ==============================

// A = salidas
// B = entradas con pull-up e interrupción por FLANCO DE BAJADA
// INTA/INTB espejadas y open-drain (puedes unirlas físicamente)
static void mcp23s17_config_ab_mirrored(void)
{
    // 1) INT espejada + open-drain + HAEN (SPI)
    mcp23s17_write_register(IOCON, IOCON_MIRROR | IOCON_HAEN | IOCON_ODR); // 0x4C

    // 2) Puerto A -> salidas (estado inicial 0)
    mcp23s17_write_register(IODIRA, 0x00);
    mcp23s17_write_register(OLATA,  0x00);

    // 3) Puerto B -> entradas + pull-ups + IRQ por caída a 0
    const uint8_t maskB = 0xFF;            // ajusta si solo algunos pines interrumpen (p.ej. 0x0F)
    mcp23s17_write_register(IODIRB,   0xFF); // B como entradas
    mcp23s17_write_register(IPOLB,    0x00); // sin invertir lectura
    mcp23s17_write_register(GPPUB,    maskB);// pull-up interno ON en esos pines
    mcp23s17_write_register(INTCONB,  maskB);// comparar contra DEFVAL (no contra valor previo)
    mcp23s17_write_register(DEFVALB,  maskB);// DEFVAL=1 -> al bajar a 0 dispara (flanco de bajada)
    mcp23s17_write_register(GPINTENB, maskB);// habilita la interrupción

    // 4) Limpieza de pendientes iniciales
    (void)mcp23s17_read_register(INTCAPB);   // leer INTCAP limpia IRQ de B
    (void)mcp23s17_read_register(INTCAPA);   // por si acaso
}
// ==============================
// INT del MCP23S17 -> ISR en ESP32
// ==============================

static void IRAM_ATTR isr_mcp_int(void *arg) {
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

// Limpia la interrupción leyendo INTCAP (o GPIO)
static void mcp_clear_irq(void) {
    (void)mcp23s17_read_register(INTCAPA);
    (void)mcp23s17_read_register(INTCAPB);
}

static void task_irq_handler(void *arg) {
    uint32_t io_num;
    for (;;) {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            // Lee qué pines causaron la IRQ (INTF*)

            vTaskDelay(CONFIG_BLINK_PERIOD / portTICK_PERIOD_MS);

            uint8_t intf_a = mcp23s17_read_register(INTFA);
            uint8_t intf_b = mcp23s17_read_register(INTFB);
            uint8_t cap_a  = mcp23s17_read_register(INTCAPA); // leer limpia
            uint8_t cap_b  = mcp23s17_read_register(INTCAPB); // leer limpia

            ESP_LOGI(TAG, "IRQ MCP INTF_A=0x%02X INTF_B=0x%02X CAP_A=0x%02X CAP_B=0x%02X",
                     intf_a, intf_b, cap_a, cap_b);
        }
    }
}

// pwm

// Función para fijar duty en porcentaje (0–100%)
void pwm_set_duty(int channel, int percent)
{
    if (channel < 0 || channel >= pwm_count) return;
    if (percent < 0) percent = 0;
    if (percent > 100) percent = 100;

    uint32_t duty_max = (1 << PWM_DUTY_RES) - 1;   // 1023
    uint32_t duty_val = (percent * duty_max) / 100;

    ESP_ERROR_CHECK(ledc_set_duty(PWM_MODE, pwm_channels[channel], duty_val));
    ESP_ERROR_CHECK(ledc_update_duty(PWM_MODE, pwm_channels[channel]));
}


// ==============================
// app_main
// ==============================

void app_main(void)
{
    init_uart();
    init_spi_bus();
    add_spi_devices();

    gpio_set_drive_capability(PIN_CLK,  GPIO_DRIVE_CAP_1);
    gpio_set_drive_capability(PIN_MOSI, GPIO_DRIVE_CAP_1);
    gpio_set_drive_capability(CS_MCP3208, GPIO_DRIVE_CAP_1);
    gpio_set_drive_capability(CS_MCP23S17, GPIO_DRIVE_CAP_1);


    // Configura MCP23S17: A=salidas, B=entradas+IRQ; INTA/INTB espejo y open-drain
    mcp23s17_config_ab_mirrored();

    // Config INT en el ESP32:
    // NOTA: GPIO39 NO tiene pull-up interno -> usa resistencia externa a 3V3 (~10k).
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << PIN_NUM_MCP_INT,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,   // pull-up externo requerido en GPIO39
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE       // INT open-drain activo en bajo
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    xTaskCreate(task_irq_handler, "mcp_irq_task", 4096, NULL, 10, NULL);
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(PIN_NUM_MCP_INT, isr_mcp_int, (void*) PIN_NUM_MCP_INT));

    ESP_LOGI(TAG, "MCP23S17 listo: A=salidas, B=entradas con IRQ (flanco a 0, MIRROR+ODR).");

    // Inicializa botones locales y LED de ejemplo
    init_buttons();
    configure_led();

    //pwm

        // Configuración del temporizador
    ledc_timer_config_t timer_conf = {
        .speed_mode       = PWM_MODE,
        .duty_resolution  = PWM_DUTY_RES,
        .timer_num        = PWM_TIMER,
        .freq_hz          = PWM_FREQ_HZ,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_conf));

    // Configuración de canales con duty inicial 50%
    for (int i = 0; i < pwm_count; i++) {
        ledc_channel_config_t ch_conf = {
            .gpio_num       = pwm_pins[i],
            .speed_mode     = PWM_MODE,
            .channel        = pwm_channels[i],
            .intr_type      = LEDC_INTR_DISABLE,
            .timer_sel      = PWM_TIMER,
            .duty           = 512,   // 50% duty con 10 bits
            .hpoint         = 0
        };
        ESP_ERROR_CHECK(ledc_channel_config(&ch_conf));
    }

        pwm_set_duty(0, 75); // GPIO22 → 75%
        pwm_set_duty(1, 25); // GPIO21 → 25%
        pwm_set_duty(2, 90); // GPIO17 → 90%
        pwm_set_duty(3, 10); // GPIO16 → 10%
        pwm_set_duty(4, 50); // GPIO2  → 50%

    uint8_t led_state = 0;

    while (1)
    {
        char msg[1024];
        int len = 0;

        ESP_LOGI(TAG, "Turning the LED %s!", s_led_state ? "ON" : "OFF");
        blink_led();
        s_led_state = !s_led_state;
        vTaskDelay(CONFIG_BLINK_PERIOD / portTICK_PERIOD_MS);
        vTaskDelay(pdMS_TO_TICKS(1000));

        // Leer estados de botones físicos (GPIO del ESP32)
        int b1 = gpio_get_level(BUTTON1_GPIO);
        int b2 = gpio_get_level(BUTTON2_GPIO);
        int b3 = gpio_get_level(BUTTON3_GPIO);
        int b4 = gpio_get_level(BUTTON4_GPIO);
        int b5 = gpio_get_level(BUTTON5_GPIO);
        int b6 = gpio_get_level(BUTTON6_GPIO);

        len += snprintf(msg + len, sizeof(msg) - len,
            "Botones GPIO:\r\n"
            "  GPIO36 (B1): %s\r\n"
            "  GPIO34 (B2): %s\r\n"
            "  GPIO35 (B3): %s\r\n"
            "  GPIO32 (B4): %s\r\n"
            "  GPIO33 (B5): %s\r\n"
            "  GPIO02 (B6): %s\r\n",
            b1 ? "SUELTO" : "PRESIONADO",
            b2 ? "PRESIONADO" : "SUELTO",
            b3 ? "PRESIONADO" : "SUELTO",
            b4 ? "PRESIONADO" : "SUELTO",
            b5 ? "PRESIONADO" : "SUELTO",
            b6 ? "PRESIONADO" : "SUELTO"
        );

        // Lecturas MCP3208
        len += snprintf(msg + len, sizeof(msg) - len, "\r\nMCP3208 Readings:\r\n");
        for (uint8_t ch = 0; ch < 8; ch++) {
            uint16_t raw = read_mcp3208(ch);
            float v_adc = (raw / MCP3208_MAX) * VREF;
            float v_input = v_adc * ((R1 + R2) / R2);
            len += snprintf(msg + len, sizeof(msg) - len,
                            "Ch %u: raw=%4u  Vadc=%.2f V  Vin=%.2f V\r\n",
                            ch, raw, v_adc, v_input);
        }

        // Toggle LEDs (GPIOA del MCP23S17)
        led_state ^= 0xFF;
        mcp23s17_write_register(OLATA, led_state);
        vTaskDelay(pdMS_TO_TICKS(1));
         

        // Leer GPIOB (entradas del MCP23S17)
        
        uint8_t inputs = mcp23s17_read_register(GPIOB); 
        inputs = ~inputs;

        len += snprintf(msg + len, sizeof(msg) - len, "GPIOB Inputs: 0x%02X\r\n", inputs);
        //len += snprintf(msg + len, sizeof(msg) - len, "GPIOB Inputs: 0x%02hhX\r\n", inputs);
       // len += snprintf(msg + len, sizeof(msg) - len, "GPIOB Inputs: 0x%02X\r\n", (unsigned)inputs); 

        ESP_LOGI(TAG, "GPIOB Inputs: 0x%02X", inputs); 

        // Enviar por UART
        uart_write_bytes(UART_PORT, msg, strlen(msg));
        //uart_write_bytes(UART_PORT, msg, len);

        
    }
}
