/*
 * Blink + Peripherals Example (ESP32 + MCP3208 + MCP23S17 + PWM + UART)
 *
 * This refactor keeps your original behavior but organizes the code and
 * adds detailed English comments explaining what each part does.
 *
 * Toolchain: ESP-IDF
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

// ==============================
// Logging
// ==============================
static const char *TAG = "example";

// ==============================
// Board Pins & Peripherals
// ==============================
// Local buttons on ESP32 GPIO (note: some are input-only)
#define BUTTON1_GPIO            36
#define BUTTON2_GPIO            34
#define BUTTON3_GPIO            35
#define BUTTON4_GPIO            32
#define BUTTON5_GPIO            33
#define BUTTON6_GPIO            2

// VSPI bus pins
#define PIN_MISO                19
#define PIN_MOSI                23
#define PIN_CLK                 18
#define CS_MCP3208              26
#define CS_MCP23S17             25

// MCP23S17 interrupt pin wired to ESP32
// IMPORTANT: GPIO39 has NO internal pull-up -> use an external ~10k pull-up to 3V3
#define PIN_NUM_MCP_INT         GPIO_NUM_39

// UART1 configuration
#define UART_PORT               UART_NUM_1
#define UART_TX                 4
#define UART_RX                 5
#define UART_BUF_SIZE           1024

// ==============================
// MCP3208 (12-bit ADC) constants
// ==============================
#define MCP3208_MAX             4095.0f
#define MCP3208_VREF            2.048f
// Input divider used in your board: Vin -> R1 -> node -> R2 -> GND, node -> ADC
#define DIVIDER_R1              6200.0f
#define DIVIDER_R2              10000.0f

// ==============================
// MCP23S17 (SPI GPIO expander) constants
// (A2..A0 = 0; change opcodes if using hardware addressing)
// ==============================
#define MCP23S17_OPCODE_WRITE   0x40
#define MCP23S17_OPCODE_READ    0x41

// Register map (BANK=0)
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
#define IOCON    0x0A // (also 0x0B)
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

// IOCON bits (BANK=0)
#define IOCON_BANK   (1<<7)
#define IOCON_MIRROR (1<<6)  // Mirror INTA/INTB (wired-OR)
#define IOCON_SEQOP  (1<<5)
#define IOCON_DISSLW (1<<4)
#define IOCON_HAEN   (1<<3)  // Enable hardware addressing in SPI
#define IOCON_ODR    (1<<2)  // INT open-drain
#define IOCON_INTPOL (1<<1)  // Ignored if ODR=1

// ==============================
// PWM (LEDC) configuration
// ==============================
#define PWM_FREQ_HZ        5000                 // 5 kHz switching
#define PWM_DUTY_RES       LEDC_TIMER_10_BIT    // 10-bit resolution (0..1023)
#define PWM_TIMER          LEDC_TIMER_0
#define PWM_MODE           LEDC_LOW_SPEED_MODE

// ==============================
// Blink LED selection from menuconfig
// ==============================
#define BLINK_GPIO         CONFIG_BLINK_GPIO

// ==============================
// Globals / Handles
// ==============================
static spi_device_handle_t spi_adc;   // MCP3208 handle
static spi_device_handle_t spi_gpio;  // MCP23S17 handle

static volatile uint8_t s_led_state = 0;     // Shared by blink
static QueueHandle_t gpio_evt_queue = NULL;  // Queue from ISR -> task

// PWM pins and channels used
static const int pwm_pins[] = {22, 21, 17, 16, 2};
static const ledc_channel_t pwm_channels[] = {
    LEDC_CHANNEL_0, LEDC_CHANNEL_1, LEDC_CHANNEL_2,
    LEDC_CHANNEL_3, LEDC_CHANNEL_4
};
static const int pwm_count = sizeof(pwm_pins)/sizeof(pwm_pins[0]);

// ==============================
// LED control (two backends)
// ==============================
#ifdef CONFIG_BLINK_LED_STRIP
static led_strip_handle_t led_strip;

// Toggle the first pixel on the addressable strip
static void blink_led(void)
{
    if (s_led_state) {
        // Magenta at ~25%
        led_strip_set_pixel(led_strip, 0, 64, 0, 64);
        led_strip_refresh(led_strip);
    } else {
        led_strip_clear(led_strip);
    }
}

static void configure_led(void)
{
    ESP_LOGI(TAG, "Configured to blink addressable LED");
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

// Simple GPIO LED toggle
static void blink_led(void)
{
    gpio_set_level(BLINK_GPIO, s_led_state);
}

static void configure_led(void)
{
    ESP_LOGI(TAG, "Configured to blink GPIO LED");
    gpio_reset_pin(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
}

#else
#error "unsupported LED type"
#endif

// ==============================
// Local buttons (ESP32 GPIO inputs)
// ==============================
static void init_buttons(void)
{
    // Note: these specific GPIOs do not have internal pull-ups -> use external if needed
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << BUTTON1_GPIO) |
                        (1ULL << BUTTON2_GPIO) |
                        (1ULL << BUTTON3_GPIO) |
                        (1ULL << BUTTON4_GPIO) |
                        (1ULL << BUTTON5_GPIO) |
                        (1ULL << BUTTON6_GPIO),
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
}

// ==============================
// UART
// ==============================
static void init_uart(void)
{
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
// SPI bus + devices
// ==============================
static void init_spi_bus(void)
{
    // Configure VSPI I/O lines
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_MISO,
        .mosi_io_num = PIN_MOSI,
        .sclk_io_num = PIN_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1
    };
    ESP_ERROR_CHECK(spi_bus_initialize(VSPI_HOST, &buscfg, SPI_DMA_CH_AUTO));

    // Bias MISO when idle using internal pull-up (~45–50 kΩ)
    ESP_ERROR_CHECK(gpio_set_pull_mode(PIN_MISO, GPIO_PULLUP_ONLY));
}

static void add_spi_devices(void)
{
    // MCP3208 (12-bit ADC)
    spi_device_interface_config_t devcfg_adc = {
        .clock_speed_hz = 1 * 1000 * 1000, // 1 MHz (safe start)
        .mode = 0,
        .spics_io_num = CS_MCP3208,
        .queue_size = 1,
        .cs_ena_posttrans = 2, // keep CS active for a few cycles after transfer
        .cs_ena_pretrans  = 1, // assert CS slightly before clocking
    };
    ESP_ERROR_CHECK(spi_bus_add_device(VSPI_HOST, &devcfg_adc, &spi_adc));

    // MCP23S17 (GPIO expander)
    spi_device_interface_config_t devcfg_gpio = {
        .clock_speed_hz = 1 * 1000 * 1000, // 1 MHz
        .mode = 0,
        .spics_io_num = CS_MCP23S17,
        .queue_size = 1,
        .cs_ena_posttrans = 2,
        .cs_ena_pretrans  = 1,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(VSPI_HOST, &devcfg_gpio, &spi_gpio));
}

// ==============================
// MCP3208 helpers
// ==============================
static uint16_t read_mcp3208(uint8_t channel)
{
    // Build 3 bytes per MCP3208 datasheet (single-ended read)
    uint8_t tx_data[3] = {
        (uint8_t)(0x06 | ((channel & 0x04) >> 2)), // Start(1) + SGL/DIFF(1) + D2
        (uint8_t)((channel & 0x03) << 6),          // D1 D0 followed by 6 dummy bits
        0x00
    };
    uint8_t rx_data[3] = {0};

    spi_transaction_t t = {
        .length = 3 * 8,
        .tx_buffer = tx_data,
        .rx_buffer = rx_data,
    };
    ESP_ERROR_CHECK(spi_device_transmit(spi_adc, &t));

    // 12-bit result: lower 4 bits from rx_data[1] and all 8 bits from rx_data[2]
    return (uint16_t)(((rx_data[1] & 0x0F) << 8) | rx_data[2]);
}

// ==============================
// MCP23S17 low-level R/W
// ==============================
static void mcp23s17_write_register(uint8_t reg, uint8_t data)
{
    uint8_t tx_data[3] = { MCP23S17_OPCODE_WRITE, reg, data };
    spi_transaction_t t = {
        .length = 8 * 3,
        .tx_buffer = tx_data,
        .rx_buffer = NULL,
    };
    ESP_ERROR_CHECK(spi_device_transmit(spi_gpio, &t));
}

static uint8_t mcp23s17_read_register(uint8_t reg)
{
    uint8_t tx_data[3] = { MCP23S17_OPCODE_READ, reg, 0x00 };
    uint8_t rx_data[3] = {0};
    spi_transaction_t t = {
        .length = 8 * 3,
        .tx_buffer = tx_data,
        .rx_buffer = rx_data,
    };
    ESP_ERROR_CHECK(spi_device_transmit(spi_gpio, &t));
    return rx_data[2];
}

// ==============================
// MCP23S17 configuration
// ==============================
// Port A -> outputs (OLATA drives LEDs)
// Port B -> inputs with pull-up + interrupt on falling edge
// INTA/INTB are mirrored and open-drain (you can wire-OR them)
static void mcp23s17_config_ab_mirrored(void)
{
    // 1) Mirror INT + open-drain + HAEN (for SPI addressing)
    mcp23s17_write_register(IOCON, IOCON_MIRROR | IOCON_HAEN | IOCON_ODR);

    // 2) Port A as outputs, initial 0
    mcp23s17_write_register(IODIRA, 0x00);
    mcp23s17_write_register(OLATA,  0x00);

    // 3) Port B as inputs, with pull-ups, IRQ on transition to 0 (falling edge)
    const uint8_t maskB = 0xFF;            // change mask if only some pins should interrupt
    mcp23s17_write_register(IODIRB,   0xFF);
    mcp23s17_write_register(IPOLB,    0x00); // no inversion
    mcp23s17_write_register(GPPUB,    maskB);// enable pull-ups on masked pins
    mcp23s17_write_register(INTCONB,  maskB);// compare against DEFVAL (not previous value)
    mcp23s17_write_register(DEFVALB,  maskB);// DEFVAL=1 -> falling to 0 triggers interrupt
    mcp23s17_write_register(GPINTENB, maskB);// enable interrupt on those pins

    // 4) Clear any pending flags by reading INTCAP* (or GPIO*)
    (void)mcp23s17_read_register(INTCAPB);
    (void)mcp23s17_read_register(INTCAPA);
}

// ==============================
// MCP23S17 -> ESP32 interrupt handling
// ==============================
static void IRAM_ATTR isr_mcp_int(void *arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

// Reading INTCAP (or GPIO) clears the interrupt
static void mcp_clear_irq(void)
{
    (void)mcp23s17_read_register(INTCAPA);
    (void)mcp23s17_read_register(INTCAPB);
}

static void task_irq_handler(void *arg)
{
    uint32_t io_num;
    for (;;) {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            // Optional debounce delay (uses blink period from menuconfig)
            vTaskDelay(CONFIG_BLINK_PERIOD / portTICK_PERIOD_MS);

            uint8_t intf_a = mcp23s17_read_register(INTFA);
            uint8_t intf_b = mcp23s17_read_register(INTFB);

            
            uint8_t cap_a  = mcp23s17_read_register(INTCAPA); // reading clears
            uint8_t cap_b  = mcp23s17_read_register(INTCAPB);

            ESP_LOGI(TAG, "IRQ MCP INTF_A=0x%02X INTF_B=0x%02X CAP_A=0x%02X CAP_B=0x%02X",
                     intf_a, intf_b, cap_a, cap_b);

            mcp_clear_irq();
        }
    }
}

// ==============================
// PWM helpers (LEDC)
// ==============================
static void pwm_set_duty(int channel, int percent)
{
    if (channel < 0 || channel >= pwm_count) return;
    if (percent < 0) percent = 0;
    if (percent > 100) percent = 100;

    const uint32_t duty_max = (1u << PWM_DUTY_RES) - 1u; // 1023 for 10-bit
    const uint32_t duty_val = (percent * duty_max) / 100u;

    ESP_ERROR_CHECK(ledc_set_duty(PWM_MODE, pwm_channels[channel], duty_val));
    ESP_ERROR_CHECK(ledc_update_duty(PWM_MODE, pwm_channels[channel]));
}

// ==============================
// app_main Loong Socket JTAG & PROG.
// ==============================
void app_main(void)
{
    // --- Basic init
    init_uart();
    init_spi_bus();
    add_spi_devices();

    // Limit drive strength on high-speed lines to reduce ringing/EMI
    gpio_set_drive_capability(PIN_CLK,       GPIO_DRIVE_CAP_1);
    gpio_set_drive_capability(PIN_MOSI,      GPIO_DRIVE_CAP_1);
    gpio_set_drive_capability(CS_MCP3208,    GPIO_DRIVE_CAP_1);
    gpio_set_drive_capability(CS_MCP23S17,   GPIO_DRIVE_CAP_1);

    // --- MCP23S17: A=outputs, B=inputs+IRQ; mirror INTs; open-drain INT
    mcp23s17_config_ab_mirrored();

    // --- Configure INT pin on ESP32 (remember external pull-up on GPIO39)
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << PIN_NUM_MCP_INT,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,   // external pull-up required
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE       // INT is active-low (open-drain)
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    xTaskCreate(task_irq_handler, "mcp_irq_task", 4096, NULL, 10, NULL);
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(PIN_NUM_MCP_INT, isr_mcp_int, (void*) PIN_NUM_MCP_INT));

    ESP_LOGI(TAG, "MCP23S17 ready: A=outputs, B=inputs with IRQ (falling edge, MIRROR+ODR)");

    // --- Local inputs + blink LED
    init_buttons();
    configure_led();

    // --- PWM init: one timer, multiple channels, start at 50% duty
    ledc_timer_config_t timer_conf = {
        .speed_mode       = PWM_MODE,
        .duty_resolution  = PWM_DUTY_RES,
        .timer_num        = PWM_TIMER,
        .freq_hz          = PWM_FREQ_HZ,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_conf));

    for (int i = 0; i < pwm_count; i++) {
        ledc_channel_config_t ch_conf = {
            .gpio_num       = pwm_pins[i],
            .speed_mode     = PWM_MODE,
            .channel        = pwm_channels[i],
            .intr_type      = LEDC_INTR_DISABLE,
            .timer_sel      = PWM_TIMER,
            .duty           = 512,   // 50% @ 10-bit
            .hpoint         = 0
        };
        ESP_ERROR_CHECK(ledc_channel_config(&ch_conf));
    }

    // Example duty updates
    pwm_set_duty(0, 75); // GPIO22 → 75%
    pwm_set_duty(1, 25); // GPIO21 → 25%
    pwm_set_duty(2, 90); // GPIO17 → 90%
    pwm_set_duty(3, 10); // GPIO16 → 10%
    pwm_set_duty(4, 50); // GPIO2  → 50%

    uint8_t gpioa_led_state = 0; // For OLATA pattern toggle

    // ==========================
    // Main loop
    // ==========================
    while (1) {
        char msg[1024];
        int len = 0;

        // Blink (using selected backend)
        ESP_LOGI(TAG, "Turning the LED %s!", s_led_state ? "ON" : "OFF");
        blink_led();
        s_led_state = !s_led_state;
        vTaskDelay(CONFIG_BLINK_PERIOD / portTICK_PERIOD_MS);
        vTaskDelay(pdMS_TO_TICKS(1000));

        // Read local buttons on ESP32
        int b1 = gpio_get_level(BUTTON1_GPIO);
        int b2 = gpio_get_level(BUTTON2_GPIO);
        int b3 = gpio_get_level(BUTTON3_GPIO);
        int b4 = gpio_get_level(BUTTON4_GPIO);
        int b5 = gpio_get_level(BUTTON5_GPIO);
        int b6 = gpio_get_level(BUTTON6_GPIO);

        len += snprintf(msg + len, sizeof(msg) - len,
                        "GPIO Buttons:\r\n"
                        "  GPIO36 (B1): %s\r\n"
                        "  GPIO34 (B2): %s\r\n"
                        "  GPIO35 (B3): %s\r\n"
                        "  GPIO32 (B4): %s\r\n"
                        "  GPIO33 (B5): %s\r\n"
                        "  GPIO02 (B6): %s\r\n",
                        b1 ? "RELEASED" : "PRESSED",
                        b2 ? "PRESSED"  : "RELEASED",
                        b3 ? "PRESSED"  : "RELEASED",
                        b4 ? "PRESSED"  : "RELEASED",
                        b5 ? "PRESSED"  : "RELEASED",
                        b6 ? "PRESSED"  : "RELEASED");

        // Read MCP3208 channels and convert to Vin using your divider
        len += snprintf(msg + len, sizeof(msg) - len, "\r\nMCP3208 Readings:\r\n");
        for (uint8_t ch = 0; ch < 8; ch++) {
            uint16_t raw = read_mcp3208(ch);
            float v_adc = (raw / MCP3208_MAX) * MCP3208_VREF;
            float v_input = v_adc * ((DIVIDER_R1 + DIVIDER_R2) / DIVIDER_R2);
            len += snprintf(msg + len, sizeof(msg) - len,
                            "Ch %u: raw=%4u  Vadc=%.2f V  Vin=%.2f V\r\n",
                            ch, raw, v_adc, v_input);
        }

        // Toggle GPIOA (MCP23S17 outputs) pattern
        gpioa_led_state ^= 0xFF;
        mcp23s17_write_register(OLATA, gpioa_led_state);
        vTaskDelay(pdMS_TO_TICKS(1));

        // Read GPIOB (MCP23S17 inputs). Your original code inverted the bits.
        uint8_t inputs = mcp23s17_read_register(GPIOB);
        inputs = (uint8_t)~inputs;
        len += snprintf(msg + len, sizeof(msg) - len, "GPIOB Inputs: 0x%02X\r\n", inputs);
        ESP_LOGI(TAG, "GPIOB Inputs: 0x%02X", inputs);

        // Send the consolidated status through UART1
        uart_write_bytes(UART_PORT, msg, strlen(msg));
    }
}
