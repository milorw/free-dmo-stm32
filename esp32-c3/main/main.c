#include <stdint.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define STM_UART_NUM UART_NUM_1
#define STM_UART_TX_GPIO GPIO_NUM_4
#define STM_UART_RX_GPIO GPIO_NUM_5
#define STM_UART_BAUDRATE 115200
#define STM_UART_BUF_SIZE 256

static const char *TAG = "freedmo-bridge";

static void stm_uart_init(void)
{
    const uart_config_t uart_config = {
        .baud_rate = STM_UART_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(STM_UART_NUM, STM_UART_BUF_SIZE, STM_UART_BUF_SIZE, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(STM_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(STM_UART_NUM, STM_UART_TX_GPIO, STM_UART_RX_GPIO,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

static void stm_rx_task(void *arg)
{
    uint8_t rx_buf[STM_UART_BUF_SIZE + 1];
    (void)arg;

    while (1) {
        const int len = uart_read_bytes(STM_UART_NUM, rx_buf, STM_UART_BUF_SIZE, pdMS_TO_TICKS(200));
        if (len > 0) {
            rx_buf[len] = '\0';
            ESP_LOGI(TAG, "STM32 -> ESP32 (%d bytes): %s", len, (char *)rx_buf);
        }
    }
}

void app_main(void)
{
    stm_uart_init();
    ESP_LOGI(TAG, "UART bridge scaffold up (UART1 TX=GPIO%d RX=GPIO%d @ %d)",
             STM_UART_TX_GPIO, STM_UART_RX_GPIO, STM_UART_BAUDRATE);

    const char *hello = "FREE-DMO ESP32-C3 bridge online\r\n";
    uart_write_bytes(STM_UART_NUM, hello, strlen(hello));

    xTaskCreate(stm_rx_task, "stm_rx_task", 3072, NULL, 5, NULL);
}
