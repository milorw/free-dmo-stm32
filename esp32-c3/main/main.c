#include <ctype.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_netif_ip_addr.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lwip/ip4_addr.h"
#include "nvs_flash.h"
#include "free_dmo_config.h"

#define WIFI_STA_SSID FREE_DMO_CFG_ESP_WIFI_STA_SSID
#define WIFI_STA_PASSWORD FREE_DMO_CFG_ESP_WIFI_STA_PASSWORD
#define WIFI_STATIC_IP FREE_DMO_CFG_ESP_WIFI_STATIC_IP
#define WIFI_STATIC_GATEWAY FREE_DMO_CFG_ESP_WIFI_STATIC_GATEWAY
#define WIFI_STATIC_NETMASK FREE_DMO_CFG_ESP_WIFI_STATIC_NETMASK

#define STM_UART_NUM ((uart_port_t)FREE_DMO_CFG_ESP_STM_UART_NUM)
#define STM_UART_TX_GPIO ((gpio_num_t)FREE_DMO_CFG_ESP_STM_UART_TX_GPIO)
#define STM_UART_RX_GPIO ((gpio_num_t)FREE_DMO_CFG_ESP_STM_UART_RX_GPIO)
#define STM_UART_BAUDRATE FREE_DMO_CFG_ESP_STM_UART_BAUDRATE
#define STM_UART_BUF_SIZE FREE_DMO_CFG_ESP_STM_UART_BUF_SIZE
#define STM_LINE_BUF_SIZE 256

#define STM_BOOT0_GPIO ((gpio_num_t)FREE_DMO_CFG_ESP_STM_BOOT0_GPIO)
#define STM_RESET_GPIO ((gpio_num_t)FREE_DMO_CFG_ESP_STM_RESET_GPIO)

#define STM_FLASH_BASE_ADDR 0x08000000UL
#define STM_FLASH_MAX_SIZE (64U * 1024U)
#define STM_FLASH_CHUNK_SIZE 256U
#define STM32F103C8_CHIP_ID 0x0410U
#define STM_BL_ENTER_SYNC_RETRIES 3

#define STM_BL_ACK 0x79
#define STM_BL_NACK 0x1F
#define STM_BL_SYNC 0x7F
#define STM_BL_CMD_GET_ID 0x02
#define STM_BL_CMD_ERASE 0x43
#define STM_BL_CMD_WRITE_MEM 0x31
#define STM_BL_CMD_GO 0x21

#define LOG_RING_SIZE 16384
#define DEBUG_SYNC_MAX_ATTEMPTS 4

static const char *TAG = "freedmo-bridge";

#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)

static SemaphoreHandle_t g_log_mutex;
static SemaphoreHandle_t g_uart_mutex;

static char g_log_ring[LOG_RING_SIZE];
static size_t g_log_head = 0;
static bool g_log_wrapped = false;

static bool g_debug_enabled = false;
static bool g_debug_sync_pending = false;
static TickType_t g_debug_retry_tick = 0;
static uint8_t g_debug_sync_attempts = 0;
static TickType_t g_stm_boot_seen_tick = 0;
static volatile bool g_flash_in_progress = false;
static bool g_last_flash_ok = false;
static uint32_t g_last_flash_bytes = 0;
static bool g_wifi_connected = false;

static uint32_t g_stm_line_count = 0;
static uint32_t g_stm_byte_count = 0;

static void stm_send_debug_toggle(bool enabled);

static const char INDEX_HTML[] =
    "<!doctype html><html><head><meta charset='utf-8'>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>FreeDMO Debug Console</title>"
    "<style>"
    "body{font-family:ui-monospace,Menlo,Consolas,monospace;background:#10171f;color:#dce7f3;margin:0;padding:20px;}"
    ".wrap{max-width:980px;margin:0 auto;}"
    "h1{margin:0 0 12px 0;font-size:24px;}"
    ".grid{display:grid;grid-template-columns:1fr;gap:12px;}"
    "@media(min-width:900px){.grid{grid-template-columns:1fr 1fr;}}"
    ".card{background:#192330;border:1px solid #2c3a4d;border-radius:10px;padding:12px;}"
    "label{display:block;font-size:12px;color:#9fb2c8;margin-bottom:6px;}"
    "select,input,button{font:inherit;padding:8px;border-radius:8px;border:1px solid #364b64;background:#121c28;color:#dce7f3;}"
    "input[type='file']{width:100%;}"
    "button{cursor:pointer;}"
    "button.primary{background:#1f7a4c;border-color:#28945d;}"
    "button.warn{background:#7a3b1f;border-color:#9a4b28;}"
    "button.flash{background:#21557a;border-color:#2c6f9e;}"
    "#logs{height:360px;overflow:auto;white-space:pre-wrap;background:#0b1118;border:1px solid #2c3a4d;border-radius:8px;padding:10px;}"
    "#status{font-size:13px;color:#9fb2c8;margin-bottom:8px;}"
    "#flashState{font-size:12px;color:#9fb2c8;margin-top:8px;min-height:1em;}"
    "small{color:#9fb2c8;}"
    "</style></head><body><div class='wrap'>"
    "<h1>FreeDMO ESP32-C3 Debug Console</h1>"
    "<div id='status'>Connecting...</div>"
    "<div class='grid'>"
    "<div class='card'><h3>Settings (Scaffold)</h3>"
    "<label>SKU (coming soon)</label><select disabled><option>Not implemented yet</option></select><br><br>"
    "<label>UID/Signature Pair (coming soon)</label><select disabled><option>Not implemented yet</option></select><br><br>"
    "<label>Additional Settings (coming soon)</label><input disabled value='Not implemented yet'><br><br>"
    "<button id='debugBtn' class='warn'>Enable Debug</button> "
    "<button id='randomizeBtn'>Randomize UID/Signature</button> "
    "<a href='/logs' download><button>Download Logs</button></a>"
    "<p><small>When enabled, STM32 debug + I2C trace forwarding will be shown here once STM firmware support is added.</small></p>"
    "<hr style='border-color:#2c3a4d'>"
    "<h3>STM32 Flash</h3>"
    "<label>Firmware (.bin, max 64 KiB)</label><input id='fwFile' type='file' accept='.bin,application/octet-stream'><br><br>"
    "<button id='flashBtn' class='flash'>Flash STM32</button>"
    "<div id='flashState'></div>"
    "<p><small>Wire STM32 `BOOT0` to ESP32 GPIO" STR(FREE_DMO_CFG_ESP_STM_BOOT0_GPIO) " and `NRST` to ESP32 GPIO" STR(FREE_DMO_CFG_ESP_STM_RESET_GPIO) ". UART stays on TX=GPIO" STR(FREE_DMO_CFG_ESP_STM_UART_TX_GPIO) " RX=GPIO" STR(FREE_DMO_CFG_ESP_STM_UART_RX_GPIO) ".</small></p>"
    "</div>"
    "<div class='card'><h3>Live Output</h3><div id='logs'></div></div>"
    "</div></div>"
    "<script>"
    "var statusEl=document.getElementById('status');"
    "var debugBtn=document.getElementById('debugBtn');"
    "var randomizeBtn=document.getElementById('randomizeBtn');"
    "var flashBtn=document.getElementById('flashBtn');"
    "var fwFileEl=document.getElementById('fwFile');"
    "var flashStateEl=document.getElementById('flashState');"
    "var logsEl=document.getElementById('logs');"
    "var debugOn=false;"
    "var flashing=false;"
    "var lastLogs='';"
    "function updateBtn(){debugBtn.textContent=debugOn?'Disable Debug':'Enable Debug';debugBtn.className=debugOn?'primary':'warn';debugBtn.disabled=flashing;randomizeBtn.disabled=flashing;flashBtn.disabled=flashing;}"
    "function setStatus(s){statusEl.textContent=s;}"
    "function setFlashState(s){flashStateEl.textContent=s;}"
    "function refreshState(){fetch('/api/state').then(function(r){return r.json();}).then(function(s){debugOn=!!s.debug;flashing=!!s.flashing;updateBtn();setStatus('Wi-Fi STA: " WIFI_STA_SSID " @ " WIFI_STATIC_IP " | Link: '+(s.wifi?'UP':'DOWN')+' | Debug: '+(debugOn?'ON':'OFF')+' | Flash: '+(flashing?'BUSY':'IDLE')+' | STM lines: '+s.lines+' | bytes: '+s.bytes);if(flashing){setFlashState('Flashing in progress. Do not power-cycle devices.');}else if(s.last_flash_bytes>0){setFlashState('Last flash '+(s.last_flash_ok?'OK':'FAILED')+' ('+s.last_flash_bytes+' bytes).');}}).catch(function(){setStatus('Disconnected from ESP.');});}"
    "function refreshLogs(){fetch('/api/logs').then(function(r){return r.text();}).then(function(t){if(t!==lastLogs){lastLogs=t;logsEl.textContent=t;logsEl.scrollTop=logsEl.scrollHeight;}}).catch(function(){});}" 
    "debugBtn.onclick=function(){var body=debugOn?'0':'1';fetch('/api/debug',{method:'POST',body:body}).then(function(){refreshState();refreshLogs();});};"
    "randomizeBtn.onclick=function(){if(flashing){return;}fetch('/api/randomize',{method:'POST'}).then(function(resp){return resp.text().then(function(text){return{ok:resp.ok,text:text};});}).then(function(result){var obj=null;try{obj=JSON.parse(result.text);}catch(e){}if(result.ok&&obj&&obj.ok){setFlashState('UID/signature randomized on STM.');}else if(obj&&obj.error){setFlashState('Randomize failed: '+obj.error);}else{setFlashState('Randomize failed.');}refreshLogs();}).catch(function(){setFlashState('Randomize request failed.');});};"
    "flashBtn.onclick=function(){if(flashing){return;}if(!fwFileEl.files||!fwFileEl.files.length){setFlashState('Choose a .bin file first.');return;}var file=fwFileEl.files[0];if(file.size>65536){setFlashState('Firmware too large for STM32F103C8 (max 65536 bytes).');return;}setFlashState('Uploading '+file.name+' ('+file.size+' bytes)...');fetch('/api/flash',{method:'POST',headers:{'Content-Type':'application/octet-stream'},body:file}).then(function(resp){return resp.text().then(function(text){return{ok:resp.ok,text:text};});}).then(function(result){var obj=null;try{obj=JSON.parse(result.text);}catch(e){}if(result.ok&&obj&&obj.ok){setFlashState('Flash complete: '+obj.bytes+' bytes.');}else if(obj&&obj.error){setFlashState('Flash failed: '+obj.error);}else{setFlashState('Flash failed.');}refreshState();refreshLogs();}).catch(function(){setFlashState('Flash request failed.');});};"
    "refreshState();refreshLogs();setInterval(refreshState,1000);setInterval(refreshLogs,1000);"
    "</script></body></html>";

static void log_ring_append(const char *text)
{
    if (text == NULL) {
        return;
    }

    const size_t len = strlen(text);
    if (len == 0) {
        return;
    }

    if (xSemaphoreTake(g_log_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        return;
    }

    for (size_t i = 0; i < len; ++i) {
        g_log_ring[g_log_head++] = text[i];
        if (g_log_head >= LOG_RING_SIZE) {
            g_log_head = 0;
            g_log_wrapped = true;
        }
    }

    xSemaphoreGive(g_log_mutex);
}

static size_t log_ring_snapshot(char *out, size_t out_size)
{
    if (out == NULL || out_size == 0) {
        return 0;
    }

    size_t written = 0;
    if (xSemaphoreTake(g_log_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        out[0] = '\0';
        return 0;
    }

    if (g_log_wrapped) {
        for (size_t i = g_log_head; i < LOG_RING_SIZE && written + 1 < out_size; ++i) {
            out[written++] = g_log_ring[i];
        }
    }

    for (size_t i = 0; i < g_log_head && written + 1 < out_size; ++i) {
        out[written++] = g_log_ring[i];
    }

    xSemaphoreGive(g_log_mutex);
    out[written] = '\0';
    return written;
}

static void push_log_line(const char *line)
{
    char timestamped[384];
    snprintf(timestamped, sizeof(timestamped), "[%lu ms] %s\n", (unsigned long)esp_log_timestamp(), line);
    log_ring_append(timestamped);
}

static void push_logf(const char *fmt, ...)
{
    char line[256];
    va_list args;

    va_start(args, fmt);
    vsnprintf(line, sizeof(line), fmt, args);
    va_end(args);

    push_log_line(line);
}

static esp_err_t stm_uart_set_parity(uart_parity_t parity)
{
    const uart_config_t uart_config = {
        .baud_rate = STM_UART_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = parity,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    return uart_param_config(STM_UART_NUM, &uart_config);
}

static void stm_uart_init(void)
{
    ESP_ERROR_CHECK(uart_driver_install(STM_UART_NUM, STM_UART_BUF_SIZE, STM_UART_BUF_SIZE, 0, NULL, 0));
    ESP_ERROR_CHECK(stm_uart_set_parity(UART_PARITY_DISABLE));
    ESP_ERROR_CHECK(uart_set_pin(STM_UART_NUM, STM_UART_TX_GPIO, STM_UART_RX_GPIO,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

static void stm_boot_pins_init(void)
{
    const gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << STM_BOOT0_GPIO) | (1ULL << STM_RESET_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    ESP_ERROR_CHECK(gpio_config(&io_conf));
    ESP_ERROR_CHECK(gpio_set_level(STM_BOOT0_GPIO, 0));
    ESP_ERROR_CHECK(gpio_set_level(STM_RESET_GPIO, 1));
}

static void stm_reset_pulse(void)
{
    gpio_set_level(STM_RESET_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(40));
    gpio_set_level(STM_RESET_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(80));
}

static void stm_enter_bootloader_mode(void)
{
    gpio_set_level(STM_BOOT0_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(25));
    stm_reset_pulse();
}

static void stm_enter_application_mode(void)
{
    gpio_set_level(STM_BOOT0_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    stm_reset_pulse();
}

static esp_err_t stm_bl_send_bytes(const uint8_t *data, size_t len)
{
    const int written = uart_write_bytes(STM_UART_NUM, data, len);
    if (written != (int)len) {
        return ESP_FAIL;
    }

    return uart_wait_tx_done(STM_UART_NUM, pdMS_TO_TICKS(1000));
}

static esp_err_t stm_bl_read_exact(uint8_t *data, size_t len, uint32_t timeout_ms)
{
    size_t total = 0;

    while (total < len) {
        const int got = uart_read_bytes(STM_UART_NUM, data + total, len - total, pdMS_TO_TICKS(timeout_ms));
        if (got <= 0) {
            return ESP_ERR_TIMEOUT;
        }

        total += (size_t)got;
    }

    return ESP_OK;
}

static esp_err_t stm_bl_wait_ack(uint32_t timeout_ms)
{
    uint8_t ack = 0;
    esp_err_t err = stm_bl_read_exact(&ack, 1, timeout_ms);
    if (err != ESP_OK) {
        return err;
    }

    if (ack == STM_BL_ACK) {
        return ESP_OK;
    }

    if (ack == STM_BL_NACK) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    return ESP_ERR_INVALID_RESPONSE;
}

static esp_err_t stm_bl_send_cmd(uint8_t cmd)
{
    uint8_t frame[2];

    frame[0] = cmd;
    frame[1] = (uint8_t)(cmd ^ 0xFFU);

    esp_err_t err = stm_bl_send_bytes(frame, sizeof(frame));
    if (err != ESP_OK) {
        return err;
    }

    return stm_bl_wait_ack(500);
}

static esp_err_t stm_bl_sync(void)
{
    uint8_t sync = STM_BL_SYNC;

    for (int attempt = 0; attempt < 5; ++attempt) {
        uart_flush_input(STM_UART_NUM);

        esp_err_t err = stm_bl_send_bytes(&sync, 1);
        if (err != ESP_OK) {
            continue;
        }

        err = stm_bl_wait_ack(400);
        if (err == ESP_OK) {
            return ESP_OK;
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }

    return ESP_FAIL;
}

static esp_err_t stm_bl_get_chip_id(uint16_t *chip_id)
{
    uint8_t id_len = 0;
    uint8_t id_data[4] = {0};

    esp_err_t err = stm_bl_send_cmd(STM_BL_CMD_GET_ID);
    if (err != ESP_OK) {
        return err;
    }

    err = stm_bl_read_exact(&id_len, 1, 400);
    if (err != ESP_OK) {
        return err;
    }

    const size_t byte_count = (size_t)id_len + 1U;
    if (byte_count != 2U) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    err = stm_bl_read_exact(id_data, byte_count, 400);
    if (err != ESP_OK) {
        return err;
    }

    err = stm_bl_wait_ack(400);
    if (err != ESP_OK) {
        return err;
    }

    *chip_id = ((uint16_t)id_data[0] << 8) | id_data[1];
    return ESP_OK;
}

static esp_err_t stm_bl_global_erase(void)
{
    const uint8_t erase_all[2] = {0xFF, 0x00};

    esp_err_t err = stm_bl_send_cmd(STM_BL_CMD_ERASE);
    if (err != ESP_OK) {
        return err;
    }

    err = stm_bl_send_bytes(erase_all, sizeof(erase_all));
    if (err != ESP_OK) {
        return err;
    }

    return stm_bl_wait_ack(10000);
}

static esp_err_t stm_bl_write_chunk(uint32_t address, const uint8_t *data, size_t len)
{
    if (len == 0 || len > STM_FLASH_CHUNK_SIZE) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t addr_frame[5];
    uint8_t write_frame[1 + STM_FLASH_CHUNK_SIZE + 1];

    esp_err_t err = stm_bl_send_cmd(STM_BL_CMD_WRITE_MEM);
    if (err != ESP_OK) {
        return err;
    }

    addr_frame[0] = (uint8_t)(address >> 24);
    addr_frame[1] = (uint8_t)(address >> 16);
    addr_frame[2] = (uint8_t)(address >> 8);
    addr_frame[3] = (uint8_t)(address >> 0);
    addr_frame[4] = (uint8_t)(addr_frame[0] ^ addr_frame[1] ^ addr_frame[2] ^ addr_frame[3]);

    err = stm_bl_send_bytes(addr_frame, sizeof(addr_frame));
    if (err != ESP_OK) {
        return err;
    }

    err = stm_bl_wait_ack(500);
    if (err != ESP_OK) {
        return err;
    }

    write_frame[0] = (uint8_t)(len - 1U);
    memcpy(&write_frame[1], data, len);

    uint8_t checksum = write_frame[0];
    for (size_t i = 0; i < len; ++i) {
        checksum ^= data[i];
    }
    write_frame[1 + len] = checksum;

    err = stm_bl_send_bytes(write_frame, len + 2U);
    if (err != ESP_OK) {
        return err;
    }

    return stm_bl_wait_ack(1200);
}

static esp_err_t stm_bl_go(uint32_t address)
{
    uint8_t addr_frame[5];

    esp_err_t err = stm_bl_send_cmd(STM_BL_CMD_GO);
    if (err != ESP_OK) {
        return err;
    }

    addr_frame[0] = (uint8_t)(address >> 24);
    addr_frame[1] = (uint8_t)(address >> 16);
    addr_frame[2] = (uint8_t)(address >> 8);
    addr_frame[3] = (uint8_t)(address >> 0);
    addr_frame[4] = (uint8_t)(addr_frame[0] ^ addr_frame[1] ^ addr_frame[2] ^ addr_frame[3]);

    err = stm_bl_send_bytes(addr_frame, sizeof(addr_frame));
    if (err != ESP_OK) {
        return err;
    }

    return stm_bl_wait_ack(1000);
}

static esp_err_t stm_flash_from_request(httpd_req_t *req, size_t image_size, size_t *bytes_written)
{
    uint8_t chunk[STM_FLASH_CHUNK_SIZE + 4];
    size_t total_received = 0;
    size_t total_written = 0;
    uint32_t address = STM_FLASH_BASE_ADDR;
    uint16_t chip_id = 0;
    bool uart_locked = false;
    esp_err_t err = ESP_OK;

    if (xSemaphoreTake(g_uart_mutex, pdMS_TO_TICKS(2500)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    uart_locked = true;

    err = stm_uart_set_parity(UART_PARITY_EVEN);
    if (err != ESP_OK) {
        push_log_line("[flash] failed to set UART parity for bootloader");
        goto cleanup;
    }

    err = ESP_FAIL;
    for (int attempt = 1; attempt <= STM_BL_ENTER_SYNC_RETRIES; ++attempt) {
        stm_enter_bootloader_mode();
        uart_flush_input(STM_UART_NUM);
        vTaskDelay(pdMS_TO_TICKS(20));

        err = stm_bl_sync();
        if (err == ESP_OK) {
            break;
        }

        push_logf("[flash] bootloader sync retry %d/%d", attempt, STM_BL_ENTER_SYNC_RETRIES);
    }
    if (err != ESP_OK) {
        push_log_line("[flash] bootloader sync failed");
        goto cleanup;
    }

    err = stm_bl_get_chip_id(&chip_id);
    if (err != ESP_OK) {
        push_log_line("[flash] could not read STM32 chip ID");
        goto cleanup;
    }

    push_logf("[flash] chip ID: 0x%04X", chip_id);
    if (chip_id != STM32F103C8_CHIP_ID) {
        push_log_line("[flash] unsupported chip ID (expected STM32F103C8T6)");
        err = ESP_ERR_NOT_SUPPORTED;
        goto cleanup;
    }

    push_log_line("[flash] erasing flash");
    err = stm_bl_global_erase();
    if (err != ESP_OK) {
        push_log_line("[flash] erase failed");
        goto cleanup;
    }

    while (total_received < image_size) {
        size_t payload_len = image_size - total_received;
        if (payload_len > STM_FLASH_CHUNK_SIZE) {
            payload_len = STM_FLASH_CHUNK_SIZE;
        }

        size_t got = 0;
        while (got < payload_len) {
            const int rc = httpd_req_recv(req, (char *)(chunk + got), payload_len - got);
            if (rc == HTTPD_SOCK_ERR_TIMEOUT) {
                continue;
            }

            if (rc <= 0) {
                err = ESP_FAIL;
                push_log_line("[flash] upload stream interrupted");
                goto cleanup;
            }

            got += (size_t)rc;
        }

        total_received += got;

        size_t write_len = got;
        while ((write_len % 4U) != 0U) {
            chunk[write_len++] = 0xFF;
        }

        err = stm_bl_write_chunk(address, chunk, write_len);
        if (err != ESP_OK) {
            push_logf("[flash] write failed at 0x%08lX", (unsigned long)address);
            goto cleanup;
        }

        address += (uint32_t)write_len;
        total_written += write_len;

        if ((total_received % 4096U) == 0U || total_received == image_size) {
            push_logf("[flash] progress %lu/%lu bytes", (unsigned long)total_received, (unsigned long)image_size);
        }
    }

    err = stm_bl_go(STM_FLASH_BASE_ADDR);
    if (err != ESP_OK) {
        push_log_line("[flash] GO command failed, forcing hardware reset");
        err = ESP_OK;
    }

cleanup:
    stm_enter_application_mode();
    (void)stm_uart_set_parity(UART_PARITY_DISABLE);
    uart_flush_input(STM_UART_NUM);

    if (uart_locked) {
        xSemaphoreGive(g_uart_mutex);
    }

    if (bytes_written != NULL) {
        *bytes_written = total_written;
    }

    return err;
}

static void stm_send_debug_toggle(bool enabled)
{
    if (xSemaphoreTake(g_uart_mutex, pdMS_TO_TICKS(1500)) != pdTRUE) {
        push_log_line("[esp] debug toggle failed: UART busy");
        return;
    }

    const char *cmd = enabled ? "CMD:DEBUG:1\n" : "CMD:DEBUG:0\n";
    uart_write_bytes(STM_UART_NUM, cmd, strlen(cmd));
    uart_wait_tx_done(STM_UART_NUM, pdMS_TO_TICKS(200));

    xSemaphoreGive(g_uart_mutex);

    push_logf("[esp] sent debug command to STM32: %s", enabled ? "ON" : "OFF");
}

static esp_err_t stm_send_text_command(const char *cmd, const char *log_label)
{
    if (cmd == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (g_flash_in_progress) {
        push_logf("[esp] %s ignored while flashing", log_label != NULL ? log_label : "command");
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(g_uart_mutex, pdMS_TO_TICKS(1500)) != pdTRUE) {
        push_logf("[esp] %s failed: UART busy", log_label != NULL ? log_label : "command");
        return ESP_ERR_TIMEOUT;
    }

    uart_write_bytes(STM_UART_NUM, cmd, strlen(cmd));
    uart_wait_tx_done(STM_UART_NUM, pdMS_TO_TICKS(300));
    xSemaphoreGive(g_uart_mutex);

    if (log_label != NULL) {
        push_logf("[esp] sent %s to STM32", log_label);
    }
    return ESP_OK;
}

static void stm_schedule_debug_sync(TickType_t delay_ticks)
{
    g_debug_sync_pending = true;
    g_debug_retry_tick = xTaskGetTickCount() + delay_ticks;
    g_debug_sync_attempts = 0;
}

static void stm_debug_sync_poll(void)
{
    if (!g_debug_sync_pending || !g_debug_enabled || g_flash_in_progress) {
        return;
    }

    const TickType_t now = xTaskGetTickCount();
    if ((int32_t)(now - g_debug_retry_tick) < 0) {
        return;
    }

    if (g_debug_sync_attempts >= DEBUG_SYNC_MAX_ATTEMPTS) {
        g_debug_sync_pending = false;
        push_log_line("[esp] debug sync failed: no ACK from STM (check ESP->STM UART RX path)");
        return;
    }

    stm_send_debug_toggle(true);
    g_debug_sync_attempts++;
    g_debug_retry_tick = now + pdMS_TO_TICKS(1200);
}

static esp_err_t index_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t logs_get_handler(httpd_req_t *req)
{
    char *snapshot = malloc(LOG_RING_SIZE + 1);
    if (snapshot == NULL) {
        return httpd_resp_send_500(req);
    }

    const size_t bytes = log_ring_snapshot(snapshot, LOG_RING_SIZE + 1);
    httpd_resp_set_type(req, "text/plain; charset=utf-8");
    httpd_resp_set_hdr(req, "Content-Disposition", "attachment; filename=freedmo-debug.log");
    esp_err_t err = httpd_resp_send(req, snapshot, bytes);

    free(snapshot);
    return err;
}

static esp_err_t api_logs_get_handler(httpd_req_t *req)
{
    char *snapshot = malloc(LOG_RING_SIZE + 1);
    if (snapshot == NULL) {
        return httpd_resp_send_500(req);
    }

    const size_t bytes = log_ring_snapshot(snapshot, LOG_RING_SIZE + 1);
    httpd_resp_set_type(req, "text/plain; charset=utf-8");
    esp_err_t err = httpd_resp_send(req, snapshot, bytes);

    free(snapshot);
    return err;
}

static esp_err_t api_state_get_handler(httpd_req_t *req)
{
    char json[224];
    snprintf(json, sizeof(json),
             "{\"debug\":%d,\"lines\":%lu,\"bytes\":%lu,\"flashing\":%d,\"last_flash_ok\":%d,\"last_flash_bytes\":%lu,\"wifi\":%d}",
             g_debug_enabled ? 1 : 0,
             (unsigned long)g_stm_line_count,
             (unsigned long)g_stm_byte_count,
             g_flash_in_progress ? 1 : 0,
             g_last_flash_ok ? 1 : 0,
             (unsigned long)g_last_flash_bytes,
             g_wifi_connected ? 1 : 0);

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, json, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t api_debug_post_handler(httpd_req_t *req)
{
    if (g_flash_in_progress) {
        httpd_resp_set_status(req, "409 Conflict");
        return httpd_resp_send(req, "flash busy", HTTPD_RESP_USE_STRLEN);
    }

    char body[16] = {0};
    const int to_read = req->content_len < (int)sizeof(body) - 1 ? req->content_len : (int)sizeof(body) - 1;
    const int read_len = httpd_req_recv(req, body, to_read);
    if (read_len <= 0) {
        httpd_resp_set_status(req, "400 Bad Request");
        return httpd_resp_send(req, "bad request", HTTPD_RESP_USE_STRLEN);
    }

    body[read_len] = '\0';

    const bool enable = (body[0] == '1' || body[0] == 'o' || body[0] == 'O' || body[0] == 't' || body[0] == 'T');
    g_debug_enabled = enable;
    if (enable) {
        stm_schedule_debug_sync(0);
    } else {
        g_debug_sync_pending = false;
        g_debug_sync_attempts = 0;
        stm_send_debug_toggle(false);
    }

    const char *resp = enable ? "{\"ok\":true,\"debug\":1}" : "{\"ok\":true,\"debug\":0}";
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t api_randomize_post_handler(httpd_req_t *req)
{
    if (g_flash_in_progress) {
        httpd_resp_set_status(req, "409 Conflict");
        return httpd_resp_send(req, "{\"ok\":false,\"error\":\"flash busy\"}", HTTPD_RESP_USE_STRLEN);
    }

    esp_err_t err = stm_send_text_command("CMD:UIDSIG:RANDOM\n", "randomize command");
    httpd_resp_set_type(req, "application/json");
    if (err != ESP_OK) {
        httpd_resp_set_status(req, "503 Service Unavailable");
        return httpd_resp_send(req, "{\"ok\":false,\"error\":\"uart unavailable\"}", HTTPD_RESP_USE_STRLEN);
    }

    return httpd_resp_send(req, "{\"ok\":true}", HTTPD_RESP_USE_STRLEN);
}

static esp_err_t api_flash_post_handler(httpd_req_t *req)
{
    if (g_flash_in_progress) {
        httpd_resp_set_status(req, "409 Conflict");
        return httpd_resp_send(req, "{\"ok\":false,\"error\":\"flash already in progress\"}", HTTPD_RESP_USE_STRLEN);
    }

    if (req->content_len <= 0) {
        httpd_resp_set_status(req, "400 Bad Request");
        return httpd_resp_send(req, "{\"ok\":false,\"error\":\"empty firmware\"}", HTTPD_RESP_USE_STRLEN);
    }

    if (req->content_len > (int)STM_FLASH_MAX_SIZE) {
        httpd_resp_set_status(req, "413 Payload Too Large");
        return httpd_resp_send(req, "{\"ok\":false,\"error\":\"firmware exceeds 64KiB\"}", HTTPD_RESP_USE_STRLEN);
    }

    g_flash_in_progress = true;
    g_last_flash_ok = false;
    g_last_flash_bytes = 0;
    g_debug_enabled = false;
    g_debug_sync_pending = false;

    push_logf("[flash] start upload (%d bytes)", req->content_len);

    size_t written = 0;
    esp_err_t err = stm_flash_from_request(req, (size_t)req->content_len, &written);

    g_flash_in_progress = false;

    httpd_resp_set_type(req, "application/json");

    if (err == ESP_OK) {
        g_last_flash_ok = true;
        g_last_flash_bytes = (uint32_t)req->content_len;

        push_logf("[flash] completed (%lu bytes payload, %lu bytes written)",
                  (unsigned long)req->content_len,
                  (unsigned long)written);

        char json[96];
        snprintf(json, sizeof(json), "{\"ok\":true,\"bytes\":%d}", req->content_len);
        return httpd_resp_send(req, json, HTTPD_RESP_USE_STRLEN);
    }

    push_logf("[flash] failed (err=0x%X)", err);
    httpd_resp_set_status(req, "500 Internal Server Error");
    return httpd_resp_send(req, "{\"ok\":false,\"error\":\"flash failed\"}", HTTPD_RESP_USE_STRLEN);
}

static httpd_handle_t start_webserver(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 12;

    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) != ESP_OK) {
        return NULL;
    }

    const httpd_uri_t index_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = index_get_handler,
    };

    const httpd_uri_t logs_uri = {
        .uri = "/logs",
        .method = HTTP_GET,
        .handler = logs_get_handler,
    };

    const httpd_uri_t api_logs_uri = {
        .uri = "/api/logs",
        .method = HTTP_GET,
        .handler = api_logs_get_handler,
    };

    const httpd_uri_t api_state_uri = {
        .uri = "/api/state",
        .method = HTTP_GET,
        .handler = api_state_get_handler,
    };

    const httpd_uri_t api_debug_uri = {
        .uri = "/api/debug",
        .method = HTTP_POST,
        .handler = api_debug_post_handler,
    };

    const httpd_uri_t api_randomize_uri = {
        .uri = "/api/randomize",
        .method = HTTP_POST,
        .handler = api_randomize_post_handler,
    };

    const httpd_uri_t api_flash_uri = {
        .uri = "/api/flash",
        .method = HTTP_POST,
        .handler = api_flash_post_handler,
    };

    httpd_register_uri_handler(server, &index_uri);
    httpd_register_uri_handler(server, &logs_uri);
    httpd_register_uri_handler(server, &api_logs_uri);
    httpd_register_uri_handler(server, &api_state_uri);
    httpd_register_uri_handler(server, &api_debug_uri);
    httpd_register_uri_handler(server, &api_randomize_uri);
    httpd_register_uri_handler(server, &api_flash_uri);

    return server;
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    (void)arg;
    (void)event_data;

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
        return;
    }

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        g_wifi_connected = false;
        esp_wifi_connect();
        push_log_line("[esp] wifi disconnected; reconnecting");
        return;
    }

    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        g_wifi_connected = true;
        const ip_event_got_ip_t *event = (const ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "STA got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        push_logf("[esp] wifi up: " IPSTR, IP2STR(&event->ip_info.ip));
    }
}

static void wifi_init_sta(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    ESP_ERROR_CHECK(sta_netif != NULL ? ESP_OK : ESP_FAIL);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

    ESP_ERROR_CHECK(esp_netif_dhcpc_stop(sta_netif));
    esp_netif_ip_info_t ip_info = {0};
    ip4_addr_t ip = {0};
    ip4_addr_t gw = {0};
    ip4_addr_t netmask = {0};
    ESP_ERROR_CHECK(ip4addr_aton(WIFI_STATIC_IP, &ip) ? ESP_OK : ESP_ERR_INVALID_ARG);
    ESP_ERROR_CHECK(ip4addr_aton(WIFI_STATIC_GATEWAY, &gw) ? ESP_OK : ESP_ERR_INVALID_ARG);
    ESP_ERROR_CHECK(ip4addr_aton(WIFI_STATIC_NETMASK, &netmask) ? ESP_OK : ESP_ERR_INVALID_ARG);
    ip_info.ip.addr = ip.addr;
    ip_info.gw.addr = gw.addr;
    ip_info.netmask.addr = netmask.addr;
    ESP_ERROR_CHECK(esp_netif_set_ip_info(sta_netif, &ip_info));

    wifi_config_t wifi_config = {0};
    memcpy(wifi_config.sta.ssid, WIFI_STA_SSID, strlen(WIFI_STA_SSID));
    memcpy(wifi_config.sta.password, WIFI_STA_PASSWORD, strlen(WIFI_STA_PASSWORD));
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_config.sta.pmf_cfg.capable = true;
    wifi_config.sta.pmf_cfg.required = false;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
}

static void handle_stm_line(char *line)
{
    if (line == NULL || line[0] == '\0') {
        return;
    }

    g_stm_line_count++;
    g_stm_byte_count += strlen(line);
    push_log_line(line);

    if (strncmp(line, "ACK:DEBUG:1", 11) == 0 && g_debug_enabled) {
        g_debug_sync_pending = false;
        g_debug_sync_attempts = 0;
        return;
    }

    if (strncmp(line, "ACK:DEBUG:0", 11) == 0 && !g_debug_enabled) {
        g_debug_sync_pending = false;
        g_debug_sync_attempts = 0;
        return;
    }

    if (strncmp(line, "DBG:", 4) == 0 && g_debug_enabled) {
        g_debug_sync_pending = false;
        g_debug_sync_attempts = 0;
        return;
    }

    if ((strstr(line, "FREE-DMO STM32 bridge online") != NULL ||
         strstr(line, "INFO:commands") != NULL) &&
        g_debug_enabled) {
        const TickType_t now = xTaskGetTickCount();
        if ((int32_t)(now - g_stm_boot_seen_tick) > pdMS_TO_TICKS(500)) {
            g_stm_boot_seen_tick = now;
            push_log_line("[esp] STM boot detected; re-syncing debug enable");
            stm_schedule_debug_sync(pdMS_TO_TICKS(80));
        }
    }
}

static void stm_rx_task(void *arg)
{
    uint8_t rx_buf[STM_UART_BUF_SIZE];
    char line_buf[STM_LINE_BUF_SIZE];
    size_t line_len = 0;
    (void)arg;

    while (1) {
        if (g_flash_in_progress) {
            vTaskDelay(pdMS_TO_TICKS(40));
            continue;
        }

        if (xSemaphoreTake(g_uart_mutex, pdMS_TO_TICKS(20)) != pdTRUE) {
            vTaskDelay(pdMS_TO_TICKS(8));
            continue;
        }

        const int len = uart_read_bytes(STM_UART_NUM, rx_buf, sizeof(rx_buf), pdMS_TO_TICKS(25));
        xSemaphoreGive(g_uart_mutex);

        for (int i = 0; i < len; ++i) {
            const char c = (char)rx_buf[i];

            if (c == '\r') {
                continue;
            }

            if (c == '\n') {
                if (line_len > 0) {
                    line_buf[line_len] = '\0';
                    handle_stm_line(line_buf);
                    line_len = 0;
                }
                continue;
            }

            if (line_len + 1 >= sizeof(line_buf)) {
                line_buf[line_len] = '\0';
                handle_stm_line(line_buf);
                line_len = 0;
            }

            line_buf[line_len++] = isprint((unsigned char)c) ? c : '.';
        }

        /* Leave scheduler room so web handlers can lock UART for commands. */
        stm_debug_sync_poll();
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    g_log_mutex = xSemaphoreCreateMutex();
    g_uart_mutex = xSemaphoreCreateMutex();
    ESP_ERROR_CHECK(g_log_mutex != NULL ? ESP_OK : ESP_FAIL);
    ESP_ERROR_CHECK(g_uart_mutex != NULL ? ESP_OK : ESP_FAIL);

    stm_uart_init();
    stm_boot_pins_init();
    wifi_init_sta();
    (void)start_webserver();

    ESP_LOGI(TAG, "Web console ready: join '%s' and open http://" WIFI_STATIC_IP, WIFI_STA_SSID);
    push_log_line("[esp] boot complete");
    push_logf("[esp] target wifi ssid: %s", WIFI_STA_SSID);
    push_log_line("[esp] web console available at http://" WIFI_STATIC_IP);
    push_logf("[esp] stm flash pins: BOOT0=GPIO%u NRST=GPIO%u",
              (unsigned)FREE_DMO_CFG_ESP_STM_BOOT0_GPIO,
              (unsigned)FREE_DMO_CFG_ESP_STM_RESET_GPIO);

    const char *hello = "FREE-DMO ESP32-C3 bridge online\n";
    uart_write_bytes(STM_UART_NUM, hello, strlen(hello));

    xTaskCreate(stm_rx_task, "stm_rx_task", 4096, NULL, 5, NULL);
}
