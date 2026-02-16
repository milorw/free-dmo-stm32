#ifndef FREE_DMO_CONFIG_H
#define FREE_DMO_CONFIG_H

/*
 * Central project configuration for both firmware targets.
 *
 * Edit this file before building to change wiring defaults and boot-time
 * emulation selections.
 */

/* ----------------------------- ESP32-C3 config ---------------------------- */

/*
 * Existing Wi-Fi network credentials used by ESP32 station mode.
 * The ESP joins this network and serves the web UI on the static IP below.
 */
#define FREE_DMO_CFG_ESP_WIFI_STA_SSID     "Loading"
#define FREE_DMO_CFG_ESP_WIFI_STA_PASSWORD "rainer5goodwill!leeds2olympics"

/* Static IPv4 configuration used by ESP32 station mode. */
#define FREE_DMO_CFG_ESP_WIFI_STATIC_IP      "192.168.68.112"
#define FREE_DMO_CFG_ESP_WIFI_STATIC_GATEWAY "192.168.68.1"
#define FREE_DMO_CFG_ESP_WIFI_STATIC_NETMASK "255.255.252.0"

/* UART link from ESP32-C3 to STM32 (ESP side numbering). */
#define FREE_DMO_CFG_ESP_STM_UART_NUM      1
#define FREE_DMO_CFG_ESP_STM_UART_TX_GPIO  4
#define FREE_DMO_CFG_ESP_STM_UART_RX_GPIO  5
#define FREE_DMO_CFG_ESP_STM_UART_BAUDRATE 115200u
#define FREE_DMO_CFG_ESP_STM_UART_BUF_SIZE 256u

/* GPIOs that drive STM32 BOOT0 and NRST for in-circuit flashing. */
#define FREE_DMO_CFG_ESP_STM_BOOT0_GPIO 6
#define FREE_DMO_CFG_ESP_STM_RESET_GPIO 7

/* ------------------------------ STM32 config ------------------------------ */

/*
 * Boot-time default label SKU.
 * Must match one of the entries in stm32/Src/dmo_skus.c (e.g. "30333").
 */
#define FREE_DMO_CFG_STM_DEFAULT_SKU_NAME "30333"

/*
 * UID/signature pair selection mode at boot:
 * 1 = rotate/random using stored next-index logic
 * 0 = always use FREE_DMO_CFG_STM_UID_SIG_FIXED_INDEX
 */
#define FREE_DMO_CFG_STM_UID_SIG_MODE_RANDOM 1

/*
 * Fixed UID/signature pair index when random mode is disabled.
 * Valid range: 0 .. (DMO_TAG_EMU_PAIRS_COUNT - 1)
 */
#define FREE_DMO_CFG_STM_UID_SIG_FIXED_INDEX 0u

/* STM32 USART1 link speed for ESP command/debug channel. */
#define FREE_DMO_CFG_STM_ESP_UART_BAUDRATE 115200u

/*
 * Default STM32 debug forwarding state at boot:
 * 1 = emit debug logs immediately (no CMD:DEBUG needed)
 * 0 = wait for CMD:DEBUG:1 from ESP
 */
#define FREE_DMO_CFG_STM_DEBUG_DEFAULT_ENABLED 0

#endif /* FREE_DMO_CONFIG_H */
