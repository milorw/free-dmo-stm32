#ifndef RFID_SCANNER_H
#define RFID_SCANNER_H

#include <stdbool.h>
#include <stdint.h>

#include "stm32f1xx_hal.h"

bool RFID_Scanner_UpdateFromRealTag(
  I2C_HandleTypeDef* phi2c,
  uint8_t* emu_inventory,
  uint8_t inventory_len,
  uint8_t* emu_sysinfo,
  uint8_t sysinfo_len,
  uint8_t* emu_nxpsysinfo,
  uint8_t nxpsysinfo_len,
  uint8_t* emu_signature,
  uint8_t signature_len,
  uint32_t* emu_blocks,
  uint8_t blocks_count,
  bool* p_tag_present
);

#endif /* RFID_SCANNER_H */
