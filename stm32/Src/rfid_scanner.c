#include "rfid_scanner.h"

#include "main.h"

#include <string.h>

#define I2C_TIMEOUT 100
#define CLRC688_ADDR (0x28 << 1)

#define DMO_MAGIC_0 0xed820a03u
#define DMO_MAGIC_1 0xd2613986u
#define DMO_MAGIC_2 0x321e1403u
#define DMO_MAGIC_3 0x3c00cab6u

static bool CLRC688_WriteRegister(I2C_HandleTypeDef* phi2c, const uint8_t addr, const uint8_t reg, const uint8_t val) {
  return( HAL_OK == HAL_I2C_Mem_Write(phi2c, addr, reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&val, sizeof(uint8_t), I2C_TIMEOUT) );
}

static bool CLRC688_ReadRegister(I2C_HandleTypeDef* phi2c, const uint8_t addr, const uint8_t reg, uint8_t* pval) {
  return( HAL_OK == HAL_I2C_Mem_Read(phi2c, addr, reg, I2C_MEMADD_SIZE_8BIT, pval, sizeof(uint8_t), I2C_TIMEOUT) );
}

static bool CLRC688_Transceive(I2C_HandleTypeDef* phi2c, const uint8_t addr, const uint8_t* send, const uint8_t sendlen, uint8_t* recv, uint8_t* precvlen) {
  if( 
      !CLRC688_WriteRegister(phi2c, addr, 0x0F,0x11) ||
      !CLRC688_WriteRegister(phi2c, addr, 0x10,0xFF) ||
      !CLRC688_WriteRegister(phi2c, addr, 0x11,0xFF) ||
      !CLRC688_WriteRegister(phi2c, addr, 0x08,0x04) ||
      !CLRC688_WriteRegister(phi2c, addr, 0x09,0x41) ||
      !CLRC688_WriteRegister(phi2c, addr, 0x06,0x7F) ||
      !CLRC688_WriteRegister(phi2c, addr, 0x07,0x7F) ||
      !CLRC688_WriteRegister(phi2c, addr, 0x00,0x00) ||
      !CLRC688_WriteRegister(phi2c, addr, 0x02,0x10)
    ) {
    return false;
  }

  if( HAL_OK != HAL_I2C_Mem_Write(phi2c, addr, 0x05, I2C_MEMADD_SIZE_8BIT, (uint8_t*)send, sendlen, I2C_TIMEOUT) )
    return false;

  if( !recv || !precvlen)
    return true;

  if( !CLRC688_WriteRegister(phi2c, addr, 0x00,0x07) )
    return false;

  for( uint32_t ticks = HAL_GetTick() + I2C_TIMEOUT; ticks>HAL_GetTick(); ) {
    uint8_t r7;
    if( !CLRC688_ReadRegister(phi2c, addr, 0x07, &r7) )
      return false;

    if( r7 & 0x40 )
      break;

    HAL_Delay(10);
  }

  uint8_t r0A;
  if( !CLRC688_ReadRegister(phi2c, addr, 0x0A, &r0A) || (0 != r0A) )
    return false;

  uint8_t r04;
  if( !CLRC688_ReadRegister(phi2c, addr, 0x04, &r04) )
    return false;

  if( r04 > (uint8_t)(*precvlen+1) )
    return false;

  uint8_t tmp[256] = {0xFF};
  if( (HAL_OK == HAL_I2C_Mem_Read(phi2c, addr, 0x05, I2C_MEMADD_SIZE_8BIT, tmp, r04, I2C_TIMEOUT)) && (0x00==tmp[0]) ) {
    if( r04>1 )
      memcpy(recv, tmp+1, r04-1);
    *precvlen = r04-1;
    return true;
  }

  *precvlen=0;
  return false;
}

static bool CLRC688_Init(I2C_HandleTypeDef* phi2c, const uint8_t addr) {
  HAL_GPIO_WritePin(OUT_PWDN_READER_GPIO_Port, OUT_PWDN_READER_Pin, GPIO_PIN_SET);
  HAL_Delay(50);
  HAL_GPIO_WritePin(OUT_PWDN_READER_GPIO_Port, OUT_PWDN_READER_Pin, GPIO_PIN_RESET);
  HAL_Delay(100);

  const uint8_t zerobuf[] = {0x00,0x00};

  if( !CLRC688_WriteRegister(phi2c, addr, 0x43,0x40) ||
      !CLRC688_WriteRegister(phi2c, addr, 0x02,0x10) ||
      !CLRC688_WriteRegister(phi2c, addr, 0x03,0xFE) ||
      !CLRC688_WriteRegister(phi2c, addr, 0x0C,0x80) ||
      !CLRC688_WriteRegister(phi2c, addr, 0x28,0x80) ||
      !CLRC688_WriteRegister(phi2c, addr, 0x2A,0x01) ||
      !CLRC688_WriteRegister(phi2c, addr, 0x2B,0x05) ||
      !CLRC688_WriteRegister(phi2c, addr, 0x34,0x00) ||
      !CLRC688_WriteRegister(phi2c, addr, 0x38,0x12) ||
      !CLRC688_Transceive(phi2c, addr, zerobuf, sizeof(zerobuf), NULL, NULL) ||
      !CLRC688_WriteRegister(phi2c, addr, 0x00,0x0D) ||
      !CLRC688_WriteRegister(phi2c, addr, 0x2C,0x7B) ||
      !CLRC688_WriteRegister(phi2c, addr, 0x2D,0x7B) ||
      !CLRC688_WriteRegister(phi2c, addr, 0x2E,0x08) ||
      !CLRC688_WriteRegister(phi2c, addr, 0x2F,0x00) ||
      !CLRC688_WriteRegister(phi2c, addr, 0x30,0x00) ||
      !CLRC688_WriteRegister(phi2c, addr, 0x31,0x00) ||
      !CLRC688_WriteRegister(phi2c, addr, 0x33,0x0F) ||
      !CLRC688_WriteRegister(phi2c, addr, 0x35,0x02) ||
      !CLRC688_WriteRegister(phi2c, addr, 0x37,0x4E) ||
      !CLRC688_WriteRegister(phi2c, addr, 0x39,0x07) ||
      !CLRC688_WriteRegister(phi2c, addr, 0x36,0x8C) ||
      !CLRC688_WriteRegister(phi2c, addr, 0x31,0xC0) ||
      !CLRC688_WriteRegister(phi2c, addr, 0x32,0x00) ||
      !CLRC688_WriteRegister(phi2c, addr, 0x29,0x10) ||
      !CLRC688_WriteRegister(phi2c, addr, 0x28,0x81) ||
      !CLRC688_WriteRegister(phi2c, addr, 0x0B,0x00) ||
      !CLRC688_WriteRegister(phi2c, addr, 0x28,0x89) ||
      !CLRC688_WriteRegister(phi2c, addr, 0x00,0x40) ||
      !CLRC688_WriteRegister(phi2c, addr, 0x00,0x00) 
    ) {
    HAL_GPIO_WritePin(OUT_PWDN_READER_GPIO_Port, OUT_PWDN_READER_Pin, GPIO_PIN_SET);
    return false;
  }

  return true;
}

static void CLRC688_DeInit(void) {
  HAL_GPIO_WritePin(OUT_PWDN_READER_GPIO_Port, OUT_PWDN_READER_Pin, GPIO_PIN_SET);
}

static bool CLRC688_ReadOutSLIX2(
  I2C_HandleTypeDef* phi2c,
  const uint8_t addr,
  uint8_t* inventory,
  const uint8_t inventory_len,
  uint8_t* sysinfo,
  const uint8_t sysinfo_len,
  uint8_t* nxpsysinfo,
  const uint8_t nxpsysinfo_len,
  uint8_t* signature,
  const uint8_t signature_len,
  uint32_t* blocks,
  const uint8_t blocks_count
) {
  if( inventory_len < 9 )
    return false;

  static const uint8_t inventorycmd[] = {0x36,0x01,0x00,0x00};
  uint8_t inventory_recv_len = inventory_len;
  if( !CLRC688_Transceive(phi2c, addr, inventorycmd, sizeof(inventorycmd), inventory, &inventory_recv_len) || (inventory_recv_len<inventory_len) )
    return false;

  uint8_t uid[] = {inventory[1],inventory[2],inventory[3],inventory[4],inventory[5],inventory[6],inventory[7],inventory[8]};

  uint8_t sysinfocmd[] = {0x22,0x2B,uid[0],uid[1],uid[2],uid[3],uid[4],uid[5],uid[6],uid[7]};
  uint8_t sysinfo_recv_len = sysinfo_len;
  if( !CLRC688_Transceive(phi2c, addr, sysinfocmd, sizeof(sysinfocmd), sysinfo, &sysinfo_recv_len) || (sysinfo_recv_len<sysinfo_len) )
    return false;

  uint8_t nxpsysinfocmd[] = {0x22,0xAB,0x04,uid[0],uid[1],uid[2],uid[3],uid[4],uid[5],uid[6],uid[7]};
  uint8_t nxpsysinfo_recv_len = nxpsysinfo_len;
  if( !CLRC688_Transceive(phi2c, addr, nxpsysinfocmd, sizeof(nxpsysinfocmd), nxpsysinfo, &nxpsysinfo_recv_len) || (nxpsysinfo_recv_len<nxpsysinfo_len) )
    return false;

  uint8_t signaturecmd[] = {0x22,0xBD,0x04,uid[0],uid[1],uid[2],uid[3],uid[4],uid[5],uid[6],uid[7]};
  uint8_t signature_recv_len = signature_len;
  if( !CLRC688_Transceive(phi2c, addr, signaturecmd, sizeof(signaturecmd), signature, &signature_recv_len) || (signature_recv_len<signature_len) )
    return false;

  for(uint8_t block=0; block<blocks_count; block+=20) {
    uint8_t chunk_blocks = (uint8_t)((blocks_count-block)>=20 ? 20 : (blocks_count-block));
    uint8_t readblockscmd[] = {0x22,0x23,uid[0],uid[1],uid[2],uid[3],uid[4],uid[5],uid[6],uid[7],block,(uint8_t)(chunk_blocks-1)};
    uint8_t readblockslen = (uint8_t)(chunk_blocks*sizeof(uint32_t));
    if( !CLRC688_Transceive(phi2c, addr, readblockscmd, sizeof(readblockscmd), (uint8_t*)&blocks[block], &readblockslen) || (readblockslen!=(uint8_t)(chunk_blocks*sizeof(uint32_t))) )
      return false;
  }

  return true;
}

static bool CLRC688_CheckPresense(I2C_HandleTypeDef* phi2c, const uint8_t addr, const uint8_t* inventory, const uint8_t inventory_len) {
  if( inventory_len < 9 )
    return false;

  uint8_t uid[] = {inventory[1],inventory[2],inventory[3],inventory[4],inventory[5],inventory[6],inventory[7],inventory[8]};
  uint8_t resettoreadycmd[] = {0x22,0x26,uid[0],uid[1],uid[2],uid[3],uid[4],uid[5],uid[6],uid[7]};
  uint8_t resettoreadylen = 0;
  return( CLRC688_Transceive(phi2c, addr, resettoreadycmd, sizeof(resettoreadycmd), (uint8_t*)inventory, &resettoreadylen) );
}

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
) {
  if( !phi2c || !emu_inventory || !emu_sysinfo || !emu_nxpsysinfo || !emu_signature || !emu_blocks || (blocks_count < 4) )
    return false;

  bool updated = false;

  if( CLRC688_Init(phi2c, CLRC688_ADDR) ) {
    uint8_t inventory[inventory_len];
    uint8_t sysinfo[sysinfo_len];
    uint8_t nxpsysinfo[nxpsysinfo_len];
    uint8_t signature[signature_len];
    uint32_t blocks[blocks_count];

    if( CLRC688_ReadOutSLIX2(phi2c, CLRC688_ADDR, inventory, inventory_len, sysinfo, sysinfo_len, nxpsysinfo, nxpsysinfo_len, signature, signature_len, blocks, blocks_count) ) {
      if( (DMO_MAGIC_0==blocks[0]) && (DMO_MAGIC_1==blocks[1]) && (DMO_MAGIC_2==blocks[2]) && (DMO_MAGIC_3==blocks[3]) ) {
        memcpy(emu_inventory, inventory, inventory_len);
        memcpy(emu_sysinfo, sysinfo, sysinfo_len);
        memcpy(emu_nxpsysinfo, nxpsysinfo, nxpsysinfo_len);
        memcpy(emu_signature, signature, signature_len);
        memcpy(emu_blocks, blocks, blocks_count*sizeof(uint32_t));
        if( p_tag_present )
          *p_tag_present = false;
        updated = true;
      }

      for(uint32_t notpresent=0; notpresent<10;) {
        HAL_Delay(100);
        if( !CLRC688_CheckPresense(phi2c, CLRC688_ADDR, inventory, inventory_len) )
          notpresent++;
        else
          notpresent=0;
      }
    }

    CLRC688_DeInit();
  }

  return updated;
}
