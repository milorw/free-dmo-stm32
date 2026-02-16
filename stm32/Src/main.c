/* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "dmo_skus.h"
#include "rfid_scanner.h"
#include "tag_emu_signature.h"

#define SLIX2_BLOCKS         80
#define SLIX2_INVENTORY_LEN   9
#define SLIX2_SYSINFO_LEN    14
#define SLIX2_NXPSYSINFO_LEN  7
#define SLIX2_SIGNATURE_LEN  32

/////////////////////////////////////////////////////////
// DMO emulation data (default emulation after power up)
//

// default emulated SKU selected at runtime from dmo_skus.c
#define DEFAULT_DMO_SKU_NAME "30333"
static const bool rfid_scanner_attached = false;

static const uint8_t DMO_TAG_SLIX2_SYSINFO_TEMPLATE[SLIX2_SYSINFO_LEN] = {
  0x0F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x3D,0x4F,0x03,0x01
};

static const uint8_t DMO_TAG_SLIX2_NXPSYSINFO[SLIX2_NXPSYSINFO_LEN] = {0x32,0x02,0x0F,0x7F,0x35,0x00,0x00};


/////////////////////////////////////////////////////////
// I2C emulation of CLRC688 + SLIX2 tag
// (with auto reset counter on power down signal)
//

static uint8_t  EMU_SLIX2_INVENTORY[SLIX2_INVENTORY_LEN];
static uint8_t  EMU_SLIX2_SYSINFO[SLIX2_SYSINFO_LEN];
static uint8_t  EMU_SLIX2_NXPSYSINFO[SLIX2_NXPSYSINFO_LEN];
static uint8_t  EMU_SLIX2_SIGNATURE[SLIX2_SIGNATURE_LEN];
static uint32_t EMU_SLIX2_BLOCKS[SLIX2_BLOCKS];
static uint16_t EMU_SLIX2_COUNTER;
static bool     EMU_SLIX2_TAG_PRESENT;

// Flash storage for selected tag pair index (append-only records in reserved last flash page).
#define EMU_TAG_PAIR_FLASH_PAGE_SIZE      1024u
#define EMU_TAG_PAIR_FLASH_BASE_ADDR      0x08000000u
#define EMU_TAG_PAIR_FLASH_EMPTY_RECORD   0xFFFFFFFFu
#define EMU_TAG_PAIR_FLASH_RECORD_MAGIC   0xA5u
#define EMU_TAG_PAIR_FLASH_RECORD_TAIL    0x5Au

static uint32_t EMU_TagPairPrngNext(void) {
  static uint32_t state = 0;
  if(0 == state) {
    state = HAL_GetTick() ^ SysTick->VAL ^ (uint32_t)(uintptr_t)&state;
#ifdef UID_BASE
    state ^= *(uint32_t*)(UID_BASE + 0);
    state ^= *(uint32_t*)(UID_BASE + 4);
    state ^= *(uint32_t*)(UID_BASE + 8);
#endif
    if(0 == state)
      state = 0x9E3779B9u;
  }

  state ^= state << 13;
  state ^= state >> 17;
  state ^= state << 5;
  return state;
}

static uint32_t EMU_TagPairFlashPageAddress(void) {
  uint16_t flash_kb = *(uint16_t*)FLASHSIZE_BASE;
  if( (0 == flash_kb) || (0xFFFFu == flash_kb) )
    flash_kb = 64;

  uint32_t flash_size = (uint32_t)flash_kb * 1024u;
  return (EMU_TAG_PAIR_FLASH_BASE_ADDR + flash_size - EMU_TAG_PAIR_FLASH_PAGE_SIZE);
}

static uint32_t EMU_TagPairMakeFlashRecord(const uint8_t next_index) {
  return (((uint32_t)EMU_TAG_PAIR_FLASH_RECORD_MAGIC) << 24) |
         (((uint32_t)next_index) << 16) |
         (((uint32_t)(uint8_t)(~next_index)) << 8) |
         ((uint32_t)EMU_TAG_PAIR_FLASH_RECORD_TAIL);
}

static bool EMU_TagPairParseFlashRecord(const uint32_t record, uint8_t* p_next_index) {
  if( EMU_TAG_PAIR_FLASH_EMPTY_RECORD == record )
    return false;

  if( ((uint8_t)(record >> 24) != EMU_TAG_PAIR_FLASH_RECORD_MAGIC) ||
      ((uint8_t)(record >> 0)  != EMU_TAG_PAIR_FLASH_RECORD_TAIL) )
    return false;

  uint8_t next = (uint8_t)(record >> 16);
  uint8_t inv = (uint8_t)(record >> 8);
  if( (uint8_t)(~next) != inv )
    return false;

  if( next >= DMO_TAG_EMU_PAIRS_COUNT )
    return false;

  if( NULL != p_next_index )
    *p_next_index = next;

  return true;
}

static bool EMU_TagPairLoadNextIndex(uint8_t* p_index) {
  if( NULL == p_index )
    return false;

  uint32_t page_addr = EMU_TagPairFlashPageAddress();
  uint32_t slots = EMU_TAG_PAIR_FLASH_PAGE_SIZE / sizeof(uint32_t);

  bool found = false;
  uint8_t last_next = 0;

  for(uint32_t i=0; i<slots; i++) {
    uint32_t record = *(volatile uint32_t*)(page_addr + i*sizeof(uint32_t));
    if( EMU_TAG_PAIR_FLASH_EMPTY_RECORD == record )
      break;

    uint8_t next = 0;
    if( EMU_TagPairParseFlashRecord(record, &next) ) {
      last_next = next;
      found = true;
    }
  }

  if( found ) {
    *p_index = last_next;
    return true;
  }

  return false;
}

static bool EMU_TagPairStoreNextIndex(const uint8_t used_index) {
  uint8_t next_index = (uint8_t)((used_index + 1u) % DMO_TAG_EMU_PAIRS_COUNT);
  uint32_t new_record = EMU_TagPairMakeFlashRecord(next_index);
  uint32_t page_addr = EMU_TagPairFlashPageAddress();
  uint32_t slots = EMU_TAG_PAIR_FLASH_PAGE_SIZE / sizeof(uint32_t);

  uint32_t write_addr = 0;
  bool slot_found = false;
  for(uint32_t i=0; i<slots; i++) {
    uint32_t addr = page_addr + i*sizeof(uint32_t);
    if( EMU_TAG_PAIR_FLASH_EMPTY_RECORD == *(volatile uint32_t*)addr ) {
      write_addr = addr;
      slot_found = true;
      break;
    }
  }

  HAL_FLASH_Unlock();

  if( !slot_found ) {
    FLASH_EraseInitTypeDef erase = {
      .TypeErase = FLASH_TYPEERASE_PAGES,
      .PageAddress = page_addr,
      .NbPages = 1
    };
    uint32_t page_error = 0;
    if( HAL_OK != HAL_FLASHEx_Erase(&erase, &page_error) ) {
      HAL_FLASH_Lock();
      return false;
    }
    write_addr = page_addr;
  }

  if( HAL_OK != HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, write_addr, new_record) ) {
    HAL_FLASH_Lock();
    return false;
  }

  HAL_FLASH_Lock();
  return true;
}

static uint8_t EMU_TagPairChooseBootIndex(void) {
  uint8_t index = 0;
  if( EMU_TagPairLoadNextIndex(&index) )
    return index;

  return (uint8_t)(EMU_TagPairPrngNext() % DMO_TAG_EMU_PAIRS_COUNT);
}

static void EMU_SLIX2_SyncSysInfoWithInventoryUID(void) {
  memcpy(EMU_SLIX2_SYSINFO, DMO_TAG_SLIX2_SYSINFO_TEMPLATE, SLIX2_SYSINFO_LEN);
  memcpy(EMU_SLIX2_SYSINFO + 1, EMU_SLIX2_INVENTORY + 1, 8);
}

void EMU_SLIX2_CounterReset() {
  uint16_t amount_of_labels = EMU_SLIX2_BLOCKS[0x0F]>>16;
  uint16_t counter_margin = EMU_SLIX2_BLOCKS[0x10]>>16;
  EMU_SLIX2_COUNTER = 0xFFFF - amount_of_labels - counter_margin;
}

void EMU_SLIX2_Communication(const uint8_t* pindata, const uint8_t inlength, uint8_t* poutdata, uint8_t* poutlength) {
  switch(pindata[1]) { //emulate command for tag
    case 0x01: { //inventory
      EMU_SLIX2_CounterReset();
      poutdata[0]=0;
      memcpy(poutdata+1, EMU_SLIX2_INVENTORY, SLIX2_INVENTORY_LEN);
      *poutlength = 1 + SLIX2_INVENTORY_LEN;
      EMU_SLIX2_TAG_PRESENT = true;
      break;
    }
    case 0x21: { //write single block
      if( 0x4F == pindata[10] ) { //write counter
        if( (1==pindata[11]) && (0==pindata[12]) && (0==pindata[13]) && (0==pindata[14]) ) { //only allow increment by 1
          EMU_SLIX2_COUNTER++;
        }
        else {
          poutdata[0]=0x01; poutdata[0]=0x0F; *poutlength=2; //set error
          break;
        }
      }
      poutdata[0]=0; *poutlength=1; 
      break;
    }
    case 0x23: { //read multiple block
      uint8_t blk = pindata[10];
      if( blk >= SLIX2_BLOCKS ) {
        poutdata[0]=0x01; poutdata[0]=0x0F; *poutlength=2; //set error
        break;
      }
      uint8_t cnt = pindata[11]+1;
      if( blk+cnt > SLIX2_BLOCKS )
        cnt = SLIX2_BLOCKS-blk;
      poutdata[0]=0;
      memcpy(poutdata+1, &EMU_SLIX2_BLOCKS[blk], cnt*4);
      *poutlength = 1 + cnt*4;

      //special case counter
      if( (79==blk) || (blk+cnt>=79) ) {
        poutdata[1 + (79-blk)*4 + 0] = (uint8_t)EMU_SLIX2_COUNTER;
        poutdata[1 + (79-blk)*4 + 1] = (uint8_t)(EMU_SLIX2_COUNTER>>8);
        poutdata[1 + (79-blk)*4 + 2] = 0;
        poutdata[1 + (79-blk)*4 + 3] = 1;
      }
      break;
    }
    case 0x26: { //reset to ready (used to check if tag still present)
      if( EMU_SLIX2_TAG_PRESENT ) {
        poutdata[0]=0; *poutlength=1; 
      } else {
        poutdata[0]=0x01; poutdata[0]=0x0F; *poutlength=2; //set error
      }
      break;
    }
    case 0x2B: poutdata[0]=0; memcpy(poutdata+1, EMU_SLIX2_SYSINFO, SLIX2_SYSINFO_LEN); *poutlength=1+SLIX2_SYSINFO_LEN; break;                          //sysinfo
    case 0xAB: poutdata[0]=0; memcpy(poutdata+1, EMU_SLIX2_NXPSYSINFO, SLIX2_NXPSYSINFO_LEN); *poutlength=1+SLIX2_NXPSYSINFO_LEN; break;                 //nxp sysinfo
    case 0xB2: poutdata[0]=0; poutdata[1]=HAL_GetTick(); poutdata[2]=HAL_GetTick()>>8; *poutlength=3; break;                                             //random
    case 0xB3: poutdata[0]=0; *poutlength=1; break;                                                                                                      //set password => just signal success
    case 0xBD: poutdata[0]=0; memcpy(poutdata+1, EMU_SLIX2_SIGNATURE, SLIX2_SIGNATURE_LEN); *poutlength=1+SLIX2_SIGNATURE_LEN; break;                    //signature
    default:   poutdata[0]=0; *poutlength=1; break;                                                                                                      //always signal success
  }
}

void EMU_CLRC688_IRQSet(uint8_t irq) {
  HAL_GPIO_WritePin(OUT_IRQ_GPIO_Port, OUT_IRQ_Pin, irq?GPIO_PIN_SET:GPIO_PIN_RESET);
}

void EMU_CLRC688_Communication(const uint8_t* pindata, const uint8_t inlength, uint8_t* poutdata, uint8_t* poutlength) {
  static uint8_t clrc668_fifo_buffer[256];
  static uint8_t clrc668_fifo_length = 0;

  *poutlength = 0;
  if( inlength>0 ) {
    switch( pindata[0] ) {
      case 0x00: { //command for reader ic
        switch( pindata[1] ) { 
          case 0x07: {  //handle xfer to_/from tag
            EMU_SLIX2_Communication(clrc668_fifo_buffer, clrc668_fifo_length, clrc668_fifo_buffer, &clrc668_fifo_length);
            EMU_CLRC688_IRQSet(1);   //signal IRQ
            break;
          }
        }
        if( pindata[1] & 0x80 ) //reset the counter when software standby mode is entered
          EMU_SLIX2_CounterReset();
        break;
      } 

      case 0x04: { //fifolen
        if( 1 == inlength ) {
          poutdata[0] = clrc668_fifo_length;
          *poutlength = 1;
        }
        break;
      }

      case 0x05: { //fifodata
        if( inlength>1 ) { //incoming data
          memcpy(clrc668_fifo_buffer, pindata+1, inlength-1);
        }
        else { //outgoing data
          if( clrc668_fifo_length )
            memcpy(poutdata, clrc668_fifo_buffer, clrc668_fifo_length);
          *poutlength = clrc668_fifo_length;
          clrc668_fifo_length = 0;
        }
        break;
      }

      case 0x06: //irq0 register
      case 0x07: //irq1 register
      { 
        if( 1 == inlength ) { //is it a read?
          poutdata[0] = 0x7F; //signal all ints
          *poutlength = 1;
        }
        else
          EMU_CLRC688_IRQSet(0);  //clear interrupts / stop signalling IRQ
        break;
      }

      case 0x0A: { //error register
        poutdata[0] = 0x00; //no error
        *poutlength = 1;
        break;
      }

      default: //ignore all other register writes
        break;
    }
  }
}

void InitEmulationWithDefaultData(void) {
  uint8_t pair_index = EMU_TagPairChooseBootIndex();
  const DmoTagEmuPair* pair = &DMO_TAG_EMU_PAIRS[pair_index];
  (void)EMU_TagPairStoreNextIndex(pair_index);

  // use the selected UID+signature pair and mirror UID bytes into sysinfo
  EMU_SLIX2_INVENTORY[0] = 0x01;
  memcpy(EMU_SLIX2_INVENTORY + 1, pair->uid, 8);
  EMU_SLIX2_SyncSysInfoWithInventoryUID();
  memcpy(EMU_SLIX2_NXPSYSINFO, DMO_TAG_SLIX2_NXPSYSINFO, SLIX2_NXPSYSINFO_LEN); 
  memcpy(EMU_SLIX2_SIGNATURE, pair->signature, SLIX2_SIGNATURE_LEN); 

  const DmoSku* sku = DMO_FindSkuByName(DEFAULT_DMO_SKU_NAME);
  if( sku == NULL && DMO_SKUS_COUNT > 0 )
    sku = &DMO_SKUS[0];
  if( sku != NULL )
    memcpy(EMU_SLIX2_BLOCKS, sku->blocks, SLIX2_BLOCKS*sizeof(uint32_t));
}

/////////////////////////////////////////////////////////
// STM32 HAL function implementation for 
// - EXTI power down line monitoring
// - I2C slave emulation

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  static uint8_t clrc668_pwr_state = 1;

  uint8_t pdown = HAL_GPIO_ReadPin(EXTI0_IN_PWDN_GPIO_Port,EXTI0_IN_PWDN_Pin);
  if( pdown == clrc668_pwr_state )
    return;

  if( !pdown )
    HAL_I2C_EnableListen_IT(&hi2c1);  //power up
  else {
    HAL_I2C_DisableListen_IT(&hi2c1); //power down
    HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1);
    HAL_PWR_EnterSTANDBYMode();
  }
  clrc668_pwr_state = pdown;
}

#define I2C_RCV_BUF_SIZE 257
#define I2C_SND_BUF_SIZE 257

static uint8_t I2CSlaveRecvBuf[I2C_RCV_BUF_SIZE];
static uint8_t I2CSlaveRecvBufLen;

static uint8_t I2CSlaveSendBuf[I2C_SND_BUF_SIZE];
static uint8_t I2CSlaveSendBufLen;

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t transferDirection, uint16_t addrMatchCode) {
  if (transferDirection == I2C_DIRECTION_RECEIVE) { //DIRECTION_RECEIVE refers to master => for us (slave) means sending 
      EMU_CLRC688_Communication(I2CSlaveRecvBuf, I2CSlaveRecvBufLen, I2CSlaveSendBuf, &I2CSlaveSendBufLen);
    HAL_I2C_Slave_Seq_Transmit_IT(hi2c, I2CSlaveSendBuf, I2CSlaveSendBufLen, I2C_LAST_FRAME);
  } else {
    I2CSlaveRecvBufLen = 0;
    HAL_I2C_Slave_Seq_Receive_IT(hi2c, I2CSlaveRecvBuf, 1, I2C_NEXT_FRAME);
  }
  HAL_GPIO_WritePin(OUT_LED_GPIO_Port, OUT_LED_Pin, GPIO_PIN_RESET);
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c) {
  I2CSlaveRecvBufLen++;
  if( I2CSlaveRecvBufLen < I2C_RCV_BUF_SIZE )
    HAL_I2C_Slave_Seq_Receive_IT(hi2c, I2CSlaveRecvBuf + I2CSlaveRecvBufLen, 1, I2C_NEXT_FRAME);
}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c) {
}

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c) {
  EMU_CLRC688_Communication(I2CSlaveRecvBuf, I2CSlaveRecvBufLen, I2CSlaveSendBuf, &I2CSlaveSendBufLen);
  HAL_I2C_EnableListen_IT(hi2c);
  HAL_GPIO_WritePin(OUT_LED_GPIO_Port, OUT_LED_Pin, GPIO_PIN_SET);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
  HAL_I2C_EnableListen_IT(hi2c);
}
//
//////////////////////////////////////////////
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  if( rfid_scanner_attached )
    MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  //////////////////////////////////////////////
  //
  InitEmulationWithDefaultData();

  //start listening as I2C slave
  HAL_I2C_EnableListen_IT(&hi2c1);

  //main loop
  for(;;) {
    if( rfid_scanner_attached )
      RFID_Scanner_UpdateFromRealTag(
        &hi2c2,
        EMU_SLIX2_INVENTORY,  SLIX2_INVENTORY_LEN,
        EMU_SLIX2_SYSINFO,    SLIX2_SYSINFO_LEN,
        EMU_SLIX2_NXPSYSINFO, SLIX2_NXPSYSINFO_LEN,
        EMU_SLIX2_SIGNATURE,  SLIX2_SIGNATURE_LEN,
        EMU_SLIX2_BLOCKS,     SLIX2_BLOCKS,
        &EMU_SLIX2_TAG_PRESENT
      );
    HAL_Delay(100);
  }
  //
  //////////////////////////////////////////////
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 80;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OUT_LED_GPIO_Port, OUT_LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, OUT_IRQ_Pin|OUT_PULLUP_I2C2_SDA_Pin|OUT_PULLUP_I2C2_SCL_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OUT_PWDN_READER_GPIO_Port, OUT_PWDN_READER_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : OUT_LED_Pin */
  GPIO_InitStruct.Pin = OUT_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OUT_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : EXTI0_IN_PWDN_Pin */
  GPIO_InitStruct.Pin = EXTI0_IN_PWDN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(EXTI0_IN_PWDN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OUT_IRQ_Pin OUT_PULLUP_I2C2_SDA_Pin OUT_PULLUP_I2C2_SCL_Pin */
  GPIO_InitStruct.Pin = OUT_IRQ_Pin|OUT_PULLUP_I2C2_SDA_Pin|OUT_PULLUP_I2C2_SCL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA2 PA3 PA6 PA7
                           PA8 PA9 PA10 PA11
                           PA12 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB14
                           PB15 PB4 PB5 PB8
                           PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_14
                          |GPIO_PIN_15|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_8
                          |GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : IN_IRQ_READER_Pin */
  GPIO_InitStruct.Pin = IN_IRQ_READER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IN_IRQ_READER_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OUT_PWDN_READER_Pin */
  GPIO_InitStruct.Pin = OUT_PWDN_READER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OUT_PWDN_READER_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
