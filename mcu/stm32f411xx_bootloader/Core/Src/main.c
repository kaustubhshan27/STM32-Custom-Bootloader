/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
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
#define COM_UART	&huart2
#define DBG_UART	&huart6
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
uint8_t bl_rx_buf[BL_RX_BUFFER_SIZE] = {0};
bl_cmd_t bl_cmd_list[] = {
		BL_GET_VER,
		BL_GET_HELP,
		BL_GET_CID,
		BL_GET_RDP_STATUS,
		BL_GO_TO_ADDR,
		BL_FLASH_ERASE,
		BL_FLASH_WRITE,
		BL_FLASH_READ,
		BL_ENABLE_READ_WRITE_PROTECT,
		BL_DISABLE_READ_WRITE_PROTECT,
		BL_READ_FLASH_SECTOR_STATUS,
		BL_OTP_READ};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_CRC_Init(void);
/* USER CODE BEGIN PFP */
#ifdef BL_DEBUG_PRINTF
static void print_dbg_msg(char *format, ...);
#endif
static uint16_t get_mcu_device_id(void);
static uint16_t get_mcu_revision_id(void);
static uint8_t get_rdp_level(void);
static uint8_t verify_addr(uint32_t go_addr);
static uint8_t flash_erase(uint8_t sector, uint8_t num_of_sectors);
static uint8_t flash_write(uint32_t write_addr, uint8_t *payload, uint32_t payload_len);
static uint16_t get_flash_sector_status(void);
static uint8_t configure_flash_rw_protect(uint8_t sector_details, uint8_t protection_mode, uint8_t ability);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */
  if(HAL_GPIO_ReadPin(USER_BUTTON_GPIO_Port, USER_BUTTON_Pin) == GPIO_PIN_RESET)
  {
#ifdef BL_DEBUG_PRINTF
	  print_dbg_msg("Entering the bootloader...\r\n");
#endif
	  bootloader_read_data();
  }
  else
  {
#ifdef BL_DEBUG_PRINTF
	  print_dbg_msg("Entering the user application...\r\n");
#endif
	  bootloader_jump_to_user_app();
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ONBOARD_LED_GPIO_Port, ONBOARD_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_BUTTON_Pin */
  GPIO_InitStruct.Pin = USER_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(USER_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ONBOARD_LED_Pin */
  GPIO_InitStruct.Pin = ONBOARD_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ONBOARD_LED_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void bootloader_read_data(void)
{
	bl_cmd_t bl_cmd = BL_INVALID_CMD;

	while(1)
	{
		memset(bl_rx_buf, 0, BL_RX_BUFFER_SIZE);
		HAL_UART_Receive(COM_UART, bl_rx_buf, 1, HAL_MAX_DELAY);
		HAL_UART_Receive(COM_UART, &bl_rx_buf[1], bl_rx_buf[0], HAL_MAX_DELAY);
		bl_cmd = (bl_cmd_t)bl_rx_buf[1];

		switch(bl_cmd)
		{
			case BL_GET_VER:
				bl_get_ver(bl_rx_buf);
				break;
			case BL_GET_HELP:
				bl_get_help(bl_rx_buf);
				break;
			case BL_GET_CID:
				bl_get_cid(bl_rx_buf);
				break;
			case BL_GET_RDP_STATUS:
				bl_get_rdp_status(bl_rx_buf);
				break;
			case BL_GO_TO_ADDR:
				bl_goto_addr(bl_rx_buf);
				break;
			case BL_FLASH_ERASE:
				bl_flash_erase(bl_rx_buf);
				break;
			case BL_FLASH_WRITE:
				bl_flash_write(bl_rx_buf);
				break;
			case BL_FLASH_READ:
				bl_flash_read(bl_rx_buf);
				break;
			case BL_ENABLE_READ_WRITE_PROTECT:
				bl_enable_flash_rw_protect(bl_rx_buf);
				break;
			case BL_DISABLE_READ_WRITE_PROTECT:
				bl_disable_flash_rw_protect(bl_rx_buf);
				break;
			case BL_READ_FLASH_SECTOR_STATUS:
				bl_read_flash_sector_status(bl_rx_buf);
				break;
			case BL_OTP_READ:
				bl_disable_otp_read(bl_rx_buf);
				break;
			default:
				break;
		}
	}
}

void bootloader_write_data(uint8_t *buf, uint32_t len)
{
	HAL_UART_Transmit(COM_UART, buf, len, HAL_MAX_DELAY);
}

void bootloader_jump_to_user_app(void)
{
	uint32_t top_of_stack = (uint32_t)(*((volatile uint32_t *)BASE_ADDR_FLASH_SECTOR_2));
	__set_MSP(top_of_stack);

	void(* app_reset_handler)(void) = (app_reset_handler_t)(*((volatile uint32_t *)(BASE_ADDR_FLASH_SECTOR_2 + 4)));
	app_reset_handler();
}

void bl_get_ver(uint8_t *bl_rx_buf)
{
	uint32_t cmd_pkt_len = bl_rx_buf[0] + 1;
	uint32_t crc32_host = *((uint32_t *)(bl_rx_buf + cmd_pkt_len - CRC_LEN_BYTES));

	if(bl_verify_crc32(bl_rx_buf, cmd_pkt_len - CRC_LEN_BYTES, crc32_host))
	{
		bl_send_ack(BL_GET_VER_RESP_LEN_BYTES);

		uint8_t bl_version = BL_VERSION;
		bootloader_write_data(&bl_version, BL_GET_VER_RESP_LEN_BYTES);
	}
	else
	{
		bl_send_nack();
	}
}

void bl_get_help(uint8_t *bl_rx_buf)
{
	uint32_t cmd_pkt_len = bl_rx_buf[0] + 1;
	uint32_t crc32_host = *((uint32_t *)(bl_rx_buf + cmd_pkt_len - CRC_LEN_BYTES));

	if(bl_verify_crc32(bl_rx_buf, cmd_pkt_len - CRC_LEN_BYTES, crc32_host))
	{
		bl_send_ack(BL_GET_HELP_RESP_LEN_BYTES);
		bootloader_write_data((uint8_t *)bl_cmd_list, BL_GET_HELP_RESP_LEN_BYTES);
	}
	else
	{
		bl_send_nack();
	}
}

void bl_get_cid(uint8_t *bl_rx_buf)
{
	uint32_t cmd_pkt_len = bl_rx_buf[0] + 1;
	uint32_t crc32_host = *((uint32_t *)(bl_rx_buf + cmd_pkt_len - CRC_LEN_BYTES));

	if(bl_verify_crc32(bl_rx_buf, cmd_pkt_len - CRC_LEN_BYTES, crc32_host))
	{
		bl_send_ack(BL_GET_CID_RESP_LEN_BYTES);

		uint16_t cid[2] = {0};
		cid[0] = get_mcu_device_id();
		cid[1] = get_mcu_revision_id();
		bootloader_write_data((uint8_t *)cid, BL_GET_CID_RESP_LEN_BYTES);
	}
	else
	{
		bl_send_nack();
	}
}

void bl_get_rdp_status(uint8_t *bl_rx_buf)
{
	uint32_t cmd_pkt_len = bl_rx_buf[0] + 1;
	uint32_t crc32_host = *((uint32_t *)(bl_rx_buf + cmd_pkt_len - CRC_LEN_BYTES));

	if(bl_verify_crc32(bl_rx_buf, cmd_pkt_len - CRC_LEN_BYTES, crc32_host))
	{
		bl_send_ack(BL_GET_RDP_STATUS_RESP_LEN_BYTES);

		uint8_t rdp_level;
		rdp_level = get_rdp_level();
		bootloader_write_data((uint8_t *)&rdp_level, BL_GET_RDP_STATUS_RESP_LEN_BYTES);
	}
	else
	{
		bl_send_nack();
	}
}

void bl_goto_addr(uint8_t *bl_rx_buf)
{
	uint32_t cmd_pkt_len = bl_rx_buf[0] + 1;
	uint32_t crc32_host = *((uint32_t *)(bl_rx_buf + cmd_pkt_len - CRC_LEN_BYTES));

	if(bl_verify_crc32(bl_rx_buf, cmd_pkt_len - CRC_LEN_BYTES, crc32_host))
	{
		bl_send_ack(BL_GO_TO_ADDR_RESP_LEN_BYTES);

		uint32_t goto_addr = *((uint32_t *)(&bl_rx_buf[2]));
		uint8_t addr_status;
		if(verify_addr(goto_addr) == VALID_ADDR)
		{
			addr_status = VALID_ADDR;
			bootloader_write_data((uint8_t *)&addr_status, BL_GO_TO_ADDR_RESP_LEN_BYTES);

			/* Increment the jump address by 1 to comply with the Thumb ISA */
			void(*jump_addr)(void) = (jump_addr_t)(goto_addr + 1);
			jump_addr();
		}
		else
		{
			addr_status = INVALID_ADDR;
			bootloader_write_data((uint8_t *)&addr_status, BL_GO_TO_ADDR_RESP_LEN_BYTES);
		}
	}
	else
	{
		bl_send_nack();
	}
}

void bl_flash_erase(uint8_t *bl_rx_buf)
{
	uint32_t cmd_pkt_len = bl_rx_buf[0] + 1;
	uint32_t crc32_host = *((uint32_t *)(bl_rx_buf + cmd_pkt_len - CRC_LEN_BYTES));

	if(bl_verify_crc32(bl_rx_buf, cmd_pkt_len - CRC_LEN_BYTES, crc32_host))
	{
		bl_send_ack(BL_FLASH_ERASE_RESP_LEN_BYTES);

		HAL_GPIO_WritePin(ONBOARD_LED_GPIO_Port, ONBOARD_LED_Pin, GPIO_PIN_SET);
		uint8_t erase_status = flash_erase(bl_rx_buf[2], bl_rx_buf[3]);
		HAL_GPIO_WritePin(ONBOARD_LED_GPIO_Port, ONBOARD_LED_Pin, GPIO_PIN_RESET);

		bootloader_write_data(&erase_status, BL_FLASH_ERASE_RESP_LEN_BYTES);
	}
	else
	{
		bl_send_nack();
	}
}

void bl_flash_write(uint8_t *bl_rx_buf)
{
	uint32_t cmd_pkt_len = bl_rx_buf[0] + 1;
	uint32_t crc32_host = *((uint32_t *)(bl_rx_buf + cmd_pkt_len - CRC_LEN_BYTES));

	if(bl_verify_crc32(bl_rx_buf, cmd_pkt_len - CRC_LEN_BYTES, crc32_host))
	{
		bl_send_ack(BL_MEM_WRITE_RESP_LEN_BYTES);

		uint32_t write_addr = *(uint32_t *)(&bl_rx_buf[2]);
		uint8_t *payload = (uint8_t *)(&bl_rx_buf[7]);
		uint32_t payload_len = bl_rx_buf[6];
		uint8_t write_status;

		if(verify_addr(write_addr) == VALID_ADDR)
		{
			HAL_GPIO_WritePin(ONBOARD_LED_GPIO_Port, ONBOARD_LED_Pin, GPIO_PIN_SET);
			write_status = flash_write(write_addr, payload, payload_len);
			HAL_GPIO_WritePin(ONBOARD_LED_GPIO_Port, ONBOARD_LED_Pin, GPIO_PIN_RESET);
		}
		else
		{
			write_status = INVALID_ADDR;
		}

		bootloader_write_data(&write_status, BL_MEM_WRITE_RESP_LEN_BYTES);
	}
	else
	{
		bl_send_nack();
	}
}

void bl_flash_read(uint8_t *bl_rx_buf)
{

}

void bl_enable_flash_rw_protect(uint8_t *bl_rx_buf)
{
	uint32_t cmd_pkt_len = bl_rx_buf[0] + 1;
	uint32_t crc32_host = *((uint32_t *)(bl_rx_buf + cmd_pkt_len - CRC_LEN_BYTES));

	if(bl_verify_crc32(bl_rx_buf, cmd_pkt_len - CRC_LEN_BYTES, crc32_host))
	{
		bl_send_ack(BL_ENABLE_READ_WRITE_PROTECT_RESP_LEN_BYTES);

		uint8_t protect_status = configure_flash_rw_protect(bl_rx_buf[2], bl_rx_buf[3], ENABLE);
		bootloader_write_data(&protect_status, BL_ENABLE_READ_WRITE_PROTECT_RESP_LEN_BYTES);
	}
	else
	{
		bl_send_nack();
	}
}

void bl_disable_flash_rw_protect(uint8_t *bl_rx_buf)
{
	uint32_t cmd_pkt_len = bl_rx_buf[0] + 1;
	uint32_t crc32_host = *((uint32_t *)(bl_rx_buf + cmd_pkt_len - CRC_LEN_BYTES));

	if(bl_verify_crc32(bl_rx_buf, cmd_pkt_len - CRC_LEN_BYTES, crc32_host))
	{
		bl_send_ack(BL_DISABLE_READ_WRITE_PROTECT_RESP_LEN_BYTES);

		/* The first two parameters don't matter during R/W protect disable */
		uint8_t protect_status = configure_flash_rw_protect(0, 0, DISABLE);
		bootloader_write_data(&protect_status, BL_ENABLE_READ_WRITE_PROTECT_RESP_LEN_BYTES);
	}
	else
	{
		bl_send_nack();
	}
}

void bl_read_flash_sector_status(uint8_t *bl_rx_buf)
{
	uint32_t cmd_pkt_len = bl_rx_buf[0] + 1;
	uint32_t crc32_host = *((uint32_t *)(bl_rx_buf + cmd_pkt_len - CRC_LEN_BYTES));

	if(bl_verify_crc32(bl_rx_buf, cmd_pkt_len - CRC_LEN_BYTES, crc32_host))
	{
		bl_send_ack(BL_READ_SECTOR_STATUS_RESP_LEN_BYTES);

		uint16_t flash_sector_status = get_flash_sector_status();
		bootloader_write_data((uint8_t *)&flash_sector_status, BL_READ_SECTOR_STATUS_RESP_LEN_BYTES);
	}
	else
	{
		bl_send_nack();
	}
}

void bl_disable_otp_read(uint8_t *bl_rx_buf)
{

}

void bl_send_ack(uint8_t resp_len)
{
	uint8_t ack_buf[BL_ACK_BUF_SIZE] = {BL_ACK, resp_len};

	HAL_UART_Transmit(COM_UART, ack_buf, BL_ACK_BUF_SIZE, HAL_MAX_DELAY);
}

void bl_send_nack(void)
{
	uint8_t nack_buf[BL_NACK_BUF_SIZE] = {BL_NACK};

	HAL_UART_Transmit(COM_UART, nack_buf, BL_NACK_BUF_SIZE, HAL_MAX_DELAY);
}

static uint16_t get_mcu_device_id(void)
{
	return ((uint16_t)(DBGMCU->IDCODE & 0xFFF));
}

static uint16_t get_mcu_revision_id(void)
{
	return ((uint16_t)((DBGMCU->IDCODE & 0xFFFF0000) >> 16));
}

static uint8_t get_rdp_level(void)
{
	FLASH_OBProgramInitTypeDef ob_handle;

	HAL_FLASH_OB_Unlock();
	HAL_FLASHEx_OBGetConfig(&ob_handle);
	HAL_FLASH_OB_Lock();

	return ((uint8_t)ob_handle.RDPLevel);
}

static uint8_t verify_addr(uint32_t go_addr)
{
	if((go_addr >= FLASH_BASE && go_addr <= FLASH_END) || (go_addr >= SRAM1_BASE && go_addr <= SRAM1_END))
	{
		return VALID_ADDR;
	}

	return INVALID_ADDR;
}

static uint8_t flash_erase(uint8_t sector, uint8_t num_of_sectors)
{
	FLASH_EraseInitTypeDef flash_erase_handle;
	uint32_t sector_error = 0;
	uint8_t status = HAL_OK;

	if(num_of_sectors > NUM_OF_ERASABLE_SECTORS)
	{
		return INVALID_SECTOR;
	}

	/* Condition for mass-erase */
	if(sector == FLASH_MASS_ERASE)
	{
		flash_erase_handle.TypeErase = FLASH_TYPEERASE_MASSERASE;
	}
	else if (sector >= FLASH_SECTOR_0 && sector <= FLASH_SECTOR_7)
	{
		flash_erase_handle.TypeErase = FLASH_TYPEERASE_SECTORS;
		flash_erase_handle.Sector = sector;

		/* Safety check - when the number of sectors requested to erase is more than the remaining number of sectors */
		if(FLASH_SECTOR_7 - sector + 1 < num_of_sectors)
		{
			num_of_sectors = FLASH_SECTOR_7 - sector + 1;
		}
		flash_erase_handle.NbSectors = num_of_sectors;
	}
	else
	{
		return INVALID_SECTOR;
	}

	flash_erase_handle.Banks = FLASH_BANK_1;
	flash_erase_handle.VoltageRange = FLASH_VOLTAGE_RANGE_3;

	HAL_FLASH_Unlock();
	status = HAL_FLASHEx_Erase(&flash_erase_handle, &sector_error);
	HAL_FLASH_Lock();

	return status;
}

static uint8_t flash_write(uint32_t write_addr, uint8_t *payload, uint32_t payload_len)
{
	uint8_t status = HAL_OK;

    HAL_FLASH_Unlock();

    for(uint32_t index = 0 ; index < payload_len ; index++)
    {
        status |= HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, write_addr + index, payload[index]);
    }

    HAL_FLASH_Lock();

	return status;
}

static uint16_t get_flash_sector_status(void)
{
	FLASH_OBProgramInitTypeDef ob_handle;

	HAL_FLASH_OB_Unlock();
	HAL_FLASHEx_OBGetConfig(&ob_handle);
	HAL_FLASH_OB_Lock();

	return ((uint16_t)ob_handle.WRPSector);
}

static uint8_t configure_flash_rw_protect(uint8_t sector_details, uint8_t protection_mode, uint8_t ability)
{
	uint8_t status = HAL_OK;

	/* Disable all protection */
	if(!ability)
	{
		FLASH_OBProgramInitTypeDef ob_handle = {.OptionType = OPTIONBYTE_WRP,
												.WRPState = OB_WRPSTATE_DISABLE,
												.WRPSector = OB_WRP_SECTOR_All,
												.Banks = FLASH_BANK_1};
		HAL_FLASH_OB_Unlock();

		status |= HAL_FLASHEx_OBProgram(&ob_handle);

		HAL_FLASH_OB_Launch(); /* Call when the option byte is modified */
		HAL_FLASH_OB_Lock();
	}
	else
	{
		/* Disable PCROP protection and enable write protection for specified sectors */
		if(protection_mode == WRITE_PROTECTION_MODE)
		{
			FLASH_OBProgramInitTypeDef ob_handle = {.OptionType = OPTIONBYTE_WRP,
													.WRPState = OB_WRPSTATE_ENABLE,
													.WRPSector = sector_details,
													.Banks = FLASH_BANK_1};
			HAL_FLASH_OB_Unlock();

			/* Note: Switching from PCROP to WRITE mode involves a Flash mass-erase. */

			status |= HAL_FLASHEx_OBProgram(&ob_handle);

			HAL_FLASH_OB_Launch(); /* Call when the option byte is modified */
			HAL_FLASH_OB_Lock();
		}
		/* Enable PCROP protection for specified sectors */
		else if(protection_mode == PCROP_PROTECTION_MODE)
		{
			FLASH_AdvOBProgramInitTypeDef adv_ob_handle = {.OptionType = OPTIONBYTE_PCROP,
														   .PCROPState = OB_PCROP_STATE_DISABLE,
														   .Sectors = OB_PCROP_SECTOR_All};

			HAL_FLASH_OB_Unlock();

			HAL_FLASHEx_AdvOBGetConfig(&adv_ob_handle);
			/* Set the SPRMOD bit only if it is not set.
			 * This can happen when directly switching from WRITE to PCROP mode*/
			if(!((adv_ob_handle.Sectors >> 8) & OB_PCROP_SELECTED))
			{
				status |= HAL_FLASHEx_OB_SelectPCROP();
				status |= HAL_FLASHEx_AdvOBProgram(&adv_ob_handle);
			}

			adv_ob_handle.PCROPState = OB_PCROP_STATE_ENABLE;
			adv_ob_handle.Sectors = sector_details;
			status |= HAL_FLASHEx_AdvOBProgram(&adv_ob_handle);

			HAL_FLASH_OB_Launch(); /* Call when the option byte is modified */
			HAL_FLASH_OB_Lock();
		}
		else
		{
			return INVALID_PROTECTION_MODE;
		}
	}

	return status;
}

uint8_t bl_verify_crc32(uint8_t *buf, uint32_t len, uint32_t crc32_host)
{
	uint32_t crc32_result = 0;
	uint32_t index_val = 0;

	for(uint32_t index = 0; index < len; index++)
	{
		index_val = (uint32_t)buf[index];
		crc32_result = HAL_CRC_Accumulate(&hcrc, &index_val, 1);
	}

	/* Reset the CRC calculation engine */
	__HAL_CRC_DR_RESET(&hcrc);

	if(crc32_result == crc32_host)
	{
		return CRC32_VERIFY_SUCCESS;
	}

	return CRC32_VERIFY_FAIL;
}

#ifdef BL_DEBUG_PRINTF
static void print_dbg_msg(char *format, ...)
{
	char msg[80];

	/* Extract the argument list */
	va_list args;
	va_start(args, format);
	vsprintf(msg, format, args);

	/* Transmit the formatted string via DBG_UART */
	HAL_UART_Transmit(DBG_UART, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);

	va_end(args);
}
#endif
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
