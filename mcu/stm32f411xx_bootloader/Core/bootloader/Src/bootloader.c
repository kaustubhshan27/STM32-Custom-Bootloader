/*
 * bootloader.c
 *
 *  Created on: Aug 9, 2023
 *      Author: kaustubh
 */

#include "stm32f4xx_hal.h"
#include "bootloader.h"

extern UART_HandleTypeDef huart2;
extern CRC_HandleTypeDef hcrc;

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

/**
 * @brief Get the MCU device ID.
 *
 * This function retrieves the MCU device ID from the Debug MCU register and returns it as a 16-bit unsigned integer.
 * The device ID is extracted from the IDCODE register using bitwise masking and shifting.
 *
 * @return The MCU device ID as a 16-bit unsigned integer.
 */
static uint16_t get_mcu_device_id(void)
{
	return ((uint16_t)(DBGMCU->IDCODE & 0xFFF));
}

/**
 * @brief Get the MCU revision ID.
 *
 * This function retrieves the MCU revision ID from the Debug MCU register and returns it as a 16-bit unsigned integer.
 * The revision ID is extracted from the IDCODE register using bitwise masking and shifting.
 *
 * @return The MCU revision ID as a 16-bit unsigned integer.
 */
static uint16_t get_mcu_revision_id(void)
{
	return ((uint16_t)((DBGMCU->IDCODE & 0xFFFF0000) >> 16));
}

/**
 * @brief Get the Read Protection (RDP) level of the MCU.
 *
 * This function retrieves the Read Protection (RDP) level of the MCU from the Flash Option Bytes (OB) configuration.
 * It uses the HAL library to access the Flash Option Bytes and extract the RDP level.
 *
 * @return The Read Protection (RDP) level as an 8-bit unsigned integer.
 */
static uint8_t get_rdp_level(void)
{
	FLASH_OBProgramInitTypeDef ob_handle;

	HAL_FLASH_OB_Unlock();
	HAL_FLASHEx_OBGetConfig(&ob_handle);
	HAL_FLASH_OB_Lock();

	return ((uint8_t)ob_handle.RDPLevel);
}

/**
 * @brief Verify the validity of a memory address.
 *
 * This function checks whether a given memory address is within the valid range of Flash or SRAM1 memory.
 * It compares the address against the base and end addresses of both Flash and SRAM1 memory regions.
 *
 * @param go_addr The memory address to be verified.
 * @return VALID_ADDR (1) if the address is within the valid range, INVALID_ADDR (0) otherwise.
 */
static uint8_t verify_addr(uint32_t go_addr)
{
	if((go_addr >= FLASH_BASE && go_addr <= FLASH_END) || (go_addr >= SRAM1_BASE && go_addr <= SRAM1_END))
	{
		return VALID_ADDR;
	}

	return INVALID_ADDR;
}

/**
 * @brief Erase Flash memory sectors or perform mass erase.
 *
 * This function performs erasing of specified Flash memory sectors or performs a mass erase of the entire Flash memory.
 * It uses the HAL library to perform Flash erasing operations.
 *
 * @param sector The starting sector to erase, or FLASH_MASS_ERASE to perform mass erase.
 * @param num_of_sectors The number of sectors to erase (used only when erasing specific sectors).
 * @return HAL status code indicating the success or failure of the erasing operation.
 *         INVALID_SECTOR if the specified sector(s) are invalid.
 */
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

/**
 * @brief Write data to Flash memory.
 *
 * This function writes a payload of data to the specified Flash memory address. It iterates over the payload
 * and programs each byte into the Flash memory using the HAL library functions.
 *
 * @param write_addr The starting address in Flash memory where the data will be written.
 * @param payload Pointer to the data to be written.
 * @param payload_len The length of the data payload in bytes.
 * @return HAL status code indicating the success or failure of the writing operation.
 */
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

/**
 * @brief Get the write protection status of Flash sectors.
 *
 * This function retrieves the write protection status of Flash sectors from the Flash Option Bytes (OB) configuration.
 * It uses the HAL library to access the Flash Option Bytes and extract the sector write protection status.
 *
 * @return The write protection status of Flash sectors as a 16-bit unsigned integer.
 */
static uint16_t get_flash_sector_status(void)
{
	FLASH_OBProgramInitTypeDef ob_handle;

	HAL_FLASH_OB_Unlock();
	HAL_FLASHEx_OBGetConfig(&ob_handle);
	HAL_FLASH_OB_Lock();

	return ((uint16_t)ob_handle.WRPSector);
}

/**
 * @brief Configure Flash Read/Write Protection and/or PCROP (PC Read-Out Protection).
 *
 * This function configures Flash protection modes such as write protection and PCROP for specified Flash sectors.
 * It allows enabling or disabling protection modes based on the provided parameters.
 *
 * @param sector_details The bitmask indicating the sectors to be configured for protection.
 * @param protection_mode The protection mode to be configured: WRITE_PROTECTION_MODE or PCROP_PROTECTION_MODE.
 * @param ability If 0, disables all protection; otherwise, enables the specified protection mode.
 * @return HAL status code indicating the success or failure of the configuration.
 *         INVALID_PROTECTION_MODE if an invalid protection mode is provided.
 */
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

/**
 * @brief Verify CRC32 checksum of a data buffer.
 *
 * This function calculates the CRC32 checksum of a given data buffer and compares it with a provided CRC32 checksum value.
 * It uses the HAL library's CRC module to calculate the CRC32 checksum.
 *
 * @param buf Pointer to the data buffer for which CRC32 checksum is to be calculated.
 * @param len Length of the data buffer in bytes.
 * @param crc32_host The expected CRC32 checksum value to compare with.
 * @return CRC32_VERIFY_SUCCESS if the calculated CRC32 matches the expected CRC32 value;
 *         CRC32_VERIFY_FAIL if the calculated CRC32 does not match the expected value.
 */
static uint8_t bl_verify_crc32(uint8_t *buf, uint32_t len, uint32_t crc32_host)
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

/**
 * @brief Bootloader command processing loop.
 *
 * This function implements the main loop of the bootloader that reads incoming commands over UART and processes them.
 * The received command is parsed and the corresponding bootloader command function is called based on the received command code.
 * It continuously listens for commands and responds accordingly.
 */
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

/**
 * @brief Transmit data over UART as part of bootloader communication.
 *
 * This function is used to send data over the UART interface as part of the bootloader communication process.
 *
 * @param buf Pointer to the data buffer containing the data to be transmitted.
 * @param len Length of the data buffer in bytes.
 */
void bootloader_write_data(uint8_t *buf, uint32_t len)
{
	HAL_UART_Transmit(COM_UART, buf, len, HAL_MAX_DELAY);
}

/**
 * @brief Jump to the user application stored in flash memory.
 *
 * This function is used to jump to the user application stored in flash memory. It retrieves the top of the stack value
 * and the reset handler address from the user application's vector table and then performs a jump to the user application.
 */
void bootloader_jump_to_user_app(void)
{
	uint32_t top_of_stack = (uint32_t)(*((volatile uint32_t *)BASE_ADDR_FLASH_SECTOR_2));
	__set_MSP(top_of_stack);

	void(* app_reset_handler)(void) = (app_reset_handler_t)(*((volatile uint32_t *)(BASE_ADDR_FLASH_SECTOR_2 + 4)));
	app_reset_handler();
}

/**
 * @brief Process the "Get Bootloader Version" command and send the bootloader version.
 *
 * This function processes the "Get Bootloader Version" command received from the host. It calculates the CRC32 checksum
 * of the received command packet and compares it with the provided CRC32 checksum. If the checksum matches, an acknowledgment
 * is sent and the bootloader version is transmitted to the host.
 *
 * @param bl_rx_buf Pointer to the received command packet.
 */
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

/**
 * @brief Process the "Get Help" command and send the list of supported commands.
 *
 * This function processes the "Get Help" command received from the host. It calculates the CRC32 checksum
 * of the received command packet and compares it with the provided CRC32 checksum. If the checksum matches,
 * an acknowledgment is sent, and the list of supported commands is transmitted to the host.
 *
 * @param bl_rx_buf Pointer to the received command packet.
 */
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

/**
 * @brief Process the "Get Chip ID" command and send the MCU device ID and revision ID.
 *
 * This function processes the "Get Chip ID" command received from the host. It calculates the CRC32 checksum
 * of the received command packet and compares it with the provided CRC32 checksum. If the checksum matches,
 * an acknowledgment is sent, and the MCU device ID and revision ID are transmitted to the host.
 *
 * @param bl_rx_buf Pointer to the received command packet.
 */
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

/**
 * @brief Process the "Get RDP Status" command and send the Read Protection (RDP) level.
 *
 * This function processes the "Get RDP Status" command received from the host. It calculates the CRC32 checksum
 * of the received command packet and compares it with the provided CRC32 checksum. If the checksum matches,
 * an acknowledgment is sent, and the current Read Protection (RDP) level is transmitted to the host.
 *
 * @param bl_rx_buf Pointer to the received command packet.
 */
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

/**
 * @brief Process the "Go to Address" command and jump to the specified address.
 *
 * This function processes the "Go to Address" command received from the host. It calculates the CRC32 checksum
 * of the received command packet and compares it with the provided CRC32 checksum. If the checksum matches,
 * an acknowledgment is sent, and the function attempts to jump to the specified address provided in the command.
 *
 * @param bl_rx_buf Pointer to the received command packet.
 */
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

/**
 * @brief Process the "Flash Erase" command to erase specified sectors of the flash memory.
 *
 * This function processes the "Flash Erase" command received from the host. It calculates the CRC32 checksum
 * of the received command packet and compares it with the provided CRC32 checksum. If the checksum matches,
 * an acknowledgment is sent, and the function attempts to erase the specified flash memory sectors provided
 * in the command.
 *
 * @param bl_rx_buf Pointer to the received command packet.
 */
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

/**
 * @brief Process the "Flash Write" command to write data to the flash memory.
 *
 * This function processes the "Flash Write" command received from the host. It calculates the CRC32 checksum
 * of the received command packet and compares it with the provided CRC32 checksum. If the checksum matches,
 * an acknowledgment is sent, and the function attempts to write data to the specified flash memory address
 * provided in the command.
 *
 * @param bl_rx_buf Pointer to the received command packet.
 */
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

/**
 * @brief Process the "Flash Read" command to read data from flash memory.
 *
 * This function processes the "Flash Read" command received from the host. It calculates the CRC32 checksum
 * of the received command packet and compares it with the provided CRC32 checksum. If the checksum matches,
 * an acknowledgment is sent, and the function attempts to read data from the specified flash memory address
 * provided in the command.
 *
 * @param bl_rx_buf Pointer to the received command packet.
 */
void bl_flash_read(uint8_t *bl_rx_buf)
{

}

/**
 * @brief Process the "Enable Flash Read/Write Protection" command to enable read/write protection for flash sectors.
 *
 * This function processes the "Enable Flash Read/Write Protection" command received from the host. It calculates
 * the CRC32 checksum of the received command packet and compares it with the provided CRC32 checksum. If the
 * checksum matches, an acknowledgment is sent, and the function attempts to enable read/write protection for
 * the specified flash sectors based on the protection mode and sector details provided in the command.
 *
 * @param bl_rx_buf Pointer to the received command packet.
 */
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

/**
 * @brief Process the "Disable Flash Read/Write Protection" command to disable read/write protection for flash sectors.
 *
 * This function processes the "Disable Flash Read/Write Protection" command received from the host. It calculates
 * the CRC32 checksum of the received command packet and compares it with the provided CRC32 checksum. If the
 * checksum matches, an acknowledgment is sent, and the function attempts to disable read/write protection for
 * the specified flash sectors by configuring the flash protection settings. After completing the operation, the
 * protection status is sent back to the host.
 *
 * @param bl_rx_buf Pointer to the received command packet.
 */
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

/**
 * @brief Process the "Read Flash Sector Protection Status" command to retrieve the flash sector protection status.
 *
 * This function processes the "Read Flash Sector Protection Status" command received from the host. It calculates
 * the CRC32 checksum of the received command packet and compares it with the provided CRC32 checksum. If the
 * checksum matches, an acknowledgment is sent, and the function retrieves the protection status of flash sectors
 * using the get_flash_sector_status function. The protection status is then sent back to the host.
 *
 * @param bl_rx_buf Pointer to the received command packet.
 */
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

/**
 * @brief Process the "Disable OTP Read Protection" command to disable OTP memory read protection.
 *
 * This function processes the "Disable OTP Read Protection" command received from the host. It calculates
 * the CRC32 checksum of the received command packet and compares it with the provided CRC32 checksum. If the
 * checksum matches, an acknowledgment is sent, and the function disables the OTP memory read protection.
 * The status of the operation is sent back to the host.
 *
 * @param bl_rx_buf Pointer to the received command packet.
 */
void bl_disable_otp_read(uint8_t *bl_rx_buf)
{

}

/**
 * @brief Send an acknowledgment response to the host.
 *
 * This function sends an acknowledgment response to the host. The acknowledgment response consists of the
 * acknowledgment byte (0x79) followed by the response length. The response length specifies the length of the
 * subsequent response data. The acknowledgment response is transmitted over the communication UART.
 *
 * @param resp_len Length of the response data following the acknowledgment byte and response length.
 */
void bl_send_ack(uint8_t resp_len)
{
	uint8_t ack_buf[BL_ACK_BUF_SIZE] = {BL_ACK, resp_len};

	HAL_UART_Transmit(COM_UART, ack_buf, BL_ACK_BUF_SIZE, HAL_MAX_DELAY);
}

/**
 * @brief Send a negative acknowledgment response to the host.
 *
 * This function sends a negative acknowledgment response (NACK) to the host. The NACK response consists of the
 * NACK byte (0x1F). The NACK response is transmitted over the communication UART.
 */
void bl_send_nack(void)
{
	uint8_t nack_buf[BL_NACK_BUF_SIZE] = {BL_NACK};

	HAL_UART_Transmit(COM_UART, nack_buf, BL_NACK_BUF_SIZE, HAL_MAX_DELAY);
}
