/*
 * bootloader.h
 *
 *  Created on: Aug 9, 2023
 *      Author: kaustubh
 */

#ifndef BOOTLOADER_INC_BOOTLOADER_H_
#define BOOTLOADER_INC_BOOTLOADER_H_

#include "uapi_bootloader.h"
#include <string.h>
#include <stdint.h>

#define ENABLE	1
#define DISABLE	0

#define BASE_ADDR_FLASH_SECTOR_2	0x08008000U

#define BL_ACK						0xA5
#define BL_NACK						0x7F
#define BL_ACK_BUF_SIZE				2
#define BL_NACK_BUF_SIZE			1
#define BL_RX_BUFFER_SIZE			200

#define BL_GET_VER_RESP_LEN_BYTES						1
#define BL_GET_HELP_RESP_LEN_BYTES						12
#define BL_GET_CID_RESP_LEN_BYTES						4
#define BL_GET_RDP_STATUS_RESP_LEN_BYTES				1
#define BL_GO_TO_ADDR_RESP_LEN_BYTES					1
#define BL_FLASH_ERASE_RESP_LEN_BYTES					1
#define BL_MEM_WRITE_RESP_LEN_BYTES						1
#define BL_MEM_READ_RESP_LEN_BYTES						0
#define BL_ENABLE_READ_WRITE_PROTECT_RESP_LEN_BYTES		1
#define BL_DISABLE_READ_WRITE_PROTECT_RESP_LEN_BYTES	1
#define BL_READ_SECTOR_STATUS_RESP_LEN_BYTES			2
#define BL_OTP_READ_RESP_LEN_BYTES						1

#define BL_VERSION	0x10

#define CRC_LEN_BYTES				0x04
#define CRC32_VERIFY_SUCCESS		1
#define CRC32_VERIFY_FAIL			0

#define SRAM1_SIZE		(128 * 1024)	/* 128KB SRAM size in STM32F411xx */
#define SRAM1_END		(SRAM1_BASE + SRAM1_SIZE)
#define VALID_ADDR		0x01
#define INVALID_ADDR	0x04

#define NUM_OF_ERASABLE_SECTORS	0x08	/* Sectors 0 - 7 */
#define FLASH_MASS_ERASE		0xFF
#define INVALID_SECTOR			0x04

#define WRITE_PROTECTION_MODE	0x01
#define PCROP_PROTECTION_MODE	0x02
#define INVALID_PROTECTION_MODE	0x03

#define COM_UART	&huart2

typedef void(*app_reset_handler_t)(void);
typedef void(*jump_addr_t)(void);

typedef enum bootloader_cmd {
	BL_GET_VER = 0x51,
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
	BL_OTP_READ,
	BL_INVALID_CMD
}bl_cmd_t;

void bootloader_write_data(uint8_t *buf, uint32_t len);

void bl_get_ver(uint8_t *bl_rx_buf);
void bl_get_help(uint8_t *bl_rx_buf);
void bl_get_cid(uint8_t *bl_rx_buf);
void bl_get_rdp_status(uint8_t *bl_rx_buf);
void bl_goto_addr(uint8_t *bl_rx_buf);
void bl_flash_erase(uint8_t *bl_rx_buf);
void bl_flash_write(uint8_t *bl_rx_buf);
void bl_flash_read(uint8_t *bl_rx_buf);
void bl_enable_flash_rw_protect(uint8_t *bl_rx_buf);
void bl_disable_flash_rw_protect(uint8_t *bl_rx_buf);
void bl_read_flash_sector_status(uint8_t *bl_rx_buf);
void bl_disable_otp_read(uint8_t *bl_rx_buf);
void bl_send_ack(uint8_t resp_len);
void bl_send_nack(void);

#endif /* BOOTLOADER_INC_BOOTLOADER_H_ */
