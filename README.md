# STM32-Custom-Bootloader

## 📒 Table of Contents
- [📍 Overview](#-overview)
- [📂 Bootloader Commands](#-bootloader-commands)
---

## 📍 Overview
STM32F411xx Custom Bootloader to update on-chip firmware, configure flash sector protection, and read flash sector status using UART.

---

## 📍 Bootloader Commands
| Command       | Command Code  | Description |
| ------------- | ------------- | ------------- |
| CMD_BL_GET_VER  | 0x51  | Read the bootloader version from the MCU |
| CMD_BL_GET_HELP  | 0x52  | Get help on the commands supported by the bootloader |
| CMD_BL_GET_CID  | 0x53  | Read the MCU chip identification number |
| CMD_BL_GET_RDP_STATUS  | 0x54  | Read the Flash read protection level |
| CMD_BL_GO_TO_ADDR  | 0x55  | Jump the bootloader to the specified address |
| CMD_BL_FLASH_ERASE  | 0x56  | Mass erase or sector erase of the user Flash |
| CMD_BL_FLASH_WRITE | 0x57  | Write to the user Flash of the MCU |
| CMD_BL_ENABLE_RW_PROTECT  | 0x59  | Enable read/write protection on different sectors of the user Flash |
| CMD_BL_DISABLE_RW_PROTECT  | 0x5A  | Disable read/write protection on different sectors of the user Flash |
| CMD_BL_READ_FLASH_SECTOR_STATUS  | 0x5B  | Read the protection level status of all Flash sectors |
| CMD_BL_OTP_READ  | 0x5C  | Read the MCU OTP settings |
