/*
 * uapi_bootloader.h
 *
 *  Created on: Aug 9, 2023
 *      Author: kaustubh
 */

#ifndef BOOTLOADER_INC_UAPI_BOOTLOADER_H_
#define BOOTLOADER_INC_UAPI_BOOTLOADER_H_

#define USER_BUTTON_Pin GPIO_PIN_13
#define USER_BUTTON_GPIO_Port GPIOC
#define ONBOARD_LED_Pin GPIO_PIN_5
#define ONBOARD_LED_GPIO_Port GPIOA

void bootloader_read_data(void);
void bootloader_jump_to_user_app(void);

#endif /* BOOTLOADER_INC_UAPI_BOOTLOADER_H_ */
