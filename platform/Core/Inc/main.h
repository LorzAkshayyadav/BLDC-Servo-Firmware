/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define POS_ENC_CS_Pin GPIO_PIN_13
#define POS_ENC_CS_GPIO_Port GPIOC
#define POS_ENC_RESET_Pin GPIO_PIN_2
#define POS_ENC_RESET_GPIO_Port GPIOE
#define POS_ENC_GETSENS_Pin GPIO_PIN_9
#define POS_ENC_GETSENS_GPIO_Port GPIOB
#define POS_ENC_MISO_Pin GPIO_PIN_4
#define POS_ENC_MISO_GPIO_Port GPIOB
#define POS_ENC_CLK_Pin GPIO_PIN_3
#define POS_ENC_CLK_GPIO_Port GPIOB
#define MOTOR_ENC_CS_Pin GPIO_PIN_15
#define MOTOR_ENC_CS_GPIO_Port GPIOA
#define STO2_TIM15_BKIN_Pin GPIO_PIN_3
#define STO2_TIM15_BKIN_GPIO_Port GPIOE
#define MOTOR_ENC_GETSENS_Pin GPIO_PIN_6
#define MOTOR_ENC_GETSENS_GPIO_Port GPIOB
#define MOTOR_ENC_RESET_Pin GPIO_PIN_5
#define MOTOR_ENC_RESET_GPIO_Port GPIOD
#define MOTOR_ENC_EOT_Pin GPIO_PIN_2
#define MOTOR_ENC_EOT_GPIO_Port GPIOD
#define MOTOR_ENC_MISO_Pin GPIO_PIN_11
#define MOTOR_ENC_MISO_GPIO_Port GPIOC
#define MOTOR_ENC_CLK_Pin GPIO_PIN_10
#define MOTOR_ENC_CLK_GPIO_Port GPIOC
#define POS_ENC_EOT_Pin GPIO_PIN_4
#define POS_ENC_EOT_GPIO_Port GPIOE
#define POS_ENC_ERR_Pin GPIO_PIN_1
#define POS_ENC_ERR_GPIO_Port GPIOE
#define POS_ENC_MOSI_Pin GPIO_PIN_5
#define POS_ENC_MOSI_GPIO_Port GPIOB
#define MOTOR_ENC_ERR_Pin GPIO_PIN_3
#define MOTOR_ENC_ERR_GPIO_Port GPIOD
#define MOTOR_ENC_MOSI_Pin GPIO_PIN_12
#define MOTOR_ENC_MOSI_GPIO_Port GPIOC
#define LAN9253_SCK_Pin GPIO_PIN_9
#define LAN9253_SCK_GPIO_Port GPIOA
#define BRAKE_PWM_Pin GPIO_PIN_5
#define BRAKE_PWM_GPIO_Port GPIOE
#define LAN9253_RST_Pin GPIO_PIN_10
#define LAN9253_RST_GPIO_Port GPIOA
#define PHASE_A_ISENSE_Pin GPIO_PIN_2
#define PHASE_A_ISENSE_GPIO_Port GPIOC
#define STO2_TIM1_BKIN2_Pin GPIO_PIN_6
#define STO2_TIM1_BKIN2_GPIO_Port GPIOE
#define LAN9253_IRQ_Pin GPIO_PIN_9
#define LAN9253_IRQ_GPIO_Port GPIOC
#define LAN9253_IRQ_EXTI_IRQn EXTI9_5_IRQn
#define LAN9253_SYNC0_Pin GPIO_PIN_7
#define LAN9253_SYNC0_GPIO_Port GPIOC
#define PHASE_C_ISENSE_Pin GPIO_PIN_0
#define PHASE_C_ISENSE_GPIO_Port GPIOC
#define PHASE_B_ISENSE_Pin GPIO_PIN_1
#define PHASE_B_ISENSE_GPIO_Port GPIOC
#define PWM_INL_B_Pin GPIO_PIN_10
#define PWM_INL_B_GPIO_Port GPIOE
#define LAN9253_SI_Pin GPIO_PIN_15
#define LAN9253_SI_GPIO_Port GPIOB
#define LED_B_Pin GPIO_PIN_1
#define LED_B_GPIO_Port GPIOA
#define PWM_INH_B_Pin GPIO_PIN_11
#define PWM_INH_B_GPIO_Port GPIOE
#define LAN9253_SO_Pin GPIO_PIN_14
#define LAN9253_SO_GPIO_Port GPIOB
#define LED_G_Pin GPIO_PIN_2
#define LED_G_GPIO_Port GPIOA
#define STO1_TIM1_BKIN_Pin GPIO_PIN_6
#define STO1_TIM1_BKIN_GPIO_Port GPIOA
#define PWM_INL_A_Pin GPIO_PIN_8
#define PWM_INL_A_GPIO_Port GPIOE
#define PWM_INL_C_Pin GPIO_PIN_12
#define PWM_INL_C_GPIO_Port GPIOE
#define STO1_INPUT_MON_Pin GPIO_PIN_10
#define STO1_INPUT_MON_GPIO_Port GPIOB
#define STO1_INPUT_MON_EXTI_IRQn EXTI15_10_IRQn
#define STO2_INPUT_MON_Pin GPIO_PIN_13
#define STO2_INPUT_MON_GPIO_Port GPIOB
#define STO2_INPUT_MON_EXTI_IRQn EXTI15_10_IRQn
#define TORQ_SNS_RX_Pin GPIO_PIN_9
#define TORQ_SNS_RX_GPIO_Port GPIOD
#define LED_R_Pin GPIO_PIN_3
#define LED_R_GPIO_Port GPIOA
#define VBUS_SENSE_Pin GPIO_PIN_1
#define VBUS_SENSE_GPIO_Port GPIOB
#define PWM_INH_A_Pin GPIO_PIN_9
#define PWM_INH_A_GPIO_Port GPIOE
#define PWM_INH_C_Pin GPIO_PIN_13
#define PWM_INH_C_GPIO_Port GPIOE
#define STO1_MCU_CTRL_Pin GPIO_PIN_11
#define STO1_MCU_CTRL_GPIO_Port GPIOB
#define STO2_MCU_CTRL_Pin GPIO_PIN_12
#define STO2_MCU_CTRL_GPIO_Port GPIOB
#define TORQ_SNS_TX_Pin GPIO_PIN_8
#define TORQ_SNS_TX_GPIO_Port GPIOD
#define TORQ_SNS_DE_Pin GPIO_PIN_12
#define TORQ_SNS_DE_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
