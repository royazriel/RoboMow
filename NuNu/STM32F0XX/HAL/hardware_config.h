#ifndef HARDWARE_CONFIG
#define HARDWARE_CONFIG

#include "compiler.h"
#include "stm32f0xx_gpio.h"
#include "stm32f0xx_i2c.h"
#include "stm32f0xx_spi.h"
#include "stm32f0xx_usart.h"

#define MOTOR_DRV_R				GPIO_Pin_4
#define MOTOR_DRV_R_AF_SRC		GPIO_PinSource4
#define MOTOR_DRV_L				GPIO_Pin_5
#define MOTOR_DRV_L_AF_SRC		GPIO_PinSource5
#define MOTOR_VAC_PUMP			GPIO_Pin_0
#define MOTOR_VAC_AF_SRC		GPIO_PinSource0
#define PWM_CH4_PB1_RESERVED	GPIO_PinSource1
#define MOTOR_IMPELER			GPIO_Pin_2
#define MOTOR_DISPENSER			GPIO_Pin_3
#define MOTOR_GPIO_PORT 		GPIOB				//TIM3 AF1 CH1->PB4  CH2->PB5 CH3->PB0 CH4->PB1

#define SENSE_PROX1				GPIO_Pin_6
#define SENSE_PROX2				GPIO_Pin_7
#define SENSE_PROX3				GPIO_Pin_8
#define SENSE_PROX4				GPIO_Pin_9
#define SENSE_BATT				GPIO_Pin_10
#define SENSE_MOTOR_DRIVE_L		GPIO_Pin_11
#define SENSE_MOTOR_DRIVE_R		GPIO_Pin_12
#define SENSE_MOTOR_IMPELER		GPIO_Pin_13
#define SENSE_MOTOR_VAC_PUMP	GPIO_Pin_14
#define SENSE_PORT				GPIOB

#define SPIx					SPI1
#define SPIx_CS					GPIO_Pin_4
#define SPIx_CLK				GPIO_Pin_5
#define SPIx_MISO				GPIO_Pin_6
#define SPIx_MOSI				GPIO_Pin_7
#define SPIx_PORT				GPIOA

#define USART_DEBUG				USART1
#define USART_DEBUG_RCC_CLK		RCC_APB2Periph_USART1
#define USART_DEBUG_TX			GPIO_Pin_9
#define USART_DEBUG_AF_TX_SRC	GPIO_PinSource9
#define USART_DEBUG_RX			GPIO_Pin_10
#define USART_DEBUG_AF_RX_SRC	GPIO_PinSource10
#define USART_DEBUG_PORT		GPIOA

#define I2C1_RCC_CLK			RCC_APB1Periph_I2C1
#define I2C1_SDA				GPIO_Pin_6
#define I2C1_AF_SDA_SRC			GPIO_PinSource6
#define I2C1_CLK				GPIO_Pin_7
#define I2C1_AF_CLK_SRC			GPIO_PinSource7
#define I2C1_RCC_PORT_CLK	    RCC_AHBPeriph_GPIOB
#define I2C1_PORT				GPIOB

#define LED1					GPIO_Pin_6
#define LED2					GPIO_Pin_7
#define LED3					GPIO_Pin_8
#define LED4					GPIO_Pin_9
#define MOTOR_DIR_R				GPIO_Pin_8
#define MOTOR_DIR_L				GPIO_Pin_9
#define LED_PORT				GPIOC

#define USER_PB1				GPIO_Pin_0
#define USER_PB2				GPIO_Pin_1
#define USER_PB_PORT			GPIOA

#define AUDIO_AMP				GPIO_Pin_0
#define AUDIO_AMP_PORT			GPIOD

#define PB1_STATE				GPIO_ReadInputDataBit(USER_PB_PORT,USER_PB1)
#define PB2_STATE				GPIO_ReadInputDataBit(USER_PB_PORT,USER_PB2)

#define port_USART_DEBUG_busy_sending()  (USART_GetFlagStatus((USART_DEBUG),(USART_FLAG_TXE))==(RESET))
#define port_USART_DEBUG_no_data()       (USART_GetFlagStatus((USART_DEBUG),(USART_FLAG_RXNE))==(RESET))
#define port_USART_DEBUG_send_data(x)    USART_SendData((USART_DEBUG),(uint8_t)(x))
#define port_USART_DEBUG_receive_data()  USART_ReceiveData(USART_DEBUG)

#define portGetTickCount()          portGetTickCnt()


int InitSystemTick();
int portGetTickCnt(void);
int InitGpio();
void UsartPrintf( char* format, ... );
int InitUsartDebug();
int InitI2c();
int InitSpi();
int InitPwm();


#endif //HARDWARE_CONFIG
