#ifndef HARDWARE_CONFIG
#define HARDWARE_CONFIG

#include "compiler.h"
#include "stm32f0xx_gpio.h"
#include "stm32f0xx_i2c.h"
#include "stm32f0xx_spi.h"
#include "stm32f0xx_usart.h"
#include "common.h"

//PWM
#define PWM_FREQUENCY			17570
#define MOTOR_DRV_R				GPIO_Pin_4
#define MOTOR_DRV_R_AF_SRC		GPIO_PinSource4
#define MOTOR_DRV_L				GPIO_Pin_5
#define MOTOR_DRV_L_AF_SRC		GPIO_PinSource5
#define MOTOR_VAC_PUMP			GPIO_Pin_0
#define MOTOR_VAC_AF_SRC		GPIO_PinSource0
#define PWM_CH4_PB1_RESERVED	GPIO_PinSource1
#define MOTOR_PWM_GPIO_PORT		GPIOB				//TIM3 AF1 CH1->PB4  CH2->PB5 CH3->PB0 CH4->PB1
#define MOTOR_PWM_R				TIM3->CCR1
#define MOTOR_PWM_L				TIM3->CCR2


//UNKNOWN PERIPH YET
#define SENSE_PROX1				GPIO_Pin_6
#define SENSE_PROX2				GPIO_Pin_7
#define SENSE_PROX3				GPIO_Pin_8
#define SENSE_PROX4				GPIO_Pin_9
#define SENSE_PROX_PORT			GPIOB

//ADC
#define AN0					    GPIO_Pin_0
#define AN1					    GPIO_Pin_1
#define AN2					    GPIO_Pin_2
#define AN3					    GPIO_Pin_3
#define AN4					    GPIO_Pin_4
#define AN5					    GPIO_Pin_5
#define ANALOG_IN_PORT			GPIOC
#define NUMBER_OF_ADC_CHANNEL 	6
#define SENSE_BATT				0
#define SENSE_MOTOR_DRIVE_L		1
#define SENSE_MOTOR_DRIVE_R		2
#define SENSE_MOTOR_IMPELER		3
#define SENSE_MOTOR_VAC_PUMP	4

//DIGITAL OUT
#define LED1					GPIO_Pin_6
#define LED2					GPIO_Pin_7
#define LED3					GPIO_Pin_8
#define LED4					GPIO_Pin_9
#define MOTOR_DIR_R				GPIO_Pin_8
#define MOTOR_DIR_L				GPIO_Pin_9
#define MOTOR_IMPELER			GPIO_Pin_10
#define MOTOR_DISPENSER			GPIO_Pin_11
#define LED_PORT				GPIOC
#define MOTOR_DIR_PORT			GPIOC
#define MOTOR_DIG_GPIO_PORT		GPIOC

//DIGITAL INPUT
#define USER_PB1				GPIO_Pin_0
#define USER_PB2				GPIO_Pin_1
#define USER_PB_PORT			GPIOA

//DAC
#define	DAC_PIN					GPIO_Pin_4
#define DAC_PORT				GPIOA

//SPI
#define SPI_FLASH				SPI2
#define SPI_FLASH_CS			GPIO_Pin_12
#define SPI_FLASH_CLK			GPIO_Pin_13
#define SPI_FLASH_SCLK_AF_SRC	GPIO_PinSource13
#define SPI_FLASH_MISO			GPIO_Pin_14
#define SPI_FLASH_MISO_AF_SRC	GPIO_PinSource14
#define SPI_FLASH_MOSI			GPIO_Pin_15
#define SPI_FLASH_MOSI_AF_SRC	GPIO_PinSource15
#define SPI_FLASH_PORT			GPIOB

//USART
#define USART_DEBUG				USART1
#define USART_DEBUG_RCC_CLK		RCC_APB2Periph_USART1
#define USART_DEBUG_TX			GPIO_Pin_9
#define USART_DEBUG_AF_TX_SRC	GPIO_PinSource9
#define USART_DEBUG_RX			GPIO_Pin_10
#define USART_DEBUG_AF_RX_SRC	GPIO_PinSource10
#define USART_DEBUG_PORT		GPIOA

//I2C
#define I2C1_RCC_CLK			RCC_APB1Periph_I2C1
#define I2C1_SDA				GPIO_Pin_6
#define I2C1_AF_SDA_SRC			GPIO_PinSource6
#define I2C1_CLK				GPIO_Pin_7
#define I2C1_AF_CLK_SRC			GPIO_PinSource7
#define I2C1_RCC_PORT_CLK	    RCC_AHBPeriph_GPIOB
#define I2C1_PORT				GPIOB

#define PB1_STATE				GPIO_ReadInputDataBit(USER_PB_PORT,USER_PB1)
#define PB2_STATE				GPIO_ReadInputDataBit(USER_PB_PORT,USER_PB2)

#define BUMPER_FRONT_STATE		GPIO_ReadInputDataBit(SENSE_PROX_PORT, SENSE_PROX3)
#define BUMPER_REAR_STATE		GPIO_ReadInputDataBit(SENSE_PROX_PORT, SENSE_PROX4)

#define port_USART_DEBUG_busy_sending()  (USART_GetFlagStatus((USART_DEBUG),(USART_FLAG_TXE))==(RESET))
#define port_USART_DEBUG_no_data()       (USART_GetFlagStatus((USART_DEBUG),(USART_FLAG_RXNE))==(RESET))
#define port_USART_DEBUG_send_data(x)    USART_SendData((USART_DEBUG),(uint8_t)(x))
#define port_USART_DEBUG_receive_data()  USART_ReceiveData(USART_DEBUG)

#define GetMiliSecondCount()          portGetTickCnt()


int InitSystemTick();
int portGetTickCnt(void);
int InitGpio();
void UsartPrintf( char* format, ... );
int InitUsartDebug();
int InitI2c();
int InitSpi();
int InitPwm();
int InitAdc();
int InitDac();;
uint16_t* GetAdcData();


#endif //HARDWARE_CONFIG
