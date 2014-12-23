/*
 * hardware_config.c
 *
 *  Created on: Dec 21, 2014
 *      Author: roy
 */

#include "hardware_config.h"
#include "common.h"

/* System tick 32 bit variable defined by the platform */
extern __IO unsigned long time32_incr;

int portGetTickCnt(void)
{
	return time32_incr;
}

int InitSystemTick()
{
	if (SysTick_Config(SystemCoreClock / CLOCKS_PER_SEC))
	{
		/* Capture error */
		while (1);
	}
	NVIC_SetPriority (SysTick_IRQn, 5);

	return 0;
}

int InitGpio()
{
	ErrorStatus HSEStartUpStatus;
	RCC_ClocksTypeDef RCC_ClockFreq;
	GPIO_InitTypeDef GPIO_InitStructure;

	/* RCC system reset(for debug purpose) */
	RCC_DeInit();

	/* Enable HSE */
	RCC_HSEConfig( RCC_HSE_ON );

	/* Wait till HSE is ready */
	HSEStartUpStatus = RCC_WaitForHSEStartUp();

	if(HSEStartUpStatus != ERROR)
	{

	}

	RCC_GetClocksFreq( &RCC_ClockFreq );
	RCC_AHBPeriphClockCmd(	RCC_AHBPeriph_GPIOA |	RCC_AHBPeriph_GPIOB |
							RCC_AHBPeriph_GPIOC |	RCC_AHBPeriph_GPIOD |
							RCC_AHBPeriph_GPIOF	, ENABLE);

	//Enable GPIO used for User button
	GPIO_StructInit( &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = USER_PB1 | USER_PB2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN ;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(USER_PB_PORT, &GPIO_InitStructure);


	//Enable GPIO used for Leds switch setting
	GPIO_InitStructure.GPIO_Pin = LED1 | LED2 | LED3 |LED4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(LED_PORT, &GPIO_InitStructure);
	LED_PORT->BSRR = LED1 | LED2 | LED3 | LED4;

	return 0;
}

int InitI2c()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	I2C_InitTypeDef I2C_InitStructure;

	/* Enable sEE_I2C Periph clock */
	RCC_APB1PeriphClockCmd( I2C1_RCC_CLK, ENABLE);

	RCC_I2CCLKConfig(RCC_I2C1CLK_HSI);

	/* Connect SCL and SDA pins to I2C alternate */
	GPIO_PinAFConfig(I2C1_PORT, I2C1_AF_SDA_SRC, GPIO_AF_1);
	GPIO_PinAFConfig(I2C1_PORT, I2C1_AF_CLK_SRC, GPIO_AF_1);

	/* GPIO configuration */
	/* Configure SCL pin */
	GPIO_InitStructure.GPIO_Pin = I2C1_CLK;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(I2C1_PORT, &GPIO_InitStructure);

	/* Configure SDA pin */
	GPIO_InitStructure.GPIO_Pin = I2C1_SDA;
	GPIO_Init(I2C1_PORT, &GPIO_InitStructure);

	RCC_APB1PeriphClockCmd( I2C1_RCC_CLK, ENABLE );

	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_AnalogFilter = I2C_AnalogFilter_Enable;
	I2C_InitStructure.I2C_DigitalFilter = 0x00;
	I2C_InitStructure.I2C_OwnAddress1 = 0xAB;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_Timing =  0x40B22536;

	/* Apply sI2C configuration after enabling it */
	I2C_Init(I2C1, &I2C_InitStructure);

	/* I2C Peripheral Enable */
	I2C_Cmd(I2C1, ENABLE);


	return 0;
}

void UsartPrintf( char* format, ... )
{
	char buffer[255];
	va_list aptr;
	int ret;

	va_start(aptr, format);
	ret = vsprintf(buffer, format, aptr);
	va_end(aptr);

	int size = strlen( buffer );
	for(int i=0; i< size;i++ )
	{
		while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
		USART_SendData( USART_DEBUG,buffer[i]);
	}
}

int InitUsartDebug()
{
	NVIC_InitTypeDef NVIC_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	//UART init
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	/* Enable USART clock */
	RCC_APB2PeriphClockCmd(USART_DEBUG_RCC_CLK, ENABLE);

	/* Connect PXx to USARTx_Tx */
	GPIO_PinAFConfig( USART_DEBUG_PORT, USART_DEBUG_AF_TX_SRC, GPIO_AF_1 );

	/* Connect PXx to USARTx_Rx */
	GPIO_PinAFConfig( USART_DEBUG_PORT, USART_DEBUG_AF_RX_SRC, GPIO_AF_1 );


	/* Configure USART Tx, Rx as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = USART_DEBUG_TX | USART_DEBUG_RX;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init( USART_DEBUG_PORT, &GPIO_InitStructure );

	/* USART configuration */
	USART_Init( USART_DEBUG, &USART_InitStructure );

	/* Enable USART */
	USART_Cmd( USART_DEBUG, ENABLE );
	USART_ITConfig( USART_DEBUG, USART_IT_RXNE, ENABLE );

    return 0;
}

int InitSpi()
{
	SPI_InitTypeDef SPI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

 /* Enable the peripheral clock of GPIOA */
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	GPIO_InitStructure.GPIO_Pin = SPIx_CLK;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_Init(SPIx_PORT, &GPIO_InitStructure);

	/* Configure SD_SPI pins: MISO */
	GPIO_InitStructure.GPIO_Pin = SPIx_MISO;
	GPIO_Init(SPIx_PORT, &GPIO_InitStructure);

	/* Configure SD_SPI pins: MOSI */
	GPIO_InitStructure.GPIO_Pin = SPIx_MOSI;
	GPIO_Init(SPIx_PORT, &GPIO_InitStructure);

	// SPIx CS pin setup
	GPIO_InitStructure.GPIO_Pin = SPIx_CS;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(SPIx_PORT, &GPIO_InitStructure);


	/* (1) Select AF mode (10) on PA4, PA5, PA6, PA7 */
	/* (2) AF0 for SPI1 signals */
	GPIOA->MODER = (GPIOA->MODER
				  & ~( GPIO_MODER_MODER5 |  GPIO_MODER_MODER6 |\
					  GPIO_MODER_MODER7))\
				  | ( GPIO_MODER_MODER5_1 | GPIO_MODER_MODER6_1 |\
					  GPIO_MODER_MODER7_1); /* (1) */
	GPIOA->AFR[0] = (GPIOA->AFR[0] & \
				   ~( GPIO_AFRL_AFRL5 | GPIO_AFRL_AFRL6 |\
						   GPIO_AFRL_AFRL7)); /* (2) */

	/* Enable the peripheral clock SPI1 */
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

	// SPIx Mode setup
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;	 //
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;

	SPI_Init(SPI1, &SPI_InitStructure);

	SPI_RxFIFOThresholdConfig(SPI1,SPI_RxFIFOThreshold_QF);  //fifo rx level 8bit

	// Disable SPIx SS Output
	SPI_SSOutputCmd(SPI1, DISABLE);

	// Enable SPIx
	SPI_Cmd(SPI1, ENABLE);

    return 0;
}

int InitPwm()
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;

	/* GPIOA Configuration: Channel 1, 2, 3 and 4 as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = MOTOR_DRV_R | MOTOR_DRV_L;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
	GPIO_Init(MOTOR_GPIO_PORT, &GPIO_InitStructure);

	GPIO_PinAFConfig(MOTOR_GPIO_PORT, MOTOR_DRV_R_AF_SRC, GPIO_AF_1);  	//TIM3 ONLY
	GPIO_PinAFConfig(MOTOR_GPIO_PORT, MOTOR_DRV_L_AF_SRC, GPIO_AF_1);	//TIM3 ONLY

	RCC_APB1PeriphClockCmd(RCC_APB1ENR_TIM3EN, ENABLE);
//
	/* Time Base configuration */
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = (SystemCoreClock/PWM_FREQUENCY) - 1;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;

	TIM_OCInitStructure.TIM_Pulse = 500;
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);

	TIM_OCInitStructure.TIM_Pulse = 500;
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);

	/* TIM1 counter enable */
	TIM_Cmd(TIM3, ENABLE);

	/* TIM1 Main Output Enable */
	//TIM_CtrlPWMOutputs(TIM3, ENABLE);  //TODO: this function exist in sample in our code its exiting
	return 0;
}
