/*
 * hardware_config.c
 *
 *  Created on: Dec 21, 2014
 *      Author: roy
 */

#include "hardware_config.h"
#include "common.h"

__IO uint16_t ADC_array[NUMBER_OF_ADC_CHANNEL]; //Array to store the values coming from the ADC and copied by DMA

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

	GPIO_InitStructure.GPIO_Pin = SPI_CLK;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_Init(SPI_PORT, &GPIO_InitStructure);

	/* Configure SD_SPI pins: MISO */
	GPIO_InitStructure.GPIO_Pin = SPI_MISO;
	GPIO_Init(SPI_PORT, &GPIO_InitStructure);

	/* Configure SD_SPI pins: MOSI */
	GPIO_InitStructure.GPIO_Pin = SPI_MOSI;
	GPIO_Init(SPI_PORT, &GPIO_InitStructure);

	// SPIx CS pin setup
	GPIO_InitStructure.GPIO_Pin = SPI_CS;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(SPI_PORT, &GPIO_InitStructure);


	GPIO_PinAFConfig( SPI_PORT, SPI_CLK_AF_SRC, GPIO_AF_0 );
	GPIO_PinAFConfig( SPI_PORT, SPI_MISO_AF_SRC, GPIO_AF_0 );
	GPIO_PinAFConfig( SPI_PORT, SPI_MOSI_AF_SRC, GPIO_AF_0 );

	RCC_APB1PeriphClockCmd( RCC_APB1Periph_SPI2, ENABLE);

#if 0
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
#endif

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

	SPI_Init(SPI2, &SPI_InitStructure);

	SPI_RxFIFOThresholdConfig(SPI2,SPI_RxFIFOThreshold_QF);  //fifo rx level 8bit

	// Disable SPIx SS Output
	SPI_SSOutputCmd(SPI2, DISABLE);

	// Enable SPIx
	SPI_Cmd(SPI2, ENABLE);

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
	GPIO_Init(MOTOR_PWM_GPIO_PORT, &GPIO_InitStructure);

	GPIO_PinAFConfig(MOTOR_PWM_GPIO_PORT, MOTOR_DRV_R_AF_SRC, GPIO_AF_1);  	//TIM3 ONLY
	GPIO_PinAFConfig(MOTOR_PWM_GPIO_PORT, MOTOR_DRV_L_AF_SRC, GPIO_AF_1);	//TIM3 ONLY

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

int InitAdc()
{
	ADC_InitTypeDef ADC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	DMA_InitTypeDef  DMA_InitStructure;

	/* ADC1 DeInit */
	ADC_DeInit(ADC1);

	/* Enable ADC and GPIO clocks ****************************************/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	/* Configure ADC1 Channel12 pin as analog input ******************************/
	GPIO_InitStructure.GPIO_Pin = AN0 | AN1 | AN2 | AN3 | AN4 |AN5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/* Initialize ADC structure */
	ADC_StructInit(&ADC_InitStructure);

	/* Configure the ADC1 in continous mode withe a resolutuion equal to 12 bits  */
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_TRGO;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_ScanDirection = ADC_ScanDirection_Upward;
	ADC_Init(ADC1, &ADC_InitStructure);

	/* Convert the ADC1 Channel 12 with 239.5 Cycles as sampling time */
	ADC_ChannelConfig(ADC1, ADC_Channel_10  | ADC_Channel_11 | ADC_Channel_12 | ADC_Channel_13 | ADC_Channel_14 | ADC_Channel_15, ADC_SampleTime_239_5Cycles);

	/* ADC Calibration */
	ADC_GetCalibrationFactor(ADC1);

	/* Enable DMA request after last transfer (OneShot-ADC mode) */
	ADC_DMARequestModeConfig(ADC1, ADC_DMAMode_Circular);

	/* Enable ADCperipheral[PerIdx] */
	ADC_Cmd(ADC1, ENABLE);

	/* Enable ADC_DMA */
	ADC_DMACmd(ADC1, ENABLE);

	/* Wait the ADCEN falg */
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_ADRDY));

	/* ADC1 regular Software Start Conv */
	ADC_StartOfConversion(ADC1);

	/* Enable DMA1 clock */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	/* DMA1 Stream1 channel1 configuration **************************************/
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) (&(ADC1->DR));
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)ADC_array;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = NUMBER_OF_ADC_CHANNEL;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

	DMA_Init(DMA1_Channel1, &DMA_InitStructure);
	DMA_Cmd(DMA1_Channel1, ENABLE);
}

uint16_t* GetAdcData()
{
	return ADC_array;
}
