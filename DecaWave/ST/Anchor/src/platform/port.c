// -------------------------------------------------------------------------------------------------------------------
//
//  File: port.c -
//
//  Copyright 2011 (c) DecaWave Ltd, Dublin, Ireland.
//
//  All rights reserved.
//
//  Author: Ekawahyu Susilo, April 2011
//
// -------------------------------------------------------------------------------------------------------------------


#include "compiler.h"
#include "port.h"

#define rcc_init(x)					RCC_Configuration(x)
#define systick_init(x)				SysTick_Configuration(x)
#define rtc_init(x)					RTC_Configuration(x)
#define interrupt_init(x)			NVIC_Configuration(x)
#define usart_init(x)				USART_Configuration(x)
#define spi_init(x)					SPI_Configuration(x)
#define gpio_init(x)				GPIO_Configuration(x)
#define ethernet_init(x)			No_Configuration(x)
#define fs_init(x)					No_Configuration(x)
#define usb_init(x)					No_Configuration(x)
#define lcd_init(x)					No_Configuration(x)
#define touch_screen_init(x)		No_Configuration(x)

/* System tick 32 bit variable defined by the platform */
extern __IO unsigned long time32_incr;

int No_Configuration(void)
{
	return -1;
}

int portGetTickCnt(void)
{
	return time32_incr;
}

int SysTick_Configuration(void)
{
	if (SysTick_Config(SystemCoreClock / 6 / CLOCKS_PER_SEC))
	{
		/* Capture error */
		while (1);
	}
	NVIC_SetPriority (SysTick_IRQn, 5);

	return 0;
}

void RTC_Configuration(void)
{
}


int NVIC_DisableDECAIRQ(void)
{
	EXTI_InitTypeDef EXTI_InitStructure;

	/* Configure EXTI line */
	EXTI_InitStructure.EXTI_Line = DECAIRQ_EXTI;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;	//MPW3 IRQ polarity is high by default
	EXTI_InitStructure.EXTI_LineCmd = DECAIRQ_EXTI_NOIRQ;
	EXTI_Init(&EXTI_InitStructure);

	return 0;
}


int NVIC_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Configure PA0 pin as input floating */
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = DECAIRQ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	//GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(DECAIRQ_GPIO, &GPIO_InitStructure);

	/* Enable SYSCFG clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	/* Connect EXTI0 Line to PA0 pin */
	SYSCFG_EXTILineConfig(DECAIRQ_EXTI_PORT, DECAIRQ_EXTI_PIN);

	/* Configure EXTI0 line */
	EXTI_InitStructure.EXTI_Line = DECAIRQ_EXTI;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;	//MPW3 IRQ polarity is high by default
	EXTI_InitStructure.EXTI_LineCmd = DECAIRQ_EXTI_USEIRQ;
	EXTI_Init(&EXTI_InitStructure);

	/* Enable and set EXTI0 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = DECAIRQ_EXTI_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	return 0;
}

int RCC_Configuration(void)
{
	ErrorStatus HSEStartUpStatus;
	RCC_ClocksTypeDef RCC_ClockFreq;
	GPIO_InitTypeDef GPIO_InitStructure;


	/* RCC system reset(for debug purpose) */
	RCC_DeInit();

	/* Enable HSE */
	RCC_HSEConfig(RCC_HSE_ON);

	/* Wait till HSE is ready */
	HSEStartUpStatus = RCC_WaitForHSEStartUp();

	if(HSEStartUpStatus != ERROR)
	{

	}

	RCC_GetClocksFreq(&RCC_ClockFreq);

	return 0;
}

int USART_Configuration(void)
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

	/* Enable GPIO clock */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

	/* Enable USART clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	/* Connect PXx to USARTx_Tx */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_1);

	/* Connect PXx to USARTx_Rx */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_1);

	/* Configure USART Tx, Rx as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* USART configuration */
	USART_Init(USART1, &USART_InitStructure);

	/* Enable USART */
	USART_Cmd(USART1, ENABLE);
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    return 0;
}

void SPI_ChangeRate(uint16_t scalingfactor)
{
	uint16_t tmpreg = 0;

	/* Get the SPIx CR1 value */
	tmpreg = SPIx->CR1;

	/*clear the scaling bits*/
	tmpreg &= 0xFFC7;

	/*set the scaling bits*/
	tmpreg |= scalingfactor;

	/* Write to SPIx CR1 */
	SPIx->CR1 = tmpreg;
}

void SPI_ConfigFastRate(uint16_t scalingfactor)
{
	SPI_InitTypeDef SPI_InitStructure;
	SPI_I2S_DeInit(SPIx);

	// SPIx Mode setup
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;	 //
	//SPI_InitStructure.SPI_CPOL = SPI_CPOL_High; //
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	//SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge; //
	//SPI_InitStructure.SPI_NSS = SPI_NSS_Hard;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = scalingfactor; //sets BR[2:0] bits - baudrate in SPI_CR1 reg bits 4-6
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;

	SPI_Init(SPIx, &SPI_InitStructure);

	SPI_RxFIFOThresholdConfig(SPIx,SPI_RxFIFOThreshold_QF);  //fifo rx level 8bit

	// Enable SPIx
	SPI_Cmd(SPIx, ENABLE);
}

int SPI_Configuration(void)
{
	SPI_InitTypeDef SPI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

 /* Enable the peripheral clock of GPIOA */
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	GPIO_InitStructure.GPIO_Pin = SPIx_SCK;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_Init(SPIx_GPIO, &GPIO_InitStructure);

	/* Configure SD_SPI pins: MISO */
	GPIO_InitStructure.GPIO_Pin = SPIx_MISO;
	GPIO_Init(SPIx_GPIO, &GPIO_InitStructure);

	/* Configure SD_SPI pins: MOSI */
	GPIO_InitStructure.GPIO_Pin = SPIx_MOSI;
	GPIO_Init(SPIx_GPIO, &GPIO_InitStructure);

	// SPIx CS pin setup
	GPIO_InitStructure.GPIO_Pin = SPIx_CS;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(SPIx_CS_GPIO, &GPIO_InitStructure);


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
	//SPI_InitStructure.SPI_CPOL = SPI_CPOL_High; //
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	//SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge; //
	//SPI_InitStructure.SPI_NSS = SPI_NSS_Hard;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	//SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4; //sets BR[2:0] bits - baudrate in SPI_CR1 reg bits 4-6
	SPI_InitStructure.SPI_BaudRatePrescaler = SPIx_PRESCALER;
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


int SPI2_Configuration(void)
{
	SPI_InitTypeDef SPI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

    return 0;
}

int GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Enable GPIOs clocks */
	RCC_AHBPeriphClockCmd( RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB |
							RCC_AHBPeriph_GPIOC | RCC_AHBPeriph_GPIOD |
							RCC_AHBPeriph_GPIOF, ENABLE);


	//Enable GPIO used for User button
	GPIO_StructInit( &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN ;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//Enable GPIO used for User button
	GPIO_StructInit( &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = ANCHOR_ID_SELECTION;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN ;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(ANCHOR_ID_SELECTION_PORT, &GPIO_InitStructure);


	//Enable GPIO used for Leds switch setting
	GPIO_InitStructure.GPIO_Pin = LD3 | LD4 | LD5 | LD6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(LD_GPIOS, &GPIO_InitStructure);
	LD_GPIOS->BSRR=LD3 | LD4 | LD5 | LD6;

    return 0;
}


void reset_DW1000(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	//Enable GPIO Reset DECA
	GPIO_StructInit( &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = DW1000_RSTn;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT ;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(DW1000_RSTn_GPIO, &GPIO_InitStructure);

	//drive the RSTn pin low
	GPIO_ResetBits(DW1000_RSTn_GPIO, DW1000_RSTn);

	//put the pin back to tri-state ... as input
	GPIO_InitStructure.GPIO_Pin = DW1000_RSTn;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(DW1000_RSTn_GPIO, &GPIO_InitStructure);
	Sleep(1);
}





int ETH_GPIOConfigure(void)
{
	return 0;
}

int is_button_low(uint16_t GPIOpin)
{
	int result = 1;
#ifdef DW1000_EVK
	if (GPIO_ReadInputDataBit(TA_BOOT1_GPIO, TA_BOOT1))
		result = 0;
#endif
	return result;
}

//when switch (S1) is 'on' the pin is low
int is_switch_on(uint16_t GPIOpin)
{
	int result = 1;
#ifdef DW1000_EVK
	if (GPIO_ReadInputDataBit(TA_SW1_GPIO, GPIOpin))
		result = 0;
#endif
	return result;
}


void led_off (led_t led)
{
	switch (led)
	{
	case LED_PC6:
		GPIO_ResetBits(GPIOC, GPIO_Pin_6);
		break;
	case LED_PC7:
		GPIO_ResetBits(GPIOC, GPIO_Pin_7);
		break;
	case LED_PC8:
		GPIO_ResetBits(GPIOC, GPIO_Pin_8);
		break;
	case LED_PC9:
		GPIO_ResetBits(GPIOC, GPIO_Pin_9);
		break;
	case LED_ALL:
		GPIO_ResetBits(GPIOC, GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_6 | GPIO_Pin_7);
		break;
	default:
		// do nothing for undefined led number
		break;
	}
}

void led_on (led_t led)
{
	switch (led)
	{
	case LED_PC6:
		GPIO_SetBits(GPIOC, GPIO_Pin_6);
		break;
	case LED_PC7:
		GPIO_SetBits(GPIOC, GPIO_Pin_7);
		break;
	case LED_PC8:
		GPIO_SetBits(GPIOC, GPIO_Pin_8);
		break;
	case LED_PC9:
		GPIO_SetBits(GPIOC, GPIO_Pin_9);
		break;
	case LED_ALL:
		GPIO_SetBits(GPIOC, GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_6 | GPIO_Pin_7);
		break;
	default:
		// do nothing for undefined led number
		break;
	}
}



int is_IRQ_enabled(void)
{
	return ((   NVIC->ISER[((uint32_t)(DECAIRQ_EXTI_IRQn) >> 5)]
	           & (uint32_t)0x01 << (DECAIRQ_EXTI_IRQn & (uint8_t)0x1F)  ) ? 1 : 0) ;
}

int peripherals_init (void)
{
	rcc_init();
	gpio_init();
	//rtc_init();
	systick_init();
	interrupt_init();
	usart_init();
	//spi_init();
	//ethernet_init();
	//fs_init();
	//usb_init();
	//lcd_init();
	//touch_screen_init();
#if (DMA_ENABLE == 1)
	dma_init();	//init DMA for SPI only. Connection of SPI to DMA in read/write functions
#endif

	return 0;
}

void spi_peripheral_init()
{
	spi_init();
#ifdef DW1000_EVK
	//initialise SPI2 peripheral for LCD control
	SPI2_Configuration();

	port_LCD_RS_clear();

	port_LCD_RW_clear();
#endif
}
