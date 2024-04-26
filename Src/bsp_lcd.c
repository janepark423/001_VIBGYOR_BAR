/*
 * bsp_lcd.c
 *
 *  Created on: Apr 26, 2024
 *      Author: ETL
 */
#include "stm32f429xx.h"

#ifdef STM32F429I_DISC1
#define SPI						SPI5
#define LCD_SCL_PIN				GPIO_PIN_7
#define LCD_SCL_PORT			GPIOF
#define LCD_SDA_PIN				GPIO_PIN_9
#define LCD_SDA_PORT			GPIOF
#define LCD_RESX_PIN			GPIO_PIN_7
#define LCD_RESX_PORT			GPIOA
#define LCD_CSX_PIN				GPIO_PIN_2
#define LCD_CSX_PORT			GPIOC
#define LCD_DCX_PIN			    GPIO_PIN_13
#define LCD_DCX_PORT		    GPIOD

#elif defined(STM32F407)
	#define SPI						SPI2
	#define LCD_SCL_PIN				GPIO_PIN_13
	#define LCD_SCL_PORT			GPIOB
	#define LCD_SDI_PIN				GPIO_PIN_15
	#define LCD_SDI_PORT			GPIOB
	#define LCD_SDO_PIN				GPIO_PIN_2
	#define LCD_SDO_PORT			GPIOC
	#define LCD_RESX_PIN			GPIO_PIN_10
	#define LCD_RESX_PORT			GPIOD
	#define LCD_CSX_PIN				GPIO_PIN_11
	#define LCD_CSX_PORT			GPIOD
	#define LCD_DCX_PIN				GPIO_PIN_9
	#define LCD_DCX_PORT			GPIOD

#else
	#error"Supported device is not selected"
#endif

void LCD_Pin_Init(void);
void LCD_SPI_Init(void);
void LCD_Reset(void);
void LCD_Config(void);
void LCD_Write_Cmd(uint8_t cmd);
void LCD_SPI_Enable(void);
void LCD_Write_Data(uint8_t *buffer, uint32_t len);

void BSP_LCD_Init(void) {
	LCD_Pin_Init();
	LCD_SPI_Init();
	LCD_SPI_Enable();
	LCD_Reset();
	LCD_Config();
}

void LCD_Pin_Init(void) {
	RCC_TypeDef *pRCC = RCC;
	GPIO_TypeDef *pGPIOA = GPIOA;

	REG_SET_BIT(pRCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN_Pos);
	REG_SET_BIT(pRCC->AHB1ENR, RCC_AHB1ENR_GPIOCEN_Pos);
	REG_SET_BIT(pRCC->AHB1ENR, RCC_AHB1ENR_GPIODEN_Pos);

	REG_SET_VAL(pGPIOA->MODER, 0x1U, 0x3, (LCD_RESX_PIN * 2U));
	REG_CLR_BIT(pGPIOA->OTYPER, LCD_RESX_PIN);
	REG_SET_VAL(pGPIOA->OSPEEDR, 2U, 0x3U, GPIO_OSPEEDR_OSPEED7_Pos);
}
