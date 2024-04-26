/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32CubeIDE
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#include <stdint.h>

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

#include "stm32f429xx.h"
#include "reg_util.h"
#include "bsp_lcd.h"

void SystemClock_Setup(void);

int main(void) {
	SystemClock_Setup();
	BSP_LCD_Init();
	/* Loop forever */
	for (;;);
}

void SystemClock_Setup(void) {
	// RCC : RESET CLOCK CONTROLLER 리셋과 클럭을 관장하는 장치

	RCC_TypeDef *pRCC = RCC;
	FLASH_TypeDef *pFlash = FLASH;
	PWR_TypeDef *pPWR = PWR;

	//1. Program flash wait states
	REG_SET_VAL(pFlash->ACR, 0x5U, 0xFU, FLASH_ACR_LATENCY_Pos);

	//2. Over drive settings
	REG_SET_BIT(pRCC->APB1ENR, RCC_APB1ENR_PWREN_Pos); /*Enable clock for PWR register access*/
	REG_SET_VAL(pPWR->CR, 0x3, 0x3, PWR_CR_VOS_Pos); /*VOS = 0b11*/
	REG_SET_BIT(pPWR->CR, PWR_CR_ODEN_Pos); /* Activate over drive mode */
	while (!REG_READ_BIT(pPWR->CSR, PWR_CSR_ODRDY_Pos))
		; /* wait for overdrive ready*/
	REG_SET_BIT(pPWR->CR, PWR_CR_ODSWEN_Pos); /* Over drive switch enable*/

	//3. Setting up main PLL
	REG_SET_VAL(pRCC->PLLCFGR, 0x8U, 0x3FU, RCC_PLLCFGR_PLLM_Pos); /*PLL_M*/
	REG_SET_VAL(pRCC->PLLCFGR, 180U, 0x1FFU, RCC_PLLCFGR_PLLN_Pos); /*PLL_N*/
	REG_SET_VAL(pRCC->PLLCFGR, 0x00U, 0x3U, RCC_PLLCFGR_PLLP_Pos); /*PLL_P*/

	/////////////////This step is only required if you are using RGB interface ////////////
	//4. Setting up LCD_CLK using PLLSAI block
	REG_SET_VAL(pRCC->PLLSAICFGR, 50U, 0x1FFU, RCC_PLLSAICFGR_PLLSAIN_Pos); /*PLLSAI_N*/
	REG_SET_VAL(pRCC->PLLSAICFGR, 0x02U, 0x7U, RCC_PLLSAICFGR_PLLSAIR_Pos); /*PLLSAI_R*/
	/*LCD_CLK = 6.25MHz*/
	REG_SET_VAL(pRCC->DCKCFGR, 0x02U, 0x3U, RCC_DCKCFGR_PLLSAIDIVR_Pos); /*DIV*/
	REG_SET_BIT(pRCC->CR, RCC_CR_PLLSAION_Pos);
	while (!REG_READ_BIT(pRCC->CR, RCC_CR_PLLSAIRDY_Pos))
		;
	///////////////////////////////////////////////////////////////////////////////////////

	//5. Setting up AHB and APBx clocks
	REG_SET_VAL(pRCC->CFGR, 0U, 0xFU, RCC_CFGR_HPRE_Pos); /*AHB prescaler*/
	REG_SET_VAL(pRCC->CFGR, 0x5U, 0x7U, RCC_CFGR_PPRE1_Pos); /*APB1 prescaler*/
	REG_SET_VAL(pRCC->CFGR, 0x4U, 0x7U, RCC_CFGR_PPRE2_Pos); /*APB2 prescaler*/

	//6. Turn on PLL and wait for PLLCLK ready
	REG_SET_BIT(pRCC->CR, RCC_CR_PLLON_Pos);
	while (!REG_READ_BIT(pRCC->CR, RCC_CR_PLLRDY_Pos))
		;

	//7. Switch PLLCLK as SYSCLK
	REG_SET_VAL(pRCC->CFGR, 0x2U, 0x3U, RCC_CFGR_SW_Pos);
	while (!(REG_READ_VAL(pRCC->CFGR, 0x3U, RCC_CFGR_SWS_Pos) == 0x2U))
		;
}