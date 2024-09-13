/**
  ******************************************************************************
  * @file    myself_Oled
  * @author  from internet
  * @brief   128*64 oled drive
  *
  ******************************************************************************
  * @attention  stm32 HAL drive I2C in cubemx default configuration
  * 
  ******************************************************************************
  @verbatim
  * I2C_HandleTypeDef hi2c change yourself handle in .h
  * initialization function: WS_OLED_Init()
  * display string function: WS_OLED_Printf()
  @endverbatim
  ******************************************************************************
  */

#ifndef __myself_Oled_H__
#define __myself_Oled_H__

#include "main.h"
#include "i2c.h"
#include <stdlib.h>
#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>

#define hi2c1 hi2c1
#define IIC_PORT GPIOB
#define IIC_PIN_SCL GPIO_PIN_9
#define IIC_PIN_SDA GPIO_PIN_8

#define CLR_IIC_SCL() IIC_PORT->ODR &= ~IIC_PIN_SCL // CLK
#define SET_IIC_SCL() IIC_PORT->ODR |= IIC_PIN_SCL

#define CLR_IIC_SDA() IIC_PORT->ODR &= ~IIC_PIN_SDA // DIN
#define SET_IIC_SDA() IIC_PORT->ODR |= IIC_PIN_SDA

#define OLED_DIS_MODE_5X7 0
#define OLED_DIS_MODE_8X16 1

void openOledDis(void);
void closeOledDis(void);
void setOLEDClear(void);
char writeOledBuf(char adds, unsigned char *data, int len);
void WS_OLED_Dis_String(int x, int y, char *string, char disMode);


void WS_OLED_Init(void);

/**
  * @brief  show string in oled
  * @param  x 0 - 127
  * @param  y 0 - 7
  * @param  disMode 0: 6*7,  1: 8*16
  * @param  *p the first address of an ASCII string
  * @retval None
  */
void WS_OLED_Printf(int x, int y, char disMode, char *p, ...);

#endif
