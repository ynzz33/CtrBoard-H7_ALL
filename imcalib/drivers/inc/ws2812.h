/*
 * 将RGB亮度编码为定时器PWM序列并用DMA发送，提供后台彩虹闪烁状态灯。
 * 静态发送缓存仅在DMA空闲时更新，颜色变化不参与控制许可。
 */

#ifndef __WS2812_H__
#define __WS2812_H__
 
 
#include "main.h" 


#define WS2812_SPI_UNIT     hspi6
extern SPI_HandleTypeDef WS2812_SPI_UNIT;
 
void WS2812_Ctrl(uint8_t r, uint8_t g, uint8_t b);
void WS2812_RainbowBlink(void);
#endif
