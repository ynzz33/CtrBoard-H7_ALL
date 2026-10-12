/*
 * 文件职责：SPI编码的WS2812灯效发送（ws2812.h）。
 * 本头文件声明本模块的数据结构、常量和调用接口；实现及实际流程见对应源文件。
 * 输入/来源：后台周期调用、RGB/色相和单调时间预算。
 * 输出/去向：编码GRB为SPI波形并显示彩虹闪烁。
 * 关键边界：SPI忙或预算不足时返回；灯效在后台而非commTask中运行，不承担故障许可决策。
 * 阅读入口：md/file-guide.md；参数当前值以源码为准。
 */

#ifndef __WS2812_H__
#define __WS2812_H__
 
 
#include "main.h" 


#define WS2812_SPI_UNIT     hspi6
extern SPI_HandleTypeDef WS2812_SPI_UNIT;
 
void WS2812_Ctrl(uint8_t r, uint8_t g, uint8_t b);
void WS2812_RainbowBlink(void);
#endif
