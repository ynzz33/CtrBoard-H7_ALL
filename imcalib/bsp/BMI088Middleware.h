/*
 * 文件职责：BMI088的板级SPI/GPIO与延时适配（BMI088Middleware.h）。
 * 本头文件声明本模块的数据结构、常量和调用接口；实现及实际流程见对应源文件。
 * 输入/来源：BMI088驱动对片选、SPI字节传输和延时的请求。
 * 输出/去向：经HAL实现传感器访问的硬件操作。
 * 关键边界：只是底层适配；当前主初始化未启用BMI088主测量链，运行姿态来源以HI229为准。
 * 阅读入口：md/file-guide.md；参数当前值以源码为准。
 */

#ifndef BMI088MIDDLEWARE_H
#define BMI088MIDDLEWARE_H

#include "stdint.h"

#define BMI088_USE_SPI
//#define BMI088_USE_IIC

extern void BMI088_GPIO_init(void);
extern void BMI088_com_init(void);
extern void BMI088_delay_ms(uint16_t ms);
extern void BMI088_delay_us(uint16_t us);

#if defined(BMI088_USE_SPI)
extern void BMI088_ACCEL_NS_L(void);
extern void BMI088_ACCEL_NS_H(void);

extern void BMI088_GYRO_NS_L(void);
extern void BMI088_GYRO_NS_H(void);

extern uint8_t BMI088_read_write_byte(uint8_t reg);

#elif defined(BMI088_USE_IIC)

#endif

#endif
