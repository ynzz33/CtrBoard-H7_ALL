# DBUS 遥控器解析

> 配合文件：[`dbus.h`](../imcalib/user-lib/dbus.h)、[`dbus.c`](../imcalib/user-lib/dbus.c)

---

## 1. 协议

DR16 遥控器 DBUS 协议，18 字节定长帧，100kbps（标准 DBUS）或 921600（部分兼容设备）。

---

## 2. 数据结构

```c
typedef struct {
    int16_t ch0;     /* 遥杆通道0 */
    int16_t ch1;
    int16_t ch2;
    int16_t ch3;
    uint8_t s1;      /* 开关 1/2/3 */
    uint8_t s2;
    int16_t mx;      /* 鼠标 */
    int16_t my;
    int16_t mz;
    uint8_t ml;      /* 鼠标按键 */
    uint8_t mr;
    uint16_t key;    /* 键盘 */
    int16_t wheel;   /* 拨轮 */
} DBUS_Data_t;
```

遥杆范围：364 ~ 1684，中值 1024

---

## 3. 使用方式

```c
#include "dbus.h"

/* 任务循环中 */
DBUS_Process();

/* 读取数据 */
if (dbus_data.s1 == 1) { ... }
int16_t ch0 = dbus_data.ch0;
```

---

## 4. 内部流程

```
dbus_rx.flag == 1 ?
    ↓ yes
拷贝 isr_buf → 局部 buf
    ↓
DBUS_Parse: 解析 18 字节 → DBUS_Data_t
```
