# DBUS 遥控器解析

> 配合文件：[`dr16.h`](../imcalib/user-lib/dr16.h)、[`dr16.c`](../imcalib/user-lib/dr16.c)

---

## 1. 协议

DR16 遥控器 DBUS 协议，18 字节定长帧，100kbps（标准 DBUS）。

---

## 2. 数据结构

```c
typedef struct {
    int16_t  ch0;          /* 遥杆右X */
    int16_t  ch1;          /* 遥杆右Y */
    int16_t  ch2;          /* 遥杆左X */
    int16_t  ch3;          /* 遥杆左Y */
    int16_t  wheel;        /* 左侧拨轮 */
    uint8_t  s1;           /* 左拨杆 */
    uint8_t  s2;           /* 右拨杆 */
    int16_t  mx;           /* 鼠标X */
    int16_t  my;           /* 鼠标Y */
    int16_t  mz;           /* 鼠标Z */
    uint8_t  ml;           /* 鼠标左键 */
    uint8_t  mr;           /* 鼠标右键 */
    uint16_t key;          /* 键盘按键 */
    bool     online;       /* 在线标志 */
    uint32_t last_rx_tick; /* 最后接收时间戳 */
} dr16_t;
```

遥杆范围：-660 ~ +660，中值 0
拨轮范围：-660 ~ +660，中值 0

---

## 3. 使用方式

```c
#include "dr16.h"

/* 任务循环中 */
DR16_Process();

/* 读取数据 */
if (dr16.s1 == DR16_SW_DOWN) { ... }
int16_t ch0 = dr16.ch0;

/* 在线检测 */
if (DR16_Online()) { ... }

/* 快照 (避免跨帧混值) */
dr16_t remote = DR16_Snapshot();

/* 死区滤波 */
int16_t value = DR16_Deadline(raw, 20);
```

---

## 4. 内部流程

```
dbus_rx.flag == 1 ?
    ↓ yes
帧长校验 (>=18，尾部多余字节忽略)
    ↓
DR16_Parse: 解析 18 字节 → dr16_t
    ↓
范围校验: ch0-3/wheel ∈ [-660,660], s1/s2 ∈ [1,3]
    ↓
写入 dr16 结构体 + 更新 last_rx_tick
```

---

## 5. 通道映射

| 字段 | 解析位置 | 范围 | 用途 |
|------|:--------:|:----:|------|
| ch0 | buf[0..1] | ±660 | yaw 指令 |
| ch1 | buf[1..2] | ±660 | 轮子速度 (手动遥操) |
| ch2 | buf[2..4] | ±660 | (未用) |
| ch3 | buf[4..5] | ±660 | 前进/后退 + 大腿偏移 |
| wheel | buf[16..17] | ±660 | 高度 + 小腿偏移 |
| s1 | buf[5]>>4 | 1/2/3 | 使能控制 |
| s2 | buf[5]>>4 | 1/2/3 | 模式选择 (未接线) |
| mx/my/mz | buf[6..11] | int16 | (未用) |
| ml/mr | buf[12..13] | 0/1 | (未用) |
| key | buf[14..15] | uint16 | (未用) |

---

## 6. 拨杆语义

| 拨杆 | 值 | 宏 | 作用 |
|:----:|:--:|:--:|------|
| s1 DOWN | 2 | `DR16_SW_LEFT_DOWN` | 失能 |
| s1 MID | 3 | `DR16_SW_LEFT_MID` | 使能 |
| s1 UP | 1 | `DR16_SW_LEFT_UP` | 使能 |
| s2 | — | — | 模式选择 (未接线) |

---

## 7. 遥控指令映射

**RL 观测指令** (task_policy.c):

| 通道 | 输入 | 缩放 | 观测索引 |
|------|------|------|:--------:|
| ch3 | 前进/后退 | `RC_Axis(ch3) × REMOTE_COMMAND_SCALE` | obs[6] |
| ch0 | yaw 旋转 | `RC_Axis(ch0) × REMOTE_COMMAND_SCALE` | obs[7] |
| wheel | 高度 | `RC_Axis(wheel) × REMOTE_COMMAND_SCALE` | obs[8] |

**手动遥操** (task_policy.c):

| 通道 | 输入 | 缩放 | 作用 |
|------|------|------|------|
| ch3 | 大腿偏移 | `RC_Axis(ch3) × 4.0` | 叠加到 base_action[0,3] |
| wheel | 小腿偏移 | `RC_Axis(wheel) × 4.0` | 叠加到 base_action[1,4] |
| ch1 | 轮子速度 | `RC_Axis_Wheel(ch1) × 4.0` | 直接赋值 (宽死区100) |

**归一化函数**:
- `RC_Axis()` — 死区 20，限幅 ±660 → [-1, 1]
- `RC_Axis_Wheel()` — 死区 100，限幅 ±660 → [-1, 1]

---

## 8. 关键常量

| 常量 | 值 | 说明 |
|------|:--:|------|
| `DR16_FRAME_LEN` | 18 | 帧长度 |
| `DR16_OFFLINE_MS` | 100 | 超时 (ms) |
| `DR16_CH_LIMIT` | 660 | 通道最大绝对值 |
| `DR16_SW_UP` | 1 | 拨杆上位 |
| `DR16_SW_MID` | 3 | 拨杆中位 |
| `DR16_SW_DOWN` | 2 | 拨杆下位 |
| `REMOTE_COMMAND_SCALE` | 3.0f | RL 指令缩放 |

---

## 9. 关键函数

| 函数 | 作用 |
|------|------|
| `DR16_Init()` | UART9+DMA 启动 |
| `DR16_Process()` | 解析一帧写入 dr16 |
| `DR16_Online()` | 100ms 超时检测 |
| `DR16_Deadline()` | 死区滤波 |
| `DR16_Snapshot()` | 返回 dr16 副本 (避免跨帧) |
