# UART IDLE + DMA 接收框架

> 配合文件：[`uart_idle.h`](../imcalib/user-lib/uart_idle.h)、[`uart_idle.c`](../imcalib/user-lib/uart_idle.c)

---

## 1. 架构

```
uart_idle.c/h    ← 底层框架 (ISR + 初始化)
dbus.c/h         ← 设备层 (遥控器: 判断+拷贝+解析)
debug_rx.c/h     ← 设备层 (调试串口: 判断+拷贝+解析)
```

---

## 2. 数据流

```
USART1 RX (PA10) → DMA1_Stream3 → dbus_rx.dma_buf (Circular)
UART7  RX (PE7)  → DMA1_Stream2 → debug_rx.dma_buf (Circular)
    ↓ IDLE 中断 (一帧结束)
UART_Idle_Isr
    ↓ 清错误 → 快照 → 置 flag
dbus_rx.flag = 1 / debug_rx.flag = 1
    ↓ 任务层调用 XXX_Process()
判断 flag → 拷贝 → 解析
```

---

## 3. 底层 API (uart_idle.c/h)

```c
/* 结构体 */
typedef struct {
    UART_HandleTypeDef *huart;
    DMA_HandleTypeDef  *hdma_rx;
    uint8_t  dma_buf[DEBUG_BUF_SIZE];
    uint8_t  isr_buf[DEBUG_BUF_SIZE];
    volatile uint16_t isr_len;
    volatile uint8_t  flag;
    uint16_t buf_size;
    UART_Parse_cb parse;
} UART_Rx_t;

/* 初始化 */
UART_Rx_Init(&dbus_rx, DBUS_HUART, DBUS_DMA_RX, DBUS_BUF_SIZE, DBUS_Parse);

/* ISR */
UART_Idle_Isr(&huart1, &dbus_rx);
```

---

## 4. 设备层 API (dbus.c/h)

```c
/* 任务里只需要调这个 */
DBUS_Process();

/* 内部实现 */
void DBUS_Process(void) {
    if (!dbus_rx.flag) return;   ← 判断是否收到一帧
    dbus_rx.flag = 0;
    memcpy(buf, dbus_rx.isr_buf, len);  ← 拷贝
    DBUS_Parse(buf, len);                ← 解析
}
```

---

## 5. 为什么用 IDLE 中断

DMA 全传输中断只在 buffer 收满时触发。如果一帧没有填满 buffer，中断不会来，数据就"卡"在 DMA buffer 里。UART IDLE 中断在线上无数据时触发，即一帧发完就通知 CPU。

---

## 6. 为什么关 DMA 中断

HAL 默认的 `HAL_UART_Receive_DMA()` 会开启 DMA 的半传输(HT)和全传输(TC)中断。这两个中断我们不需要（靠 IDLE 就够了），留着会产生无用中断。

---

## 7. ISR 中的错误处理

不诊断错误原因，只关心帧是否对齐。串口线上的干扰无法在软件层面修复，清掉标志等下一帧。

---

## 8. CubeMX 配置要求

| 配置项 | USART1 (遥控) | UART7 (调试) |
|--------|--------------|-------------|
| Mode | 异步 | 异步 |
| DMA RX | Circular | Circular |
| NVIC | 全局中断使能 | 全局中断使能 |

---

## 9. 扩展新设备

1. 写一个 `XXX_Parse(data, len)` 回调
2. 声明 `UART_Rx_t xxx_rx` + 宏定义
3. `UART_Rx_Init(&xxx_rx, ...)` 初始化
4. 中断里加 `UART_Idle_Isr(&huartN, &xxx_rx)`
5. 写 `XXX_Process()` 封装判断+拷贝+解析
6. 任务里调 `XXX_Process()`
