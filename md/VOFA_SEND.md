# Vofa+ 调试发送模块

> 配合文件：[`Vofa_send.h`](../imcalib/user-lib/Vofa_send.h) 和 [`Vofa_send.c`](../imcalib/user-lib/Vofa_send.c)

---

## 1. 协议：FireWater

```
[float0][float1]...[floatN-1][0x00 0x00 0x80 0x7F]
  ↑ N × 4 字节 (小端)        ↑ 帧尾 (固定)
```

Vofa+ 通过帧尾 `0x00 0x00 0x80 0x7F` 识别一帧结束。

---

## 2. 使用方式

```c
/* 配置串口: 修改宏定义即可 */
#define VOFA_UART   &huart7

/* 发送 */
float data[] = {1.0f, 2.0f, 3.0f};
Vofa_Send(data, 3);
```

---

## 3. 实现

```c
static const uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7F};

void Vofa_Send(const float *data, uint8_t n)
{
    if (data == NULL || n == 0 || n > VOFA_MAX_CH) return;

    uint8_t buf[VOFA_MAX_CH * sizeof(float) + 4];
    uint16_t data_len = n * sizeof(float);

    memcpy(buf, data, data_len);
    memcpy(buf + data_len, tail, 4);

    HAL_UART_Transmit_DMA(VOFA_UART, buf, data_len + 4);
}
```

使用 DMA 发送，不阻塞 CPU。

---

## 4. 上位机配置

Vofa+ 设置：
- 协议：FireWater
- 串口：对应 COM 口
- 波特率：与 CubeMX 中 VOFA_UART 的波特率一致

## 当前后级 PID 调试通道

| 通道 | 内容 |
|------|------|
| 0~3 | 左大腿、左虚拟小腿、右大腿、右虚拟小腿反馈 |
| 4~7 | 四个腿部 PID 目标 |
| 8~11 | 四个腿部 PID 误差 |
| 12~15 | 四个腿部 PID 的 P 项 |
| 16~17 | 左右腿长 `l0` |
| 18~19 | 右腿两个 PID 的 D 项 |
| 20~23 | 四个腿部虚拟关节力矩 |
| 24~25 | 保留，当前为 0 |
| 26~29 | 左前髋、左后髋、右前髋、右后髋输出力矩 |
| 30 | DM 力矩发送成功标志 |
| 31 | bit0~3=DM 在线，bit4~5=DJI 在线，bit6=整车使能，bit7=力矩总开关 |

右侧极性在驱动解码后统一到机体坐标系；右侧 DM 力矩下发在驱动边界同步取反一次，后续模块禁止重复取反。
