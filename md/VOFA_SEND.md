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
#define VOFA_UART   &huart8
#define VOFA_MAX_CH 48   /* 最大通道数 */

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

---

## 5. 常量

| 常量 | 值 | 说明 |
|------|:--:|------|
| `VOFA_MAX_CH` | 48 | 最大通道数 |
| `VOFA_UART` | `&huart8` | 发送串口 |

---

## 当前 VOFA 通道布局（30 通道）

发送频率：200Hz（commTask 1kHz，每 5 周期发一次）

| 通道 | 内容 | 单位 |
|:----:|------|------|
| 0 | online_mask（bit0=IMU, bit1=RC, bit2~5=DM, bit6~7=DJI） | - |
| 1 | pitch (IMU 欧拉角) | rad |
| 2 | leg_valid（左+右×2） | - |
| 3 | motor_enabled + base_action_locked×2 | - |
| 4 | 左大腿当前角 | rad |
| 5 | 左大腿目标角 | rad |
| 6 | 左大腿误差（环绕） | rad |
| 7 | 左大腿虚拟力矩 | Nm |
| 8 | 左小腿当前角 | rad |
| 9 | 左小腿目标角 | rad |
| 10 | 左小腿误差（环绕） | rad |
| 11 | 左小腿虚拟力矩 | Nm |
| 12 | 右大腿当前角 | rad |
| 13 | 右大腿目标角 | rad |
| 14 | 右大腿误差（环绕） | rad |
| 15 | 右大腿虚拟力矩 | Nm |
| 16 | 右小腿当前角 | rad |
| 17 | 右小腿目标角 | rad |
| 18 | 右小腿误差（环绕） | rad |
| 19 | 右小腿虚拟力矩 | Nm |
| 20 | DM 左前髋输出力矩 | Nm |
| 21 | DM 左后髋输出力矩 | Nm |
| 22 | DM 右前髋输出力矩 | Nm |
| 23 | DM 右后髋输出力矩 | Nm |
| 24 | 左轮目标速度 | rad/s |
| 25 | 左轮当前速度 | rad/s |
| 26 | 左轮虚拟力矩 | Nm |
| 27 | 右轮目标速度 | rad/s |
| 28 | 右轮当前速度 | rad/s |
| 29 | 右轮虚拟力矩 | Nm |

右侧极性在驱动解码后统一到机体坐标系；右侧 DM 力矩下发在驱动边界同步取反一次，后续模块禁止重复取反。
