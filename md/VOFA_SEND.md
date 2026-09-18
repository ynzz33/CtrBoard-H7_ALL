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
#define VOFA_MAX_CH 32   /* 最大通道数 */

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
| `VOFA_MAX_CH` | 32 | 最大通道数（上限） |
| `VOFA_PORT` | 8 | 发送串口编号：**8 = UART8（默认）/ 1 = USART1**；可用 `-DVOFA_PORT=n` 覆盖 |
| `VOFA_UART` | `&huart8` | 由 `VOFA_PORT` 推导 |

可选串口只有两个（都是 921600 + TX DMA NORMAL）：

| 口 | 状态 | 说明 |
|:--:|------|------|
| UART8 | 默认 | 当前 VOFA 口 |
| USART1 | 空闲 | 可直接切；CubeMX 已初始化且 DMA1_Stream6 已绑定 |

**不能选**：UART7（被 HI229 占用，且 TX DMA 是 CIRCULAR 模式）、UART9（被 DR16 占用，没有 TX DMA）。
填别的数字会在编译期报错（`#error VOFA_PORT ...`）。

---

## 当前 VOFA 通道布局（27 通道，上限 32）

发送频率：200Hz（commTask 1kHz，每 5 周期发一次）

| 通道 | 内容 | 单位 |
|:----:|------|:----:|
| 0 | 在线掩码（bit0=IMU, bit1=遥控, bit2~5=四台 DM 腿电机, bit6~7=两个轮 DJI） | - |
| 1 | 解算有效掩码（1=左腿, 2=右腿, 3=两腿都有效） | - |
| 2 | 控制策略（0=手动遥操/RL, 1=LQR, 2=测试模式） | - |
| 3~6 | 电机**原始解码角**：前左 / 后左 / 前右 / 后右 | rad |
| 7~10 | 电机**零点后角度**（实际进解算、进 PID 的值）：前左 / 后左 / 前右 / 后右 | rad |
| 11~14 | 电机速度：前左 / 后左 / 前右 / 后右 | rad/s |
| 15~18 | 左腿解算：腿长 / 腿摆倾角 / 大腿角 / 虚拟小腿角 | m, rad |
| 19~22 | 右腿解算：腿长 / 腿摆倾角 / 大腿角 / 虚拟小腿角 | m, rad |
| 23~26 | 下发力矩：前左 / 后左 / 前右 / 后右 | Nm |

帧长：27×4+4 = **112 字节**；`VOFA_MAX_CH` = 32（上限，实际发 27 路）。

### 判读方法

- **ch1 先看**：不是 3 说明有一侧解算无效（杆长/偏置几何不成立），后面数值都不用信。
- **腿长**（ch15/ch19）：腿摆竖直时 ≈ 卷尺量的轴心到足端距离；理论区间 **0.136~0.46 m**（`lu=0.21, lg=0.25`）。
- **腿摆倾角**（ch16/ch20）：腿竖直时 ≈ 0；腿前摆/后摆应正负相反。
- **左右对称**：手动把两腿摆到相同姿态，ch15 与 ch19 应**同向同值**。反向 = 右腿前后电机的表项顺序或 `dm_sign` 反了。
- **电机观测量**（ch3~14）：值在动 = 该电机通信正常；手推腿时位置连续变化、无跳变。
- **位置**：ch3~6 是"原始解码角"，ch7~10 是"零点后"的电机角（进解算和 PID 实际消费的值）；两者成对，差一个零点常数。
- **速度**（ch11~14）：静置应接近 0（小幅抖动正常）。

> 轮在线看 ch0 的 bit6/7；轮速/轮角当前不在 VOFA 通道，需要时用调试器看 `motor_state.dji`。

---

## 极性与坐标约定

右侧极性在驱动解码后统一到机体坐标系（`dm_sign[i].fb` / `dji_sign[i].fb`）；下发时按 `dm_sign[i].out` / `dji_sign[i].out` 在**驱动边界取反一次**，后续模块禁止重复取反。两份极性都在 `machine_config.c` 的机器表里，换机器只改表。
