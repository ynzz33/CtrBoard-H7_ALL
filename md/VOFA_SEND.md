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

## 当前电机反馈测试通道

| 通道 | 内容 |
|------|------|
| 0~3 | 左前髋、左后髋、右前髋、右后髋角度（rad，右侧已取反） |
| 4~7 | 左前髋、左后髋、右前髋、右后髋角速度（rad/s，右侧已取反） |
| 8~9 | DJI 左/右轮总编码器值 `angle_total`（机体坐标系 count） |
| 10~11 | DJI 左/右轮总编码角度 `angle_total_rad`（rad） |
| 12~13 | DJI 左/右轮速度 `vel_raw`（机体坐标系 rpm） |
| 14~15 | DJI 左/右轮角速度 `vel_rad_s`（rad/s） |
| 16~17 | DJI 左/右轮单圈编码器值 `angle_raw`（机体坐标系 count） |
| 18~19 | DJI 左/右轮电流反馈 `current_raw`（机体坐标系） |
| 20 | 500Hz 执行周期计数 |
| 21 | 原始 `ch0` |
| 24 | 原始 `ch3` |
| 25 | 取反后的拨轮 `wheel` |
| 26 | 左拨杆 `s1` |
| 27 | 右拨杆 `s2` |
| 28 | DR16 在线状态 |
| 29 | 整车使能请求 |
| 30 | 故障位 `ctrl_fault` |
| 31 | 电机/安全位图：bit0~3=DM 在线，bit4~5=DJI 在线，bit6=robot_enabled，bit7=torque_output_enabled |

右侧极性在驱动解码后统一到机体坐标系；右侧 DM 力矩下发在驱动边界同步取反一次，后续模块禁止重复取反。
