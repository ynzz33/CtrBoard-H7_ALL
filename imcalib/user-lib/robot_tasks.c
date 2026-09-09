#include "robot_tasks.h"
#include "dr16.h"
#include "hi229.h"
#include "dm.h"
#include "dji.h"
#include "can_bus.h"
#include "Vofa_send.h"
#include "tim.h"
#include "Attitude_Algorithm.h"

/* 共享状态 */
imu_state_t       imu_state;
motor_state_t     motor_state;
leg_state_t       leg_l;
leg_state_t       leg_r;
action_state_t    action_state;
command_state_t   command_state;
volatile uint32_t ctrl_fault;

/* 500Hz 节拍信号量 */
osSemaphoreDef(ctrl_tick_sem);
osSemaphoreId ctrl_tick_sem_handle = NULL;

static volatile uint32_t tick_count;

/* 更新电机状态 */
static void Motor_State_Update(void)
{
    for (uint8_t i = 0; i < DM_MOTOR_NUM; i++)
    {
        const dm_motor_feedback_t *feedback = &dm_motor_feedback[i];
        motor_state.dm.pos_rad[i] = feedback->pos_rad;
        motor_state.dm.vel_rad_s[i] = feedback->vel_rad_s;
        motor_state.dm.trq_nm[i] = feedback->trq_nm;
        motor_state.dm.last_rx_tick[i] = feedback->last_rx_tick;
        motor_state.dm.online[i] = (uint8_t)Dm_Is_Online(i);
    }

    for (uint8_t i = 0; i < DJI_MOTOR_NUM; i++)
    {
        const dji_motor_feedback_t *feedback = &dji_motor_feedback[i];
        motor_state.dji.angle_rad[i] = feedback->angle_rad;
        motor_state.dji.angle_total_rad[i] = feedback->angle_total_rad;
        motor_state.dji.vel_rad_s[i] = feedback->vel_rad_s;
        motor_state.dji.current_raw[i] = feedback->current_raw;
        motor_state.dji.last_rx_tick[i] = feedback->last_rx_tick;
        motor_state.dji.online[i] = (uint8_t)Dji_Is_Online(i);
    }

    motor_state.timestamp_ms = HAL_GetTick();
    motor_state.updated = 1u;
}

/* 初始化：建信号量 */
void Robot_Control_Init(void)
{
    ctrl_tick_sem_handle = osSemaphoreCreate(osSemaphore(ctrl_tick_sem), 1);
    Leg_Init(&leg_l);
    Leg_Init(&leg_r);
}

/* IMU 任务体：HI229 通信 + 姿态 */
void imu_task_body(void)
{
    Attitude_Init(&imu_state);

    for (;;)
    {
        HI229_Process();
        if (HI229_Online())
        {
            hi229_data_t s = HI229_Snapshot();

            if (!imu_state.timestamp_valid)
            {
                Attitude_Init(&imu_state);
                imu_state.timestamp_valid = 1u;
                imu_state.last_timestamp_ms = s.ts;
            }

            imu_state.input.gyro_dps[0] = s.gyr[0];
            imu_state.input.gyro_dps[1] = s.gyr[1];
            imu_state.input.gyro_dps[2] = s.gyr[2];
            imu_state.input.accel_g[0] = s.acc[0];
            imu_state.input.accel_g[1] = s.acc[1];
            imu_state.input.accel_g[2] = s.acc[2];
            imu_state.input.timestamp_ms = s.ts;
            imu_state.reference.quat[0] = s.quat[0];
            imu_state.reference.quat[1] = s.quat[1];
            imu_state.reference.quat[2] = s.quat[2];
            imu_state.reference.quat[3] = s.quat[3];
            imu_state.reference.euler_deg[0] = s.eul[0];
            imu_state.reference.euler_deg[1] = s.eul[1];
            imu_state.reference.euler_deg[2] = s.eul[2];
            IMU_State_Convert_Unit(&imu_state);

            if (s.ts != imu_state.last_timestamp_ms)
            {
                imu_state.input.dt_s = (float)(s.ts - imu_state.last_timestamp_ms) * 0.001f;
                (void)Attitude_Update(&imu_state);
            }
            imu_state.last_timestamp_ms = s.ts;
            imu_state.online = 1u;
        }
        else
        {
            imu_state.online = 0u;
            imu_state.timestamp_valid = 0u;
        }
        osDelay(5);
    }
}

/* 控制任务体：观测量构建 + 推理 */
void ctrl_task_body(void)
{
    for (;;)
    {
        /* TODO: 读 imu_state/motor_state/command_state
         *   -> 电机偏置 -> 五连杆运动学 -> obs(25)+history(125)
         *   -> CubeAI 推理 -> action_state */
        osDelay(10);
    }
}

/* 输出任务体：电机解析 + 力矩 + 安全门 + CAN */
void output_task_body(void)
{
    HAL_TIM_Base_Start_IT(&htim6);   /* 启动 500Hz 节拍 */

    for (;;)
    {
        osSemaphoreWait(ctrl_tick_sem_handle, osWaitForever);
        tick_count++;

        Dm_Parse();    /* 电机反馈解码 */
        Dji_Parse();
        Motor_State_Update();

        if (ctrl_fault != FAULT_NONE)
        {
            Dji_All_Stop();
            for (uint8_t i = 0; i < DM_MOTOR_NUM; i++)
                Dm_Send_Command(i, DM_CMD_DISABLE);
            continue;
        }

        /* TODO: 力矩计算(PD+雅可比+气弹簧) + 限幅 -> 下发 */
        Dji_All_Stop();   /* 骨架：轮零电流，保证安全 */
    }
}

/* 监控任务体：DR16 + 在线检测 + VOFA */
void monitor_task_body(void)
{
    static uint8_t vofa_div = 0;

    for (;;)
    {
        /* 遥控解析 */
        DR16_Process();
        if (DR16_Online())
        {
            command_state.vx       = (float)dr16.ch2;   /* TODO 映射+缩放 */
            command_state.yaw_rate = (float)dr16.ch0;
            command_state.height   = (float)dr16.ch1;
            command_state.mode     = dr16.s1;
        }
        else
        {
            command_state.vx       = 0.0f;
            command_state.yaw_rate = 0.0f;
            command_state.height   = 0.0f;
            command_state.mode     = 0u;
        }

        /* 在线检测 + CAN 看门狗 */
        bool can_ok = Can_Bus_Online(true);
        uint32_t fault = FAULT_NONE;
        if (!imu_state.online) fault |= FAULT_IMU;
        if (!DR16_Online())    fault |= FAULT_RC;
        if (!can_ok)           fault |= FAULT_CAN;
        /* TODO: 电机在线 Dm_Is_Online / Dji_Is_Online */
        ctrl_fault = fault;

        /* VOFA 200Hz，HI229 基础链路验证：0~11 见下方通道表。 */
        if (++vofa_div >= 5)
        {
            vofa_div = 0;
            static float dbg[12];

            dbg[0]  = (float)tick_count;
            dbg[1]  = hi229_data.eul[HI229_ROLL];
            dbg[2]  = hi229_data.eul[HI229_PITCH];
            dbg[3]  = hi229_data.eul[HI229_YAW];
            dbg[4]  = (float)imu_state.online;
            dbg[5]  = (float)hi229_data.ts;
            dbg[6]  = hi229_data.acc[HI229_ACC_X];
            dbg[7]  = hi229_data.acc[HI229_ACC_Y];
            dbg[8]  = hi229_data.acc[HI229_ACC_Z];
            dbg[9]  = hi229_data.gyr[HI229_ROLL];
            dbg[10] = hi229_data.gyr[HI229_PITCH];
            dbg[11] = hi229_data.gyr[HI229_YAW];

            Vofa_Send(dbg, 12);
        }

        osDelay(1);
    }
}
