#ifndef __DR16_H
#define __DR16_H

#include "main.h"
#include "rc_types.h"
#include <stdbool.h>

#define DR16_FRAME_LEN   18
#define DR16_OFFLINE_MS  50u




extern dr16_t dr16;

void        DR16_Init(void);
void        DR16_Process(void);
bool        DR16_Online(void);
int16_t     DR16_Deadline(int16_t input, uint16_t deadline);
dr16_t      DR16_Snapshot(void);

#define DR16_SW_LEFT_UP     (dr16.s1 == DR16_SW_UP)
#define DR16_SW_LEFT_MID    (dr16.s1 == DR16_SW_MID)
#define DR16_SW_LEFT_DOWN   (dr16.s1 == DR16_SW_DOWN)
#define DR16_SW_RIGHT_UP    (dr16.s2 == DR16_SW_UP)
#define DR16_SW_RIGHT_MID   (dr16.s2 == DR16_SW_MID)
#define DR16_SW_RIGHT_DOWN  (dr16.s2 == DR16_SW_DOWN)

#endif
