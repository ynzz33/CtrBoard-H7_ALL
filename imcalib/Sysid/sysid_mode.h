#ifndef __SYSID_MODE_H
#define __SYSID_MODE_H

#include "sysid_config.h"

#if SYSID_ENABLE

/* 进入 sysid 模式时调用一次 */
void Sysid_Mode_Init(void);

/* 测试模式单周期体 (500Hz actuationTask) */
void Sysid_Mode_Run(void);

#endif /* SYSID_ENABLE */
#endif /* __SYSID_MODE_H */
