#ifndef __SYSID_CONFIG_H
#define __SYSID_CONFIG_H

/* 测试总开关: 0=正常控制 1=系统辨识数据采集 */
/* 采完数据跑正常控制时改回 0 */
#ifndef SYSID_ENABLE
#define SYSID_ENABLE    0
#endif

/* 测试计划: 1=仅腿 2=仅轮 3=全部 */
/* 默认仅腿; 轮测试需架空且安全确认后才开 */
#ifndef SYSID_PLAN
#define SYSID_PLAN      2       /* 当前: 仅轮 (测髋时改回 1) */
#endif

#endif
