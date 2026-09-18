#ifndef __SYSID_CONFIG_H
#define __SYSID_CONFIG_H

/* 测试总开关(系统辨识用): 0=不编译测试代码 1=打开 */
/* 0: 策略仲裁回到 LQR/手动两路, 且不需要 imcalib/Sysid 参与构建 */
/* 1: 需要 imcalib/Sysid 在构建源目录里 (Keil 组 / eIDE srcDirs) */
#ifndef SYSID_ENABLE
#define SYSID_ENABLE    0
#endif

#endif
