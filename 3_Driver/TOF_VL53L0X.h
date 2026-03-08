#ifndef __VL53L0X_H
#define __VL53L0X_H

//#include "hsc.h"          // 你的 MCU 头文件，含 u8 等定义
#include "soft_iic.h"       // 你的软件 IIC 头文件

#define VL53L0X_ADDR      0x29   // 7bit 设备地址，读地址 = 0x53

/* 最常用寄存器 */
#define REG_IDENTIFICATION_MODEL_ID    0xC0
#define REG_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV 0x89
#define REG_MSRC_CONFIG_CONTROL        0x60
#define REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT 0x44
#define REG_SYSTEM_SEQUENCE_CONFIG     0x01
#define REG_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD 0x4E
#define REG_GLOBAL_CONFIG_REF_EN_START_SELECT 0xB6
#define REG_DYNAMIC_SPAD_REF_EN_START_OFFSET 0x4F
#define REG_POWER_MANAGEMENT_GO1_POWER_FORCE 0x80
#define REG_SYSRANGE_START               0x00
#define REG_RESULT_RANGE_STATUS          0x14
#define REG_RESULT_INTERRUPT_STATUS      0x13
#define REG_ALGO_PART_TO_PART_RANGE_OFFSET_MM 0x28
#define REG_I2C_SLAVE_DEVICE_ADDRESS     0x8A

u8  VL53L0X_Init(void);
u16 VL53L0X_ReadDistance(void);

#endif