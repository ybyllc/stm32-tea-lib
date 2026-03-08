#ifndef _GYRO_ICM_H
#define _GYRO_ICM_H

#include "stdint.h"
#include "common.h"
#include "soft_iic.h"


// 默认引脚配置（可根据硬件修改）
#define ICM_DEFAULT_SCL_PORT        GPIOA
#define ICM_DEFAULT_SCL_PIN         GPIO_PIN_8
#define ICM_DEFAULT_SDA_PORT        GPIOC
#define ICM_DEFAULT_SDA_PIN         GPIO_PIN_9
#define ICM_DEFAULT_SA0_PORT        GPIOC
#define ICM_DEFAULT_SA0_PIN         GPIO_PIN_8
#define ICM_DEFAULT_CS_PORT         GPIOC
#define ICM_DEFAULT_CS_PIN          GPIO_PIN_6

// ICM系列型号定义
typedef enum {
    ICM_TYPE_NONE = 0,
    ICM_TYPE_42670,     // WHO_AM_I = 0x41
    ICM_TYPE_42688,     // WHO_AM_I = 0x47
    ICM_TYPE_42605,     // WHO_AM_I = 0x47 (与42688相同)
} ICM_Type;

// ICM系列引脚配置结构体
typedef struct {
    GPIO_TypeDef *scl_port;
    uint16_t scl_pin;
    GPIO_TypeDef *sda_port;
    uint16_t sda_pin;
    GPIO_TypeDef *sa0_port;
    uint16_t sa0_pin;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;
} ICM_PinConfig;

// ICM系列配置结构体
typedef struct {
    ICM_TYPE type;
    uint8_t i2c_addr;
    uint8_t gyro_fs;    // 陀螺仪量程
    uint8_t accel_fs;   // 加速度计量程
    uint8_t odr;        // 采样率
    ICM_PinConfig pins;
    soft_iic_t iic;
} ICM_Device;

// 寄存器地址（ICM系列通用）
#define ICM_REG_WHO_AM_I            0x0F
#define ICM_REG_DEVICE_CONFIG       0x11
#define ICM_REG_GYRO_CONFIG_0       0x14
#define ICM_REG_ACCEL_CONFIG_0      0x15
#define ICM_REG_LP_CONFIG           0x16
#define ICM_REG_TEMP_DATA1          0x1D
#define ICM_REG_TEMP_DATA0          0x1E
#define ICM_REG_GYRO_DATA_X1        0x21
#define ICM_REG_GYRO_DATA_X0        0x22
#define ICM_REG_GYRO_DATA_Y1        0x23
#define ICM_REG_GYRO_DATA_Y0        0x24
#define ICM_REG_GYRO_DATA_Z1        0x25
#define ICM_REG_GYRO_DATA_Z0        0x26
#define ICM_REG_ACCEL_DATA_X1       0x29
#define ICM_REG_ACCEL_DATA_X0       0x2A
#define ICM_REG_ACCEL_DATA_Y1       0x2B
#define ICM_REG_ACCEL_DATA_Y0       0x2C
#define ICM_REG_ACCEL_DATA_Z1       0x2D
#define ICM_REG_ACCEL_DATA_Z0       0x2E
#define ICM_REG_PWR_MGMT_0          0x4E

// WHO_AM_I 返回值（ICM-42670 与 ICM-42688/42605 相同）
#define ICM_WHO_AM_I_42670          0x41
#define ICM_WHO_AM_I_42688_42605    0x47    // ICM-42688 与 ICM-42605 相同  

// 陀螺仪满量程选项
#define ICM_GYRO_FS_2000DPS         0x00
#define ICM_GYRO_FS_1000DPS         0x01
#define ICM_GYRO_FS_500DPS          0x02
#define ICM_GYRO_FS_250DPS          0x03
#define ICM_GYRO_FS_125DPS          0x04

// 加速度计满量程选项
#define ICM_ACCEL_FS_16G            0x00
#define ICM_ACCEL_FS_8G             0x01
#define ICM_ACCEL_FS_4G             0x02
#define ICM_ACCEL_FS_2G             0x03

// 采样率选项 (Hz)
#define ICM_ODR_1000                0x00
#define ICM_ODR_500                 0x01
#define ICM_ODR_250                 0x02
#define ICM_ODR_125                 0x03
#define ICM_ODR_62_5                0x04
#define ICM_ODR_31_25               0x05
#define ICM_ODR_15_625              0x06
#define ICM_ODR_7_8125              0x07

// 全局变量
extern int16_t icm_gyro_x, icm_gyro_y, icm_gyro_z;
extern int16_t icm_acc_x, icm_acc_y, icm_acc_z;
extern int16_t icm_temp;

extern float icm_gyro_x_dps, icm_gyro_y_dps, icm_gyro_z_dps;
extern float icm_acc_x_g, icm_acc_y_g, icm_acc_z_g;
extern float icm_temp_c;

extern ICM_TYPE icm_current_type;

// 函数声明

/**
 * @brief 初始化ICM系列陀螺仪（自动检测型号）
 * @retval 1-成功 0-失败
 */
u8 ICM_Init(void);

/**
 * @brief 使用自定义引脚配置初始化
 * @param pins 引脚配置
 * @retval 1-成功 0-失败
 */
u8 ICM_InitWithPins(ICM_PinConfig *pins);

/**
 * @brief 检测ICM型号
 * @retval ICM_TYPE
 */
ICM_TYPE ICM_DetectType(void);

/**
 * @brief 获取当前ICM型号
 * @retval ICM_TYPE
 */
ICM_TYPE ICM_GetType(void);

/**
 * @brief 获取型号名称字符串
 * @retval 型号名称
 */
const char* ICM_GetTypeName(void);

/**
 * @brief 读取陀螺仪数据
 */
void ICM_Get_Gyro(void);

/**
 * @brief 读取加速度计数据（原始计数）
 */
void ICM_Get_Accel(void);

/**
 * @brief 读取温度数据
 */
void ICM_Get_Temp(void);

/**
 * @brief 读取所有数据（包括陀螺仪、加速度计、温度）
 */
void ICM_Get_All(void);

/**
 * @brief 数据单位转换（原始计数转换为物理量）
 */
void ICM_Data_Unit_Convert(void);

/**
 * @brief 兼容接口：更新陀螺仪数据
 */
void ICM_Gryo_Update(void);

/**
 * @brief I2C设备扫描
 */
void ICM_I2C_Scan(void);

// 兼容旧接口（保留）
extern int16_t iAcc[3], iGyro[3];
extern float fAcc[3], fGyro[3], fAngle[3], fYaw;

#endif
