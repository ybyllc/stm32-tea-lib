#include "icm42670.h"
#include "soft_iic.h"
#include <stdio.h>

// I2C 实例
static soft_iic_t icm_iic;

// 全局变量
int16_t icm42670_gyro_x, icm42670_gyro_y, icm42670_gyro_z;
int16_t icm42670_acc_x, icm42670_acc_y, icm42670_acc_z;
int16_t icm42670_temp;

float icm42670_gyro_x_dps, icm42670_gyro_y_dps, icm42670_gyro_z_dps;
float icm42670_acc_x_g, icm42670_acc_y_g, icm42670_acc_z_g;
float icm42670_temp_c;

// 兼容原来的陀螺仪接口变量
extern float fAcc[3], fGyro[3], fAngle[3], fYaw;

// ICM426xx 默认配置
#define ICM_GYRO_FS_SEL                  ICM42670_GYRO_FS_250DPS
#define ICM_ACCEL_FS_SEL                 ICM42670_ACCEL_FS_4G
#define ICM_GYRO_ODR                     ICM42670_ODR_1000
#define ICM_ACCEL_ODR                    ICM42670_ODR_1000

// LSM6 默认配置（104Hz，陀螺仪±2000dps，加速度计±4g）
#define LSM6_CFG_CTRL1_XL                0x48
#define LSM6_CFG_CTRL2_G                 0x4C
#define LSM6_CFG_CTRL3_C                 0x44

// LSM6 固定灵敏度（对应上述配置）
#define LSM6_GYRO_SENS_2000DPS           0.07f
#define LSM6_ACCEL_SENS_4G               0.000122f

static IMU_DeviceType s_imu_type = IMU_DEVICE_NONE;
static uint8_t s_imu_who_am_i = 0x00;
static uint8_t s_imu_addr = ICM42670_I2C_ADDR;

static const uint8_t k_imu_addr_list[] = {0x6B, 0x6A, 0x69, 0x68};

static int16_t ICM42670_Be16(uint8_t high, uint8_t low)
{
    return (int16_t)(((uint16_t)high << 8) | low);
}

static int16_t ICM42670_Le16(uint8_t low, uint8_t high)
{
    return (int16_t)(((uint16_t)high << 8) | low);
}

static uint8_t ICM42670_IsInvalidWhoAmI(uint8_t who_am_i)
{
    return (uint8_t)((who_am_i == 0x00) || (who_am_i == 0xFF));
}

static uint8_t ICM42670_IsIcmWhoAmI(uint8_t who_am_i)
{
    switch (who_am_i) {
        case IMU_WHO_AM_I_ICM42670:
        case IMU_WHO_AM_I_ICM42670_ALT:
        case IMU_WHO_AM_I_ICM42605:
        case IMU_WHO_AM_I_ICM42688:
            return 1;
        default:
            return 0;
    }
}

static uint8_t ICM42670_IsLsmWhoAmI(uint8_t who_am_i)
{
    switch (who_am_i) {
        case IMU_WHO_AM_I_LSM6DS3:
        case IMU_WHO_AM_I_LSM6DSL:
        case IMU_WHO_AM_I_LSM6DSR:
        case IMU_WHO_AM_I_LSM6DSO:
            return 1;
        default:
            return 0;
    }
}

IMU_DeviceType ICM42670_GetDeviceType(void)
{
    return s_imu_type;
}

const char* ICM42670_GetDeviceName(void)
{
    switch (s_imu_type) {
        case IMU_DEVICE_ICM426XX:
            return "ICM426xx";
        case IMU_DEVICE_LSM6:
            return "LSM6";
        case IMU_DEVICE_LSM6_COMPAT:
            return "LSM6-Compat";
        default:
            return "Unknown";
    }
}

uint8_t ICM42670_GetWhoAmI(void)
{
    return s_imu_who_am_i;
}

uint8_t ICM42670_GetI2CAddr(void)
{
    return s_imu_addr;
}

/**
 * @brief  向 IMU 写入一个字节
 */
void ICM42670_Write_Reg(uint8_t reg, uint8_t data)
{
    soft_iic_write_reg(&icm_iic, reg, data);
}

/**
 * @brief  从 IMU 读取一个字节
 */
u8 ICM42670_Read_Reg(uint8_t reg)
{
    return soft_iic_read_reg(&icm_iic, reg);
}

/**
 * @brief  从 IMU 读取多个字节
 */
void ICM42670_Read_Regs(uint8_t reg, uint8_t *buf, uint8_t len)
{
    soft_iic_read_bytes(&icm_iic, reg, buf, len);
}

static uint8_t ICM42670_ReadWhoAmIStable(uint8_t *who_am_i_out)
{
    uint8_t a, b, c;

    a = ICM42670_Read_Reg(ICM42670_WHO_AM_I);
    delay_ms(1);
    b = ICM42670_Read_Reg(ICM42670_WHO_AM_I);
    delay_ms(1);
    c = ICM42670_Read_Reg(ICM42670_WHO_AM_I);

    if (!ICM42670_IsInvalidWhoAmI(a) && (a == b || a == c)) {
        *who_am_i_out = a;
        return 1;
    }
    if (!ICM42670_IsInvalidWhoAmI(b) && b == c) {
        *who_am_i_out = b;
        return 1;
    }
    if (!ICM42670_IsInvalidWhoAmI(a)) {
        *who_am_i_out = a;
        return 1;
    }
    if (!ICM42670_IsInvalidWhoAmI(b)) {
        *who_am_i_out = b;
        return 1;
    }
    if (!ICM42670_IsInvalidWhoAmI(c)) {
        *who_am_i_out = c;
        return 1;
    }

    return 0;
}

static IMU_DeviceType ICM42670_DecodeDeviceType(uint8_t who_am_i)
{
    if (ICM42670_IsIcmWhoAmI(who_am_i)) {
        return IMU_DEVICE_ICM426XX;
    }
    if (ICM42670_IsLsmWhoAmI(who_am_i)) {
        return IMU_DEVICE_LSM6;
    }
    return IMU_DEVICE_LSM6_COMPAT;
}

static uint8_t ICM42670_AutoDetect(void)
{
    uint8_t i;
    uint8_t who_am_i;

    for (i = 0; i < (uint8_t)(sizeof(k_imu_addr_list) / sizeof(k_imu_addr_list[0])); i++) {
        icm_iic.addr = k_imu_addr_list[i];
        if (ICM42670_ReadWhoAmIStable(&who_am_i)) {
            s_imu_addr = k_imu_addr_list[i];
            s_imu_who_am_i = who_am_i;
            s_imu_type = ICM42670_DecodeDeviceType(who_am_i);
            return 1;
        }
    }

    s_imu_type = IMU_DEVICE_NONE;
    s_imu_who_am_i = 0x00;
    return 0;
}

static void ICM42670_ConfigIcm(void)
{
    ICM42670_Write_Reg(ICM42670_PWR_MGMT_0, 0x80);
    delay_ms(1);

    // 加速度计和陀螺仪都开启 LN 模式
    ICM42670_Write_Reg(ICM42670_PWR_MGMT_0, 0x0F);
    delay_ms(1);

    ICM42670_Write_Reg(ICM42670_GYRO_CONFIG_0, (uint8_t)((ICM_GYRO_FS_SEL << 5) | (ICM_GYRO_ODR << 2)));
    delay_us(100);

    ICM42670_Write_Reg(ICM42670_ACCEL_CONFIG_0, (uint8_t)((ICM_ACCEL_FS_SEL << 5) | (ICM_ACCEL_ODR << 2)));
    delay_us(100);

    ICM42670_Write_Reg(ICM42670_LP_CONFIG, 0x00);
    delay_us(100);
}

static void ICM42670_ConfigLsm(void)
{
    ICM42670_Write_Reg(LSM6_CTRL3_C, LSM6_CFG_CTRL3_C);
    delay_us(100);

    ICM42670_Write_Reg(LSM6_CTRL1_XL, LSM6_CFG_CTRL1_XL);
    delay_us(100);

    ICM42670_Write_Reg(LSM6_CTRL2_G, LSM6_CFG_CTRL2_G);
    delay_us(100);
}

/**
 * @brief  GPIO电平测试
 */
void GPIO_Test(void)
{
    printf("GPIO test: SCL pin state=%d, SDA pin state=%d\r\n",
           HAL_GPIO_ReadPin(ICM42670_SCL_PORT, ICM42670_SCL_PIN),
           HAL_GPIO_ReadPin(ICM42670_SDA_PORT, ICM42670_SDA_PIN));
}

/**
 * @brief  I2C设备扫描测试
 */
void I2C_Scan(void)
{
    uint8_t i;
    uint8_t ack;

    printf("Starting I2C scan...\r\n");
    for (i = 0; i < 128; i++) {
        soft_iic_start(&icm_iic);
        soft_iic_send_byte(&icm_iic, (uint8_t)((i << 1) & 0xFE));
        ack = soft_iic_wait_ack(&icm_iic);
        soft_iic_stop(&icm_iic);
        if (ack == 0) {
            printf("I2C device found at address 0x%02X (7-bit: 0x%02X)\r\n", (i << 1), i);
        }
        delay_ms(1);
    }
    printf("I2C scan completed.\r\n");
}

/**
 * @brief  检查设备是否存在（当前地址）
 * @retval 1: 成功 0: 失败
 */
u8 ICM42670_Check(void)
{
    uint8_t who_am_i;

    if (!ICM42670_ReadWhoAmIStable(&who_am_i)) {
        return 0;
    }

    s_imu_who_am_i = who_am_i;
    s_imu_type = ICM42670_DecodeDeviceType(who_am_i);
    return 1;
}

/**
 * @brief  初始化 IMU（自动适配 ICM426xx / LSM6 系列）
 * @retval u8 初始化结果：1成功，0失败
 */
u8 ICM42670_Init(void)
{
    GPIO_InitTypeDef GPIO_Initure;

    // 初始化软件 I2C（地址会在自动扫描后覆盖）
    soft_iic_init(&icm_iic, ICM42670_SCL_PORT, ICM42670_SCL_PIN,
                  ICM42670_SDA_PORT, ICM42670_SDA_PIN, ICM42670_I2C_ADDR, 5);

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    // SA0 引脚（优先拉高；若硬件外接不同地址，后续自动扫描可识别）
    GPIO_Initure.Pin = ICM42670_SA0_PIN;
    GPIO_Initure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_Initure.Pull = GPIO_PULLUP;
    GPIO_Initure.Speed = GPIO_SPEED_FAST;
    HAL_GPIO_Init(ICM42670_SA0_PORT, &GPIO_Initure);
    HAL_GPIO_WritePin(ICM42670_SA0_PORT, ICM42670_SA0_PIN, GPIO_PIN_SET);

    // CS 引脚（拉高，禁用 SPI 模式）
    GPIO_Initure.Pin = ICM42670_CS_PIN;
    HAL_GPIO_Init(ICM42670_CS_PORT, &GPIO_Initure);
    HAL_GPIO_WritePin(ICM42670_CS_PORT, ICM42670_CS_PIN, GPIO_PIN_SET);

    delay_ms(30);

    if (!ICM42670_AutoDetect()) {
        printf("IMU detect failed: no valid WHO_AM_I on 0x68/0x69/0x6A/0x6B\r\n");
        return 0;
    }

    icm_iic.addr = s_imu_addr;

    printf("IMU detected: addr=0x%02X, WHO_AM_I=0x%02X, type=%s\r\n",
           s_imu_addr, s_imu_who_am_i, ICM42670_GetDeviceName());

    if (s_imu_type == IMU_DEVICE_ICM426XX) {
        ICM42670_ConfigIcm();
    } else {
        ICM42670_ConfigLsm();
    }

    return 1;
}

/**
 * @brief  读取陀螺仪数据
 */
void ICM42670_Get_Gyro(void)
{
    uint8_t buf[6];

    if (s_imu_type == IMU_DEVICE_ICM426XX) {
        ICM42670_Read_Regs(ICM42670_GYRO_DATA_X1, buf, 6);
        icm42670_gyro_x = ICM42670_Be16(buf[0], buf[1]);
        icm42670_gyro_y = ICM42670_Be16(buf[2], buf[3]);
        icm42670_gyro_z = ICM42670_Be16(buf[4], buf[5]);
    } else {
        ICM42670_Read_Regs(LSM6_OUTX_L_G, buf, 6);
        icm42670_gyro_x = ICM42670_Le16(buf[0], buf[1]);
        icm42670_gyro_y = ICM42670_Le16(buf[2], buf[3]);
        icm42670_gyro_z = ICM42670_Le16(buf[4], buf[5]);
    }
}

/**
 * @brief  读取加速度计数据
 */
void ICM42670_Get_Accel(void)
{
    uint8_t buf[6];

    if (s_imu_type == IMU_DEVICE_ICM426XX) {
        ICM42670_Read_Regs(ICM42670_ACCEL_DATA_X1, buf, 6);
        icm42670_acc_x = ICM42670_Be16(buf[0], buf[1]);
        icm42670_acc_y = ICM42670_Be16(buf[2], buf[3]);
        icm42670_acc_z = ICM42670_Be16(buf[4], buf[5]);
    } else {
        ICM42670_Read_Regs(LSM6_OUTX_L_A, buf, 6);
        icm42670_acc_x = ICM42670_Le16(buf[0], buf[1]);
        icm42670_acc_y = ICM42670_Le16(buf[2], buf[3]);
        icm42670_acc_z = ICM42670_Le16(buf[4], buf[5]);
    }
}

/**
 * @brief  读取温度数据
 */
void ICM42670_Get_Temp(void)
{
    uint8_t buf[2];

    if (s_imu_type == IMU_DEVICE_ICM426XX) {
        ICM42670_Read_Regs(ICM42670_TEMP_DATA1, buf, 2);
        icm42670_temp = ICM42670_Be16(buf[0], buf[1]);
    } else {
        ICM42670_Read_Regs(LSM6_TEMP_OUT_L, buf, 2);
        icm42670_temp = ICM42670_Le16(buf[0], buf[1]);
    }
}

/**
 * @brief  读取所有数据（陀螺仪、加速度计、温度）
 */
void ICM42670_Get_All(void)
{
    uint8_t buf[14];

    if (s_imu_type == IMU_DEVICE_ICM426XX) {
        ICM42670_Read_Regs(ICM42670_GYRO_DATA_X1, buf, 14);
        icm42670_gyro_x = ICM42670_Be16(buf[0], buf[1]);
        icm42670_gyro_y = ICM42670_Be16(buf[2], buf[3]);
        icm42670_gyro_z = ICM42670_Be16(buf[4], buf[5]);
        icm42670_temp = ICM42670_Be16(buf[6], buf[7]);
        icm42670_acc_x = ICM42670_Be16(buf[8], buf[9]);
        icm42670_acc_y = ICM42670_Be16(buf[10], buf[11]);
        icm42670_acc_z = ICM42670_Be16(buf[12], buf[13]);
    } else {
        ICM42670_Read_Regs(LSM6_TEMP_OUT_L, buf, 14);
        icm42670_temp = ICM42670_Le16(buf[0], buf[1]);
        icm42670_gyro_x = ICM42670_Le16(buf[2], buf[3]);
        icm42670_gyro_y = ICM42670_Le16(buf[4], buf[5]);
        icm42670_gyro_z = ICM42670_Le16(buf[6], buf[7]);
        icm42670_acc_x = ICM42670_Le16(buf[8], buf[9]);
        icm42670_acc_y = ICM42670_Le16(buf[10], buf[11]);
        icm42670_acc_z = ICM42670_Le16(buf[12], buf[13]);
    }
}

/**
 * @brief  将原始数据转换为物理单位
 */
void ICM42670_Data_Unit_Convert(void)
{
    if (s_imu_type == IMU_DEVICE_ICM426XX) {
        switch (ICM_GYRO_FS_SEL) {
            case ICM42670_GYRO_FS_2000DPS:
                icm42670_gyro_x_dps = (float)icm42670_gyro_x / 16.4f;
                icm42670_gyro_y_dps = (float)icm42670_gyro_y / 16.4f;
                icm42670_gyro_z_dps = (float)icm42670_gyro_z / 16.4f;
                break;
            case ICM42670_GYRO_FS_1000DPS:
                icm42670_gyro_x_dps = (float)icm42670_gyro_x / 32.8f;
                icm42670_gyro_y_dps = (float)icm42670_gyro_y / 32.8f;
                icm42670_gyro_z_dps = (float)icm42670_gyro_z / 32.8f;
                break;
            case ICM42670_GYRO_FS_500DPS:
                icm42670_gyro_x_dps = (float)icm42670_gyro_x / 65.5f;
                icm42670_gyro_y_dps = (float)icm42670_gyro_y / 65.5f;
                icm42670_gyro_z_dps = (float)icm42670_gyro_z / 65.5f;
                break;
            case ICM42670_GYRO_FS_250DPS:
                icm42670_gyro_x_dps = (float)icm42670_gyro_x / 131.0f;
                icm42670_gyro_y_dps = (float)icm42670_gyro_y / 131.0f;
                icm42670_gyro_z_dps = (float)icm42670_gyro_z / 131.0f;
                break;
            case ICM42670_GYRO_FS_125DPS:
                icm42670_gyro_x_dps = (float)icm42670_gyro_x / 262.0f;
                icm42670_gyro_y_dps = (float)icm42670_gyro_y / 262.0f;
                icm42670_gyro_z_dps = (float)icm42670_gyro_z / 262.0f;
                break;
            default:
                icm42670_gyro_x_dps = 0.0f;
                icm42670_gyro_y_dps = 0.0f;
                icm42670_gyro_z_dps = 0.0f;
                break;
        }

        switch (ICM_ACCEL_FS_SEL) {
            case ICM42670_ACCEL_FS_16G:
                icm42670_acc_x_g = (float)icm42670_acc_x / 2048.0f;
                icm42670_acc_y_g = (float)icm42670_acc_y / 2048.0f;
                icm42670_acc_z_g = (float)icm42670_acc_z / 2048.0f;
                break;
            case ICM42670_ACCEL_FS_8G:
                icm42670_acc_x_g = (float)icm42670_acc_x / 4096.0f;
                icm42670_acc_y_g = (float)icm42670_acc_y / 4096.0f;
                icm42670_acc_z_g = (float)icm42670_acc_z / 4096.0f;
                break;
            case ICM42670_ACCEL_FS_4G:
                icm42670_acc_x_g = (float)icm42670_acc_x / 8192.0f;
                icm42670_acc_y_g = (float)icm42670_acc_y / 8192.0f;
                icm42670_acc_z_g = (float)icm42670_acc_z / 8192.0f;
                break;
            case ICM42670_ACCEL_FS_2G:
                icm42670_acc_x_g = (float)icm42670_acc_x / 16384.0f;
                icm42670_acc_y_g = (float)icm42670_acc_y / 16384.0f;
                icm42670_acc_z_g = (float)icm42670_acc_z / 16384.0f;
                break;
            default:
                icm42670_acc_x_g = 0.0f;
                icm42670_acc_y_g = 0.0f;
                icm42670_acc_z_g = 0.0f;
                break;
        }

        icm42670_temp_c = (float)icm42670_temp / 132.48f + 25.0f;
    } else {
        icm42670_gyro_x_dps = (float)icm42670_gyro_x * LSM6_GYRO_SENS_2000DPS;
        icm42670_gyro_y_dps = (float)icm42670_gyro_y * LSM6_GYRO_SENS_2000DPS;
        icm42670_gyro_z_dps = (float)icm42670_gyro_z * LSM6_GYRO_SENS_2000DPS;

        icm42670_acc_x_g = (float)icm42670_acc_x * LSM6_ACCEL_SENS_4G;
        icm42670_acc_y_g = (float)icm42670_acc_y * LSM6_ACCEL_SENS_4G;
        icm42670_acc_z_g = (float)icm42670_acc_z * LSM6_ACCEL_SENS_4G;

        if (s_imu_who_am_i == IMU_WHO_AM_I_LSM6DS3) {
            icm42670_temp_c = (float)icm42670_temp / 16.0f + 25.0f;
        } else {
            icm42670_temp_c = (float)icm42670_temp / 256.0f + 25.0f;
        }
    }
}

/**
 * @brief  兼容原来的陀螺仪接口，更新陀螺仪数据
 */
void ICM42670_Gryo_Update(void)
{
    static float angle_x = 0.0f;
    static float angle_y = 0.0f;
    static float angle_z = 0.0f;
    static uint32_t last_time = 0;
    uint32_t current_time;
    float dt;

    ICM42670_Get_All();
    ICM42670_Data_Unit_Convert();

    fAcc[0] = icm42670_acc_x_g;
    fAcc[1] = icm42670_acc_y_g;
    fAcc[2] = icm42670_acc_z_g;

    fGyro[0] = icm42670_gyro_x_dps;
    fGyro[1] = icm42670_gyro_y_dps;
    fGyro[2] = icm42670_gyro_z_dps;

    current_time = HAL_GetTick();
    if (last_time == 0) {
        last_time = current_time;
    }
    dt = (current_time - last_time) / 1000.0f;
    last_time = current_time;

    angle_x += fGyro[0] * dt;
    angle_y += fGyro[1] * dt;
    angle_z += fGyro[2] * dt;

    if (angle_x > 180.0f) angle_x -= 360.0f;
    else if (angle_x < -180.0f) angle_x += 360.0f;

    if (angle_y > 180.0f) angle_y -= 360.0f;
    else if (angle_y < -180.0f) angle_y += 360.0f;

    if (angle_z > 180.0f) angle_z -= 360.0f;
    else if (angle_z < -180.0f) angle_z += 360.0f;

    fAngle[0] = angle_x;
    fAngle[1] = angle_y;
    fAngle[2] = angle_z;
    fYaw = angle_z;
}
