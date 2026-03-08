#include "gyro_icm.h"
#include <stdio.h>  // 添加 printf 支持

// 全局设备实例
static ICM_Device icm_dev;

// 全局变量
int16_t icm_gyro_x, icm_gyro_y, icm_gyro_z;
int16_t icm_acc_x, icm_acc_y, icm_acc_z;
int16_t icm_temp;

float icm_gyro_x_dps, icm_gyro_y_dps, icm_gyro_z_dps;
float icm_acc_x_g, icm_acc_y_g, icm_acc_z_g;
float icm_temp_c;

ICM_TYPE icm_current_type = ICM_TYPE_NONE;

// 兼容变量
extern float fAcc[3], fGyro[3], fAngle[3], fYaw;

// 默认配置
#define DEFAULT_GYRO_FS     ICM_GYRO_FS_250DPS
#define DEFAULT_ACCEL_FS    ICM_ACCEL_FS_4G
#define DEFAULT_ODR         ICM_ODR_1000

// 写寄存器
static void ICM_Write_Reg(uint8_t reg, uint8_t data)
{
    soft_iic_write_reg(&icm_dev.iic, reg, data);
}

// 读寄存器
static uint8_t ICM_Read_Reg(uint8_t reg)
{
    return soft_iic_read_reg(&icm_dev.iic, reg);
}

// 读多字节
static void ICM_Read_Regs(uint8_t reg, uint8_t *buf, uint8_t len)
{
    soft_iic_read_bytes(&icm_dev.iic, reg, buf, len);
}

// 获取型号名称
const char* ICM_GetTypeName(void)
{
    switch (icm_current_type) {
        case ICM_TYPE_42670: return "ICM-42670";
        case ICM_TYPE_42688: return "ICM-42688";
        case ICM_TYPE_42605: return "ICM-42605";
        default: return "Unknown";
    }
}

// 获取当前型号
ICM_TYPE ICM_GetType(void)
{
    return icm_current_type;
}

// 检测ICM型号
ICM_TYPE ICM_DetectType(void)
{
    uint8_t who_am_i = ICM_Read_Reg(ICM_REG_WHO_AM_I);
    
    switch (who_am_i) {
        case ICM_WHO_AM_I_42670:
            return ICM_TYPE_42670;
        case ICM_WHO_AM_I_42688_42605:
            return ICM_TYPE_42688;  // 42688和42605返回相同ID，默认识别为42688
        default:
            return ICM_TYPE_NONE;
    }
}

// I2C扫描
void ICM_I2C_Scan(void)
{
    uint8_t i;
    uint8_t ack;
    
    printf("I2C Scan...\r\n");
    
    for (i = 0; i < 128; i++) {
        soft_iic_start(&icm_dev.iic);
        soft_iic_send_byte(&icm_dev.iic, (i << 1) & 0xFE);
        ack = soft_iic_wait_ack(&icm_dev.iic);
        soft_iic_stop(&icm_dev.iic);
        
        if (ack == 0) {
            printf("Found: 0x%02X\r\n", i);
        }
        delay_ms(1);
    }
}

// 初始化GPIO
static void ICM_InitGPIO(ICM_PinConfig *pins)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    // 使能时钟
    if (pins->scl_port == GPIOA || pins->sda_port == GPIOA || 
        pins->sa0_port == GPIOA || pins->cs_port == GPIOA)
        __HAL_RCC_GPIOA_CLK_ENABLE();
    if (pins->scl_port == GPIOB || pins->sda_port == GPIOB || 
        pins->sa0_port == GPIOB || pins->cs_port == GPIOB)
        __HAL_RCC_GPIOB_CLK_ENABLE();
    if (pins->scl_port == GPIOC || pins->sda_port == GPIOC || 
        pins->sa0_port == GPIOC || pins->cs_port == GPIOC)
        __HAL_RCC_GPIOC_CLK_ENABLE();
    
    // SA0引脚 - 设置I2C地址
    GPIO_InitStruct.Pin = pins->sa0_pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(pins->sa0_port, &GPIO_InitStruct);
    HAL_GPIO_WritePin(pins->sa0_port, pins->sa0_pin, GPIO_PIN_SET);  // 高电平 = 0x69
    
    // CS引脚 - 禁用SPI模式
    GPIO_InitStruct.Pin = pins->cs_pin;
    HAL_GPIO_Init(pins->cs_port, &GPIO_InitStruct);
    HAL_GPIO_WritePin(pins->cs_port, pins->cs_pin, GPIO_PIN_SET);  // 高电平禁用SPI
}

// 使用自定义引脚初始化
u8 ICM_InitWithPins(ICM_PinConfig *pins)
{
    uint16_t timeout = 0;
    
    // 保存引脚配置
    icm_dev.pins = *pins;
    icm_dev.gyro_fs = DEFAULT_GYRO_FS;
    icm_dev.accel_fs = DEFAULT_ACCEL_FS;
    icm_dev.odr = DEFAULT_ODR;
    icm_dev.i2c_addr = 0x69;  // SA0高电平
    
    // 初始化GPIO
    ICM_InitGPIO(pins);
    
    // 初始化软件I2C
    soft_iic_init(&icm_dev.iic, pins->scl_port, pins->scl_pin,
                  pins->sda_port, pins->sda_pin, icm_dev.i2c_addr, 5);
    
    delay_ms(50);
    
    // 检测设备
    while ((icm_current_type = ICM_DetectType()) == ICM_TYPE_NONE) {
        timeout++;
        if (timeout > 10) {
            printf("ICM not found!\r\n");
            return 0;
        }
        delay_ms(10);
    }
    
    printf("Detected: %s (WHO_AM_I=0x%02X)\r\n", ICM_GetTypeName(), 
           ICM_Read_Reg(ICM_REG_WHO_AM_I));
    
    // 复位设备
    ICM_Write_Reg(ICM_REG_PWR_MGMT_0, 0x80);
    delay_ms(1);
    
    // 唤醒设备
    ICM_Write_Reg(ICM_REG_PWR_MGMT_0, 0x01);
    delay_us(100);
    
    // 配置陀螺仪
    ICM_Write_Reg(ICM_REG_GYRO_CONFIG_0, (icm_dev.gyro_fs << 5) | (icm_dev.odr << 2));
    delay_us(100);
    
    // 配置加速度计
    ICM_Write_Reg(ICM_REG_ACCEL_CONFIG_0, (icm_dev.accel_fs << 5) | (icm_dev.odr << 2));
    delay_us(100);
    
    // 禁用低功耗模式
    ICM_Write_Reg(ICM_REG_LP_CONFIG, 0x00);
    delay_us(100);
    
    printf("ICM Init OK!\r\n");
    return 1;
}

// 默认初始化
u8 ICM_Init(void)
{
    ICM_PinConfig default_pins = {
        .scl_port = ICM_DEFAULT_SCL_PORT,
        .scl_pin = ICM_DEFAULT_SCL_PIN,
        .sda_port = ICM_DEFAULT_SDA_PORT,
        .sda_pin = ICM_DEFAULT_SDA_PIN,
        .sa0_port = ICM_DEFAULT_SA0_PORT,
        .sa0_pin = ICM_DEFAULT_SA0_PIN,
        .cs_port = ICM_DEFAULT_CS_PORT,
        .cs_pin = ICM_DEFAULT_CS_PIN,
    };
    
    return ICM_InitWithPins(&default_pins);
}

// 读取陀螺仪数据
void ICM_Get_Gyro(void)
{
    uint8_t buf[6];
    ICM_Read_Regs(ICM_REG_GYRO_DATA_X1, buf, 6);
    
    icm_gyro_x = ((int16_t)buf[0] << 8) | buf[1];
    icm_gyro_y = ((int16_t)buf[2] << 8) | buf[3];
    icm_gyro_z = ((int16_t)buf[4] << 8) | buf[5];
}

// 读取加速度计数据
void ICM_Get_Accel(void)
{
    uint8_t buf[6];
    ICM_Read_Regs(ICM_REG_ACCEL_DATA_X1, buf, 6);
    
    icm_acc_x = ((int16_t)buf[0] << 8) | buf[1];
    icm_acc_y = ((int16_t)buf[2] << 8) | buf[3];
    icm_acc_z = ((int16_t)buf[4] << 8) | buf[5];
}

// 读取温度数据
void ICM_Get_Temp(void)
{
    uint8_t buf[2];
    ICM_Read_Regs(ICM_REG_TEMP_DATA1, buf, 2);
    icm_temp = ((int16_t)buf[0] << 8) | buf[1];
}

// 读取所有数据
void ICM_Get_All(void)
{
    uint8_t buf[14];
    ICM_Read_Regs(ICM_REG_GYRO_DATA_X1, buf, 14);
    
    icm_gyro_x = ((int16_t)buf[0] << 8) | buf[1];
    icm_gyro_y = ((int16_t)buf[2] << 8) | buf[3];
    icm_gyro_z = ((int16_t)buf[4] << 8) | buf[5];
    
    icm_temp = ((int16_t)buf[6] << 8) | buf[7];
    
    icm_acc_x = ((int16_t)buf[8] << 8) | buf[9];
    icm_acc_y = ((int16_t)buf[10] << 8) | buf[11];
    icm_acc_z = ((int16_t)buf[12] << 8) | buf[13];
}

// 数据单位转换
void ICM_Data_Unit_Convert(void)
{
    // 陀螺仪转换 (dps)
    switch (icm_dev.gyro_fs) {
        case ICM_GYRO_FS_2000DPS:
            icm_gyro_x_dps = (float)icm_gyro_x / 16.4f;
            icm_gyro_y_dps = (float)icm_gyro_y / 16.4f;
            icm_gyro_z_dps = (float)icm_gyro_z / 16.4f;
            break;
        case ICM_GYRO_FS_1000DPS:
            icm_gyro_x_dps = (float)icm_gyro_x / 32.8f;
            icm_gyro_y_dps = (float)icm_gyro_y / 32.8f;
            icm_gyro_z_dps = (float)icm_gyro_z / 32.8f;
            break;
        case ICM_GYRO_FS_500DPS:
            icm_gyro_x_dps = (float)icm_gyro_x / 65.5f;
            icm_gyro_y_dps = (float)icm_gyro_y / 65.5f;
            icm_gyro_z_dps = (float)icm_gyro_z / 65.5f;
            break;
        case ICM_GYRO_FS_250DPS:
            icm_gyro_x_dps = (float)icm_gyro_x / 131.0f;
            icm_gyro_y_dps = (float)icm_gyro_y / 131.0f;
            icm_gyro_z_dps = (float)icm_gyro_z / 131.0f;
            break;
        case ICM_GYRO_FS_125DPS:
            icm_gyro_x_dps = (float)icm_gyro_x / 262.0f;
            icm_gyro_y_dps = (float)icm_gyro_y / 262.0f;
            icm_gyro_z_dps = (float)icm_gyro_z / 262.0f;
            break;
        default:
            break;
    }
    
    // 加速度计转换 (g)
    switch (icm_dev.accel_fs) {
        case ICM_ACCEL_FS_16G:
            icm_acc_x_g = (float)icm_acc_x / 2048.0f;
            icm_acc_y_g = (float)icm_acc_y / 2048.0f;
            icm_acc_z_g = (float)icm_acc_z / 2048.0f;
            break;
        case ICM_ACCEL_FS_8G:
            icm_acc_x_g = (float)icm_acc_x / 4096.0f;
            icm_acc_y_g = (float)icm_acc_y / 4096.0f;
            icm_acc_z_g = (float)icm_acc_z / 4096.0f;
            break;
        case ICM_ACCEL_FS_4G:
            icm_acc_x_g = (float)icm_acc_x / 8192.0f;
            icm_acc_y_g = (float)icm_acc_y / 8192.0f;
            icm_acc_z_g = (float)icm_acc_z / 8192.0f;
            break;
        case ICM_ACCEL_FS_2G:
            icm_acc_x_g = (float)icm_acc_x / 16384.0f;
            icm_acc_y_g = (float)icm_acc_y / 16384.0f;
            icm_acc_z_g = (float)icm_acc_z / 16384.0f;
            break;
        default:
            break;
    }
    
    // 温度转换
    icm_temp_c = (float)icm_temp / 132.48f + 25.0f;
}

// 兼容接口：更新陀螺仪数据
void ICM_Gryo_Update(void)
{
    ICM_Get_All();
    ICM_Data_Unit_Convert();
    
    // 赋值给兼容变量
    fAcc[0] = icm_acc_x_g;
    fAcc[1] = icm_acc_y_g;
    fAcc[2] = icm_acc_z_g;
    
    fGyro[0] = icm_gyro_x_dps;
    fGyro[1] = icm_gyro_y_dps;
    fGyro[2] = icm_gyro_z_dps;
    
    // 简单角度积分
    static float angle_x = 0, angle_y = 0, angle_z = 0;
    static uint32_t last_time = 0;
    uint32_t current_time = HAL_GetTick();
    float dt = (current_time - last_time) / 1000.0f;
    last_time = current_time;
    
    angle_x += fGyro[0] * dt;
    angle_y += fGyro[1] * dt;
    angle_z += fGyro[2] * dt;
    
    // 角度范围限制
    if (angle_x > 180) angle_x -= 360;
    else if (angle_x < -180) angle_x += 360;
    if (angle_y > 180) angle_y -= 360;
    else if (angle_y < -180) angle_y += 360;
    if (angle_z > 180) angle_z -= 360;
    else if (angle_z < -180) angle_z += 360;
    
    fAngle[0] = angle_x;
    fAngle[1] = angle_y;
    fAngle[2] = angle_z;
    fYaw = angle_z;
}
