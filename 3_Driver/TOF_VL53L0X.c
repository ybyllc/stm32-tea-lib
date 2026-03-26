#include "TOF_VL53L0X.h"
#include "soft_iic.h"
#include <stdio.h>

// TOF 软 I2C 实例
static soft_iic_t tof_iic;

// INT 引脚读取，保留 PC8 作为可选状态输入
#define VL53L0X_INT()     HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8)

// 调试日志开关，任务模式下默认关闭高频日志
#define TOF_DEBUG_LOG     0
#define TOF_BOOT_WAIT_MS  20U
#define TOF_RANGING_KICK_DELAY_MS  1U
#define TOF_RANGING_PREP_DELAY_MS  1U
#define TOF_RANGING_POLL_DELAY_MS  2U
#define TOF_RANGING_POLL_LIMIT     20U

#if TOF_DEBUG_LOG
#define TOF_LOG(...)      printf(__VA_ARGS__)
#else
#define TOF_LOG(...)      ((void)0)
#endif

// 常用寄存器定义
#define REG_IDENTIFICATION_MODEL_ID          0xC0
#define REG_IDENTIFICATION_REVISION_ID       0xC2
#define REG_PRE_RANGE_CONFIG_VCSEL_PERIOD    0x50
#define REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD  0x70
#define REG_SYSRANGE_START                   0x00
#define REG_SYSTEM_SEQUENCE_CONFIG           0x01
#define REG_RESULT_INTERRUPT_STATUS          0x13
#define REG_RESULT_RANGE_STATUS              0x14

/**
 * @brief 写 1 字节寄存器
 */
static void VL_WriteByte(u8 reg, u8 dat)
{
    soft_iic_write_reg(&tof_iic, reg, dat);
}

/**
 * @brief 读 1 字节寄存器
 */
static u8 VL_ReadByte(u8 reg)
{
    return soft_iic_read_reg(&tof_iic, reg);
}

/**
 * @brief 读 2 字节寄存器
 */
static u16 VL_ReadWord(u8 reg)
{
    return soft_iic_read_reg16(&tof_iic, reg);
}

/**
 * @brief 写 2 字节寄存器
 */
static void VL_WriteWord(u8 reg, u16 dat)
{
    soft_iic_write_reg16(&tof_iic, reg, dat);
}

/**
 * @brief 上电后等待 TOF 自检完成
 */
static void VL_WaitBoot(void)
{
    delay_ms(TOF_BOOT_WAIT_MS);
}

/**
 * @brief 数据初始化
 * @retval 0-成功
 */
static u8 VL_DataInit(void)
{
    VL_WriteByte(0x88, 0x00);

    VL_WriteByte(REG_SYSRANGE_START, 0x01);
    delay_ms(10);
    VL_WriteByte(REG_SYSRANGE_START, 0x00);
    delay_ms(10);

    VL_WriteByte(0x80, 0x01);
    VL_WriteByte(0xFF, 0x01);
    VL_WriteByte(0x00, 0x00);
    VL_WriteByte(0x91, VL_ReadByte(0x91));
    VL_WriteByte(0x00, 0x01);
    VL_WriteByte(0xFF, 0x00);
    VL_WriteByte(0x80, 0x00);

    return 0;
}

/**
 * @brief 静态配置初始化
 * @retval 0-成功
 */
static u8 VL_StaticInit(void)
{
    u8 i;
    u8 val1 = VL_ReadByte(0xC0);
    u8 val2 = VL_ReadByte(0xC1);
    TOF_LOG("NVM SPAD Info: 0xC0=0x%02X, 0xC1=0x%02X\r\n", val1, val2);

    VL_WriteByte(0xFF, 0x01);
    VL_WriteByte(0x4F, 0x00);
    VL_WriteByte(0x4E, 0x2C);
    VL_WriteByte(0xFF, 0x00);
    VL_WriteByte(0xB6, 0xB4);

    {
        u8 spad_map[6] = {0xCE, 0xCF, 0xFF, 0xFF, 0xFF, 0xFF};
        for (i = 0; i < 6; i++) {
            VL_WriteByte((u8)(0xB0 + i), spad_map[i]);
        }
    }

    VL_WriteByte(0xFF, 0x01);
    VL_WriteByte(0x00, 0x00);
    VL_WriteByte(0xFF, 0x00);
    VL_WriteByte(0x09, 0x04);
    VL_WriteByte(0x10, 0x00);
    VL_WriteByte(0x11, 0x00);
    VL_WriteByte(0x24, 0x01);
    VL_WriteByte(0x25, 0xFF);
    VL_WriteByte(0x75, 0x00);
    VL_WriteByte(0xFF, 0x01);
    VL_WriteByte(0x4E, 0x2C);
    VL_WriteByte(0x48, 0x00);
    VL_WriteByte(0x30, 0x20);
    VL_WriteByte(0xFF, 0x00);
    VL_WriteByte(0x30, 0x09);
    VL_WriteByte(0x54, 0x00);
    VL_WriteByte(0x31, 0x04);
    VL_WriteByte(0x32, 0x03);
    VL_WriteByte(0x40, 0x83);
    VL_WriteByte(0x46, 0x25);
    VL_WriteByte(0x60, 0x00);
    VL_WriteByte(0x27, 0x00);
    VL_WriteByte(0x50, 0x06);
    VL_WriteByte(0x51, 0x00);
    VL_WriteByte(0x52, 0x96);
    VL_WriteByte(0x56, 0x08);
    VL_WriteByte(0x57, 0x30);
    VL_WriteByte(0x61, 0x00);
    VL_WriteByte(0x62, 0x00);
    VL_WriteByte(0x64, 0x00);
    VL_WriteByte(0x65, 0x00);
    VL_WriteByte(0x66, 0xA0);
    VL_WriteByte(0xFF, 0x01);
    VL_WriteByte(0x22, 0x32);
    VL_WriteByte(0x47, 0x14);
    VL_WriteByte(0x49, 0xFF);
    VL_WriteByte(0x4A, 0x00);
    VL_WriteByte(0xFF, 0x00);
    VL_WriteByte(0x7A, 0x0A);
    VL_WriteByte(0x7B, 0x00);
    VL_WriteByte(0x78, 0x21);
    VL_WriteByte(0xFF, 0x01);
    VL_WriteByte(0x23, 0x34);
    VL_WriteByte(0x42, 0x00);
    VL_WriteByte(0x44, 0xFF);
    VL_WriteByte(0x45, 0x26);
    VL_WriteByte(0x46, 0x05);
    VL_WriteByte(0x40, 0x40);
    VL_WriteByte(0x0E, 0x06);
    VL_WriteByte(0x20, 0x1A);
    VL_WriteByte(0x43, 0x40);
    VL_WriteByte(0xFF, 0x00);
    VL_WriteByte(0x34, 0x03);
    VL_WriteByte(0x35, 0x44);
    VL_WriteByte(0xFF, 0x01);
    VL_WriteByte(0x31, 0x04);
    VL_WriteByte(0x4B, 0x09);
    VL_WriteByte(0x4C, 0x05);
    VL_WriteByte(0x4D, 0x04);
    VL_WriteByte(0xFF, 0x00);
    VL_WriteByte(0x44, 0x00);
    VL_WriteByte(0x45, 0x20);
    VL_WriteByte(0x47, 0x08);
    VL_WriteByte(0x48, 0x28);
    VL_WriteByte(0x67, 0x00);
    VL_WriteByte(0x70, 0x04);
    VL_WriteByte(0x71, 0x01);
    VL_WriteByte(0x72, 0xFE);
    VL_WriteByte(0x76, 0x00);
    VL_WriteByte(0x77, 0x00);
    VL_WriteByte(0xFF, 0x01);
    VL_WriteByte(0x0D, 0x01);
    VL_WriteByte(0xFF, 0x00);
    VL_WriteByte(0x80, 0x01);
    VL_WriteByte(0x01, 0xF8);
    VL_WriteByte(0xFF, 0x01);
    VL_WriteByte(0x8E, 0x01);
    VL_WriteByte(0x00, 0x01);
    VL_WriteByte(0xFF, 0x00);
    VL_WriteByte(0x80, 0x00);

    VL_WriteByte(0x0A, 0x04);
    VL_WriteByte(0x84, 0x00);
    VL_WriteByte(0x0B, 0x01);

    return 0;
}

/**
 * @brief 设置测量时间预算
 * @param budget_us 时间预算，单位 us
 * @retval 0-成功
 */
static u8 VL_SetMeasurementTimingBudgetMicroSeconds(u32 budget_us)
{
    (void)budget_us;

    VL_WriteByte(REG_PRE_RANGE_CONFIG_VCSEL_PERIOD, 0x0A);
    VL_WriteByte(REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD, 0x08);
    VL_WriteByte(0x1A, 0x0B);
    VL_WriteByte(0x1B, 0x00);

    return 0;
}

/**
 * @brief 执行参考校准
 * @retval 0-成功
 */
static u8 VL_PerformRefCalibration(void)
{
    u8 val;

    VL_WriteByte(0x01, 0x01);
    delay_ms(100);
    do {
        val = VL_ReadByte(0x00);
    } while (val & 0x01);

    VL_WriteByte(0x01, 0x02);
    delay_ms(100);
    do {
        val = VL_ReadByte(0x00);
    } while (val & 0x01);

    return 0;
}

/**
 * @brief 配置连续测量参数
 */
static void VL_ConfigureContinuous(void)
{
    VL_WriteWord(0x01, 0x0064);
}

/**
 * @brief 初始化 VL53L0X
 * @retval 0-成功，其它为错误码
 */
u8 VL53L0X_Init(void)
{
    u8 id;
    GPIO_InitTypeDef gpio_init = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();

    gpio_init.Pin = GPIO_PIN_8;
    gpio_init.Mode = GPIO_MODE_INPUT;
    gpio_init.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &gpio_init);

    soft_iic_init(&tof_iic, GPIOC, GPIO_PIN_9, GPIOA, GPIO_PIN_8, 0x29, 5);
    VL_WaitBoot();

    id = VL_ReadByte(REG_IDENTIFICATION_MODEL_ID);
    if (id != 0xEE) {
        printf("TOF_Init: Chip ID error: 0x%02X\r\n", id);
        return 1;
    }
    printf("TOF_Init: Chip ID OK: 0x%02X\r\n", id);

    if (VL_DataInit() != 0U) {
        printf("TOF_Init: Data init failed\r\n");
        return 2;
    }

    if (VL_StaticInit() != 0U) {
        printf("TOF_Init: Static init failed\r\n");
        return 3;
    }

    if (VL_SetMeasurementTimingBudgetMicroSeconds(33000U) != 0U) {
        printf("TOF_Init: Timing budget failed\r\n");
        return 4;
    }

    VL_PerformRefCalibration();
    VL_ConfigureContinuous();
    VL_WriteByte(REG_SYSTEM_SEQUENCE_CONFIG, 0xE8);
    delay_ms(10);

    printf("TOF_Init: Initialization completed\r\n");
    return 0;
}

/**
 * @brief 读取一次距离，返回值单位 mm
 * @retval 量程内距离，失败返回 0xFFFF
 */
u16 VL53L0X_ReadDistance(void)
{
    u16 dist;
    u16 timeout = TOF_RANGING_POLL_LIMIT;
    u8 status;

    VL_WriteByte(REG_SYSRANGE_START, 0x01);
    delay_ms(TOF_RANGING_KICK_DELAY_MS);
    VL_WriteByte(REG_SYSRANGE_START, 0x00);
    delay_ms(TOF_RANGING_KICK_DELAY_MS);

    VL_WriteByte(0x0B, 0x01);
    delay_ms(TOF_RANGING_PREP_DELAY_MS);

    VL_WriteByte(REG_SYSRANGE_START, 0x01);

    do {
        delay_ms(TOF_RANGING_POLL_DELAY_MS);
        status = VL_ReadByte(REG_RESULT_INTERRUPT_STATUS);
        if ((status & 0x07) != 0U) {
            break;
        }
    } while (--timeout);

    if (timeout == 0U) {
        TOF_LOG("VL53L0X: Measurement timeout\r\n");
        VL_WriteByte(0x0B, 0x01);
        return 0xFFFFU;
    }

    dist = VL_ReadWord(REG_RESULT_RANGE_STATUS + 10U);
    VL_WriteByte(0x0B, 0x01);

    if ((dist == 0U) || (dist > 8190U)) {
        TOF_LOG("VL53L0X: Invalid distance %u mm\r\n", dist);
        return 0xFFFFU;
    }

    return dist;
}

/**
 * @brief 检查 TOF 是否在线
 * @retval 1-在线，0-离线
 */
u8 VL53L0X_IsReady(void)
{
    u8 id = VL_ReadByte(REG_IDENTIFICATION_MODEL_ID);
    return (id == 0xEEU) ? 1U : 0U;
}

/**
 * @brief 读取 INT 引脚电平
 * @retval 引脚状态
 */
u8 VL53L0X_ReadINT(void)
{
    return VL53L0X_INT();
}

/**
 * @brief TOF 快速测试接口
 * @retval 距离值，单位 mm，失败返回 0xFFFF
 */
u16 TOF_QuickTest(void)
{
    return VL53L0X_ReadDistance();
}

/**
 * @brief TOF 诊断测试
 * @retval 0-通过，1-失败
 */
u8 TOF_DiagnosticTest(void)
{
    u8 result = 0;
    u8 id;
    u16 distance;
    u8 i;

    printf("\r\n=== TOF VL53L0X 诊断测试 ===\r\n");
    printf("1. 上电等待...\r\n");
    VL_WaitBoot();
    printf("   [OK] 上电等待完成\r\n");

    printf("2. I2C 通讯测试...\r\n");
    id = VL_ReadByte(REG_IDENTIFICATION_MODEL_ID);
    printf("   芯片 ID: 0x%02X ", id);
    if (id == 0xEEU) {
        printf("[OK] 正确\r\n");
    } else {
        printf("[ERR] 错误 (应为 0xEE)\r\n");
        result = 1;
    }

    printf("3. 版本信息: Rev=0x%02X\r\n", VL_ReadByte(REG_IDENTIFICATION_REVISION_ID));

    printf("4. 测距测试...\r\n");
    for (i = 0; i < 3; i++) {
        distance = VL53L0X_ReadDistance();
        if (distance != 0xFFFFU) {
            printf("   [%d] %u mm [OK]\r\n", i + 1, distance);
        } else {
            printf("   [%d] 失败\r\n", i + 1);
            result = 1;
        }
        delay_ms(200);
    }

    printf("=== 诊断 %s ===\r\n", result ? "失败" : "成功");
    return result;
}

/**
 * @brief 打印 TOF 当前状态信息
 * @retval 无
 */
void TOF_ReadStatusInfo(void)
{
    printf("\r\n=== TOF 状态信息 ===\r\n");
    printf("INT引脚: %s\r\n", VL53L0X_ReadINT() ? "高" : "低");
    printf("中断状态: 0x%02X\r\n", VL_ReadByte(REG_RESULT_INTERRUPT_STATUS));
    printf("测距状态: 0x%02X\r\n", VL_ReadByte(REG_RESULT_RANGE_STATUS));
    printf("系统范围: 0x%02X\r\n", VL_ReadByte(REG_SYSRANGE_START));
    printf("测量配置: 0x%02X\r\n", VL_ReadByte(REG_SYSTEM_SEQUENCE_CONFIG));
    printf("芯片ID: 0x%02X\r\n", VL_ReadByte(REG_IDENTIFICATION_MODEL_ID));
}
