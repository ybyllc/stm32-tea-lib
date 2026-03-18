// imu陀螺仪 碰撞检测算法
/* ------------  头文件  ------------ */
#include "stdint.h"
#include "math.h"   // 只用 ABS 宏，可自己写
#include "gyro_collision.h"

// 已有变量
//int16_t acc[3];   // 原始加速度  g * 1000  (±16 g 量程)
//int16_t gyro[3];  // 原始角速度  °/s * 10   (±2000 °/s 量程)

/* ------------  宏定义  ------------ */
#define SAMPLE_HZ    100        // 检测频率 100Hz
#define WIN          8          // 方差窗长 80 ms
#define G2UNIT       1000       // 1 g = 1000
#define GYRO2UNIT    10         // 1 °/s = 10
#define ACC_THR_BASE 2800       // 0.28 g 突变，经验起点值
#define GYRO_EN_THR  4500       // 45 °/s 能量阈值

/* ------------  全局变量  ------------ */
static int16_t  accWin[WIN][3];
static uint8_t  winPtr = 0;
static int32_t  accSum[3] = {0};
static int32_t  accSqSum = 0;
static int32_t  gyroSum = 0;

static int32_t  varAcc = 0;        // 当前方差 * 1000
static int32_t  gyroEnergy = 0;    // 角速度能量 * 10
static int32_t  thrAcc = ACC_THR_BASE; // 动态阈值

static uint32_t learnCnt = 0;
static int32_t  learnSum = 0;
static int32_t  learnSq = 0;

static uint32_t sqrt32(uint32_t x);

/* ------------  算法入口  ------------ */
//int16_t acc[3];   // 原始加速度  g * 1000  (±16 g 量程)
//int16_t gyro[3];  // 原始角速度  °/s * 10   (±2000 °/s 量程)
/* 每 10 ms 在中断里调用一次 */
void CDK_Update(int16_t *accRaw, int16_t *gyroRaw)
{
    /* 1. 滑动窗更新 */
    int16_t *old = accWin[winPtr];
    for(int i=0;i<3;i++){
        accSum[i] += accRaw[i] - old[i];
        old[i] = accRaw[i];
    }
    winPtr = (winPtr+1) % WIN;

    /* 2. 计算加速度模长均值 */
    int32_t mx = accSum[0]/WIN;
    int32_t my = accSum[1]/WIN;
    int32_t mz = accSum[2]/WIN;
    int32_t mean = sqrt32(mx*mx + my*my + mz*mz);  // 见下方 sqrt32

    /* 3. 计算加速度模长方差（*1000） */
    int32_t sq = 0;
    for(int i=0;i<WIN;i++){
        int32_t dx = accWin[i][0] - mx;
        int32_t dy = accWin[i][1] - my;
        int32_t dz = accWin[i][2] - mz;
        sq += (dx*dx + dy*dy + dz*dz);
    }
    varAcc = sq / WIN;        // 单位 (g^2)*1000

    /* 4. 陀螺仪能量 */
    gyroEnergy = 0;
    for(int i=0;i<3;i++){
        int32_t g = gyroRaw[i];
        gyroEnergy += g*g;
    }
    gyroEnergy = sqrt32(gyroEnergy) / GYRO2UNIT;  // -> °/s

    /* 5. 阈值自学习（前 30 s） */
    if(learnCnt < SAMPLE_HZ*30){
        learnSum += varAcc;
        learnSq  += varAcc*varAcc;
        learnCnt++;
        int32_t mean = learnSum / learnCnt;
        int32_t var  = (learnSq - mean*mean*learnCnt) / learnCnt;
        thrAcc = mean + 6 * sqrt32(var);   // 6σ
        if(thrAcc < ACC_THR_BASE) thrAcc = ACC_THR_BASE;
    }

    /* 6. 碰撞判定 */
    static uint8_t collide = 0;
    if(varAcc > thrAcc && gyroEnergy > GYRO_EN_THR){
        collide = 3;   // 连续 3 帧确认
    }
    if(collide){
        collide--;
        if(collide == 1){
            CDK_Callback();   // 下方用户钩子
        }
    }
}

/* ------------  快速开方（32 位，0~2^30） ------------ */
static uint32_t sqrt32(uint32_t x)
{
    uint32_t res = 0;
    uint32_t bit = 1<<15;
    while(bit){
        uint32_t temp = res | bit;
        uint32_t g2   = temp*temp;
        if(x >= g2) res = temp;
        bit >>= 1;
    }
    return res;
}

/* ------------  用户钩子，在这里发 CAN、断油电、打双闪  ------------ */
void CDK_Callback(void)
{
    /* 你的业务代码，例如： */
    // BSP_CollisionAlert();   // 点亮 LED
    // CAN_Tx(COLLISION_ID, 1);
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);
}

/* ------------  姿态滤波（控制层算法）  ------------ */
#define GYRO_FILTER_GYRO_ALPHA          0.10f
#define GYRO_FILTER_ACC_ALPHA           0.10f
#define GYRO_FILTER_COMP_ALPHA          0.98f
#define GYRO_FILTER_BIAS_LERP_STATIC    0.002f
#define GYRO_FILTER_STILL_GYRO_THR      2.0f
#define GYRO_FILTER_STILL_ACC_THR       0.12f

typedef struct
{
    uint8_t inited;
    uint8_t ready;
    uint32_t last_ms;
    float gyro_bias[3];
    float gyro_lpf[3];
    float acc_lpf[3];
    float angle[3];
} GyroFilterState;

static GyroFilterState s_gyro_filter = {0};

static float Gyro_Filter_Wrap180(float deg)
{
    while (deg > 180.0f) {
        deg -= 360.0f;
    }
    while (deg < -180.0f) {
        deg += 360.0f;
    }
    return deg;
}

void Gyro_Filter_Reset(void)
{
    uint8_t i;

    s_gyro_filter.inited = 0;
    s_gyro_filter.ready = 0;
    s_gyro_filter.last_ms = 0;
    for (i = 0; i < 3; i++) {
        s_gyro_filter.gyro_bias[i] = 0.0f;
        s_gyro_filter.gyro_lpf[i] = 0.0f;
        s_gyro_filter.acc_lpf[i] = 0.0f;
        s_gyro_filter.angle[i] = 0.0f;
    }
}

uint8_t Gyro_Filter_IsReady(void)
{
    return s_gyro_filter.ready;
}

void Gyro_Filter_Update(float *acc_g, float *gyro_dps, float *angle_deg, float *yaw_deg)
{
    uint32_t now_ms;
    float dt_s;
    float ax, ay, az;
    float gx, gy, gz;
    float acc_norm;
    float gyro_abs_sum;
    float acc_roll_deg;
    float acc_pitch_deg;
    uint8_t i;

    if (acc_g == 0 || gyro_dps == 0 || angle_deg == 0 || yaw_deg == 0) {
        return;
    }

    now_ms = HAL_GetTick();
    if (!s_gyro_filter.inited) {
        s_gyro_filter.inited = 1;
        s_gyro_filter.ready = 1;
        s_gyro_filter.last_ms = now_ms;
        for (i = 0; i < 3; i++) {
            s_gyro_filter.gyro_lpf[i] = gyro_dps[i];
            s_gyro_filter.acc_lpf[i] = acc_g[i];
            s_gyro_filter.angle[i] = angle_deg[i];
        }
        return;
    }

    dt_s = (float)(now_ms - s_gyro_filter.last_ms) / 1000.0f;
    s_gyro_filter.last_ms = now_ms;
    if (dt_s <= 0.0f || dt_s > 0.2f) {
        dt_s = 0.01f;
    }

    ax = acc_g[0];
    ay = acc_g[1];
    az = acc_g[2];
    gx = gyro_dps[0];
    gy = gyro_dps[1];
    gz = gyro_dps[2];

    acc_norm = sqrtf(ax * ax + ay * ay + az * az);
    gyro_abs_sum = fabsf(gx) + fabsf(gy) + fabsf(gz);

    // 静止时慢速更新零偏，降低积分漂移
    if ((fabsf(acc_norm - 1.0f) < GYRO_FILTER_STILL_ACC_THR) &&
        (gyro_abs_sum < GYRO_FILTER_STILL_GYRO_THR)) {
        s_gyro_filter.gyro_bias[0] += GYRO_FILTER_BIAS_LERP_STATIC * (gx - s_gyro_filter.gyro_bias[0]);
        s_gyro_filter.gyro_bias[1] += GYRO_FILTER_BIAS_LERP_STATIC * (gy - s_gyro_filter.gyro_bias[1]);
        s_gyro_filter.gyro_bias[2] += GYRO_FILTER_BIAS_LERP_STATIC * (gz - s_gyro_filter.gyro_bias[2]);
    }

    gx -= s_gyro_filter.gyro_bias[0];
    gy -= s_gyro_filter.gyro_bias[1];
    gz -= s_gyro_filter.gyro_bias[2];

    s_gyro_filter.gyro_lpf[0] += GYRO_FILTER_GYRO_ALPHA * (gx - s_gyro_filter.gyro_lpf[0]);
    s_gyro_filter.gyro_lpf[1] += GYRO_FILTER_GYRO_ALPHA * (gy - s_gyro_filter.gyro_lpf[1]);
    s_gyro_filter.gyro_lpf[2] += GYRO_FILTER_GYRO_ALPHA * (gz - s_gyro_filter.gyro_lpf[2]);

    s_gyro_filter.acc_lpf[0] += GYRO_FILTER_ACC_ALPHA * (ax - s_gyro_filter.acc_lpf[0]);
    s_gyro_filter.acc_lpf[1] += GYRO_FILTER_ACC_ALPHA * (ay - s_gyro_filter.acc_lpf[1]);
    s_gyro_filter.acc_lpf[2] += GYRO_FILTER_ACC_ALPHA * (az - s_gyro_filter.acc_lpf[2]);

    acc_roll_deg = atan2f(s_gyro_filter.acc_lpf[1], s_gyro_filter.acc_lpf[2]) * 57.2957795f;
    acc_pitch_deg = atan2f(-s_gyro_filter.acc_lpf[0],
                           sqrtf(s_gyro_filter.acc_lpf[1] * s_gyro_filter.acc_lpf[1] +
                                 s_gyro_filter.acc_lpf[2] * s_gyro_filter.acc_lpf[2])) * 57.2957795f;

    s_gyro_filter.angle[0] = GYRO_FILTER_COMP_ALPHA * (s_gyro_filter.angle[0] + s_gyro_filter.gyro_lpf[0] * dt_s) +
                             (1.0f - GYRO_FILTER_COMP_ALPHA) * acc_roll_deg;
    s_gyro_filter.angle[1] = GYRO_FILTER_COMP_ALPHA * (s_gyro_filter.angle[1] + s_gyro_filter.gyro_lpf[1] * dt_s) +
                             (1.0f - GYRO_FILTER_COMP_ALPHA) * acc_pitch_deg;
    s_gyro_filter.angle[2] += s_gyro_filter.gyro_lpf[2] * dt_s;

    s_gyro_filter.angle[0] = Gyro_Filter_Wrap180(s_gyro_filter.angle[0]);
    s_gyro_filter.angle[1] = Gyro_Filter_Wrap180(s_gyro_filter.angle[1]);
    s_gyro_filter.angle[2] = Gyro_Filter_Wrap180(s_gyro_filter.angle[2]);

    gyro_dps[0] = s_gyro_filter.gyro_lpf[0];
    gyro_dps[1] = s_gyro_filter.gyro_lpf[1];
    gyro_dps[2] = s_gyro_filter.gyro_lpf[2];

    acc_g[0] = s_gyro_filter.acc_lpf[0];
    acc_g[1] = s_gyro_filter.acc_lpf[1];
    acc_g[2] = s_gyro_filter.acc_lpf[2];

    angle_deg[0] = s_gyro_filter.angle[0];
    angle_deg[1] = s_gyro_filter.angle[1];
    angle_deg[2] = s_gyro_filter.angle[2];
    *yaw_deg = s_gyro_filter.angle[2];
}
