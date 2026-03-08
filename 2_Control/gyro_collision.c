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
