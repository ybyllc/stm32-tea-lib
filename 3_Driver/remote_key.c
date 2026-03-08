#include "remote_key.h"
#include "main.h"

typedef struct
{
    GPIO_TypeDef *port;
    uint16_t pin;
} RemoteKeyPinMap;

/* 遥控器 R1A/M5N 四路按键输入映射表 */
static const RemoteKeyPinMap s_key_pin_map[REMOTE_KEY_MAX] =
{
    {D0_433_GPIO_Port, D0_433_Pin},
    {D1_433_GPIO_Port, D1_433_Pin},
    {D2_433_GPIO_Port, D2_433_Pin},
    {D3_433_GPIO_Port, D3_433_Pin}
};

/* 当前版本默认按键按下为低电平 */
#define REMOTE_KEY_ACTIVE_LEVEL     GPIO_PIN_RESET
#define REMOTE_KEY_DEBOUNCE_MS      20U
#define REMOTE_KEY_LONG_PRESS_MS    1000U

static u8 s_stable_pressed[REMOTE_KEY_MAX];
static u8 s_last_raw_pressed[REMOTE_KEY_MAX];
static u8 s_long_fired[REMOTE_KEY_MAX];
static uint32_t s_change_tick[REMOTE_KEY_MAX];
static uint32_t s_press_tick[REMOTE_KEY_MAX];
static RemoteKeyEventType s_event[REMOTE_KEY_MAX];

static u8 remote_key_read_pressed(RemoteKeyType key)
{
    GPIO_PinState state = HAL_GPIO_ReadPin(s_key_pin_map[key].port, s_key_pin_map[key].pin);
    return (state == REMOTE_KEY_ACTIVE_LEVEL) ? 1U : 0U;
}

void RemoteKey_Init(void)
{
    uint8_t i;
    uint32_t now = HAL_GetTick();

    for (i = 0; i < REMOTE_KEY_MAX; i++)
    {
        u8 pressed = remote_key_read_pressed((RemoteKeyType)i);
        s_stable_pressed[i] = pressed;
        s_last_raw_pressed[i] = pressed;
        s_long_fired[i] = 0;
        s_change_tick[i] = now;
        s_press_tick[i] = now;
        s_event[i] = REMOTE_KEY_EVENT_NONE;
    }
}

void RemoteKey_Scan(void)
{
    uint8_t i;
    uint32_t now = HAL_GetTick();

    for (i = 0; i < REMOTE_KEY_MAX; i++)
    {
        u8 raw = remote_key_read_pressed((RemoteKeyType)i);

        if (raw != s_last_raw_pressed[i])
        {
            s_last_raw_pressed[i] = raw;
            s_change_tick[i] = now;
        }

        if ((now - s_change_tick[i] >= REMOTE_KEY_DEBOUNCE_MS) && (raw != s_stable_pressed[i]))
        {
            s_stable_pressed[i] = raw;
            if (raw)
            {
                s_press_tick[i] = now;
                s_long_fired[i] = 0;
            }
            else
            {
                s_event[i] = s_long_fired[i] ? REMOTE_KEY_EVENT_LONG_PRESS_RELEASE : REMOTE_KEY_EVENT_SINGLE_CLICK;
                s_long_fired[i] = 0;
            }
        }

        if (s_stable_pressed[i] && (!s_long_fired[i]) && (now - s_press_tick[i] >= REMOTE_KEY_LONG_PRESS_MS))
        {
            s_long_fired[i] = 1;
            s_event[i] = REMOTE_KEY_EVENT_LONG_PRESS;
        }
    }
}

u8 RemoteKey_GetState(RemoteKeyType key)
{
    if ((uint8_t)key >= REMOTE_KEY_MAX)
    {
        return 0;
    }
    return s_stable_pressed[(uint8_t)key];
}

RemoteKeyEventType RemoteKey_GetEvent(RemoteKeyType key)
{
    RemoteKeyEventType event;

    if ((uint8_t)key >= REMOTE_KEY_MAX)
    {
        return REMOTE_KEY_EVENT_NONE;
    }

    event = s_event[(uint8_t)key];
    s_event[(uint8_t)key] = REMOTE_KEY_EVENT_NONE;
    return event;
}
