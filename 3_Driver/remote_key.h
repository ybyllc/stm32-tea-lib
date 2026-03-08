#ifndef REMOTE_KEY_H
#define REMOTE_KEY_H

#include "common.h"

typedef enum
{
    REMOTE_KEY_D0 = 0,
    REMOTE_KEY_D1,
    REMOTE_KEY_D2,
    REMOTE_KEY_D3,
    REMOTE_KEY_MAX
} RemoteKeyType;

typedef enum
{
    REMOTE_KEY_EVENT_NONE = 0,
    REMOTE_KEY_EVENT_SINGLE_CLICK,
    REMOTE_KEY_EVENT_LONG_PRESS,
    REMOTE_KEY_EVENT_LONG_PRESS_RELEASE
} RemoteKeyEventType;

void RemoteKey_Init(void);
void RemoteKey_Scan(void);
u8 RemoteKey_GetState(RemoteKeyType key);
RemoteKeyEventType RemoteKey_GetEvent(RemoteKeyType key);

#endif /* REMOTE_KEY_H */
