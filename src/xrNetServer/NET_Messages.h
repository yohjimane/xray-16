#pragma once
#include "NET_Common.h"

// Direct Play defines

#define DPNSEND_SYNC							DPNOP_SYNC
#define DPNSEND_NOCOPY							0x0001
#define DPNSEND_NOCOMPLETE						0x0002
#define DPNSEND_COMPLETEONPROCESS				0x0004
#define DPNSEND_GUARANTEED						0x0008
#define DPNSEND_NONSEQUENTIAL					0x0010
#define DPNSEND_NOLOOPBACK						0x0020
#define DPNSEND_PRIORITY_LOW					0x0040
#define DPNSEND_PRIORITY_HIGH					0x0080

#pragma pack(push, 1)
#define DPNSEND_IMMEDIATELLY 0x0100

IC u32 net_flags(
    bool bReliable = false, bool bSequental = true, bool bHighPriority = false, bool bSendImmediatelly = false)
{
#ifdef XR_PLATFORM_WINDOWS
    return (bReliable ? DPNSEND_GUARANTEED : DPNSEND_NOCOMPLETE) | (bSequental ? 0 : DPNSEND_NONSEQUENTIAL) |
        (bHighPriority ? DPNSEND_PRIORITY_HIGH : 0) | (bSendImmediatelly ? DPNSEND_IMMEDIATELLY : 0);
#else // XXX: multiplayer on other platforms...
    return 0;
#endif
}

struct MSYS_CONFIG
{
    u32 sign1; // 0x12071980;
    u32 sign2; // 0x26111975;
};

struct MSYS_PING
{
    u32 sign1; // 0x12071980;
    u32 sign2; // 0x26111975;
    u32 dwTime_ClientSend;
    u32 dwTime_Server;
    u32 dwTime_ClientReceive;
};

#pragma pack(pop)
