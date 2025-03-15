#pragma once
#include "NET_Shared.h"
#pragma warning(push)
#pragma warning(disable:4995)
#if defined(XR_PLATFORM_WINDOWS)
#include <DPlay/dplay8.h>
#endif
#pragma warning(pop)

struct SteamNetConnectionRealTimeStatus_t;

class XRNETSERVER_API IClientStatistic
{
	u32 mps_recive, mps_receive_base;
	u32 mps_send, mps_send_base;
	u32 dwBaseTime;
	CTimer* device_timer;
public:
	IClientStatistic(CTimer* timer) { ZeroMemory(this, sizeof(*this)); device_timer = timer; dwBaseTime = TimeGlobal(device_timer); }

	void Update(DPN_CONNECTION_INFO& CI);
    void Update(SteamNetConnectionRealTimeStatus_t& status);
    IC u32 getPing() const { return dwRoundTripLatencyMS; }
	IC u32 getBPS() const { return dwThroughputBPS; }
	IC u32 getPeakBPS() const { return dwPeakThroughputBPS; }
	IC u32 getDroppedCount() const { return dwPacketsDropped; }
	IC u32 getRetriedCount() const { return dwPacketsRetried; }
	IC u32 getMPS_Receive() const { return mps_recive; }
	IC u32 getMPS_Send() const { return mps_send; }
	IC u32 getReceivedPerSec() const { return dwBytesReceivedPerSec; }
	IC u32 getSendedPerSec() const { return dwBytesSendedPerSec; }
	IC void	Clear() { CTimer* timer = device_timer; ZeroMemory(this, sizeof(*this)); device_timer = timer; dwBaseTime = TimeGlobal(device_timer); }

	u32 dwRoundTripLatencyMS = 0;
	u32 dwThroughputBPS = 0;
	u32 dwPeakThroughputBPS = 0;
	u32 dwPacketsDropped = 0;
	u32 dwPacketsRetried = 0;

	u32 dwTimesBlocked = 0;
	u32 dwBytesSended = 0;
	u32 dwBytesSendedPerSec = 0;
	u32 dwBytesReceived = 0;
	u32 dwBytesReceivedPerSec = 0;
};
