#pragma once

class ScriptEvent
{
public:
    int SenderID;
    NET_Packet Packet;
    NET_Packet GetPacket() { return Packet; }
};
