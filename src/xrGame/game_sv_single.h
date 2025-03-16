#pragma once

#include "game_sv_base.h"
#include "cdkey_ban_list.h"

class xrServer;
class CALifeSimulator;

class game_sv_Single : public game_sv_GameState
{
private:
    typedef game_sv_GameState inherited;

protected:
    CALifeSimulator* m_alife_simulator;
    cdkey_ban_list m_cdkey_ban_list;

public:
    game_sv_Single();
    virtual ~game_sv_Single();

    virtual LPCSTR type_name() const { return "single"; };
    virtual void Create(shared_str& options);
    //	virtual		CSE_Abstract*		get_entity_from_eid		(u16 id);

    virtual void OnCreate(u16 id_who);
    virtual BOOL OnTouch(u16 eid_who, u16 eid_what, BOOL bForced = FALSE);
    virtual void OnDetach(u16 eid_who, u16 eid_what);

    // Main
    virtual void Update();
    virtual ALife::_TIME_ID GetStartGameTime();
    virtual ALife::_TIME_ID GetGameTime();
    virtual float GetGameTimeFactor();
    virtual void SetGameTimeFactor(const float fTimeFactor);
    virtual void SetGameTimeFactor(ALife::_TIME_ID GameTime, const float fTimeFactor) override;

    virtual ALife::_TIME_ID GetEnvironmentGameTime();
    virtual float GetEnvironmentGameTimeFactor();
    virtual void SetEnvironmentGameTimeFactor(const float fTimeFactor);

    virtual bool change_level(NET_Packet& net_packet, ClientID sender);
    virtual void save_game(NET_Packet& net_packet, ClientID sender);
    virtual bool load_game(NET_Packet& net_packet, ClientID sender);
    virtual void reload_game(NET_Packet& net_packet, ClientID sender);
    virtual void switch_distance(NET_Packet& net_packet, ClientID sender);
    virtual BOOL CanHaveFriendlyFire() { return FALSE; }
    virtual void teleport_object(NET_Packet& packet, u16 id);
    virtual void add_restriction(NET_Packet& packet, u16 id);
    virtual void remove_restriction(NET_Packet& packet, u16 id);
    virtual void remove_all_restrictions(NET_Packet& packet, u16 id);
    virtual bool custom_sls_default() { return !!m_alife_simulator; };
    virtual void sls_default();
    virtual shared_str level_name(const shared_str& server_options) const;
    virtual void on_death(CSE_Abstract* e_dest, CSE_Abstract* e_src);
    void restart_simulator(LPCSTR saved_game_name);

    void RespawnPlayer(ClientID id_who, bool NoSpectator);
    void SpawnPlayer(ClientID id, LPCSTR N);

    void __stdcall net_Relcase(IGameObject* O) {};

    virtual void OnPlayerReady(ClientID id_who);
    virtual void OnPlayerConnect(ClientID id_who);
    virtual void OnPlayerConnectFinished(ClientID id_who);
    virtual void OnPlayerDisconnect(ClientID id_who, LPSTR Name, u16 GameID);
    // virtual void OnPlayerKillPlayer(game_PlayerState* ps_killer, game_PlayerState* ps_killed, KILL_TYPE KillType, SPECIAL_KILL_TYPE SpecialKillType, CSE_Abstract* pWeaponA);
    virtual void OnEvent(NET_Packet& tNetPacket, u16 type, u32 time, ClientID sender);

    bool IsPlayerBanned(char const* hexstr_digest, shared_str& by_who);

    IC xrServer& server() const
    {
        VERIFY(m_server);
        return (*m_server);
    }

    IC CALifeSimulator& alife() const
    {
        VERIFY(m_alife_simulator);
        return (*m_alife_simulator);
    }
};
