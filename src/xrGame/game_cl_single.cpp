#include "pch_script.h"
#include "game_cl_single.h"
#include "UIGameSP.h"
#include "Actor.h"
#include "clsid_game.h"
#include "ai_space.h"
#include "alife_simulator.h"
#include "alife_time_manager.h"
#include "xrScriptEngine/ScriptExporter.hpp"
#include "xrCore/xr_token.h"

ESingleGameDifficulty g_SingleGameDifficulty = egdStalker;

extern const  xr_token difficulty_type_token[] = {
    {"gd_novice", egdNovice}, {"gd_stalker", egdStalker}, {"gd_veteran", egdVeteran}, {"gd_master", egdMaster}, {0, 0}};

game_cl_Single::game_cl_Single() {}
CUIGameCustom* game_cl_Single::createGameUI()
{
    CLASS_ID clsid = CLSID_GAME_UI_SINGLE;
    CUIGameSP* pUIGame = smart_cast<CUIGameSP*>(NEW_INSTANCE(clsid));
    R_ASSERT(pUIGame);
    pUIGame->Load();
    pUIGame->SetClGame(this);
    pUIGame->Init(0);
    pUIGame->Init(1);
    pUIGame->Init(2);
    return pUIGame;
}

pcstr game_cl_Single::getTeamSection(int Team) { return NULL; };
void game_cl_Single::OnDifficultyChanged() { Actor()->OnDifficultyChanged(); }
ALife::_TIME_ID game_cl_Single::GetGameTime()
{
    if (ai().get_alife() && ai().alife().initialized())
        return (ai().alife().time_manager().game_time());
    else
        return (inherited::GetGameTime());
}

ALife::_TIME_ID game_cl_Single::GetStartGameTime()
{
    if (ai().get_alife() && ai().alife().initialized())
        return (ai().alife().time_manager().start_game_time());
    else
        return (inherited::GetStartGameTime());
}

float game_cl_Single::GetGameTimeFactor()
{
    if (ai().get_alife() && ai().alife().initialized())
        return (ai().alife().time_manager().time_factor());
    else
        return (inherited::GetGameTimeFactor());
}

void game_cl_Single::SetGameTimeFactor(const float fTimeFactor)
{
    Level().Server->GetGameState()->SetGameTimeFactor(fTimeFactor);
}

ALife::_TIME_ID game_cl_Single::GetEnvironmentGameTime()
{
    if (ai().get_alife() && ai().alife().initialized())
        return (ai().alife().time_manager().game_time());
    else
        return (inherited::GetEnvironmentGameTime());
}

float game_cl_Single::GetEnvironmentGameTimeFactor()
{
    if (ai().get_alife() && ai().alife().initialized())
        return (ai().alife().time_manager().time_factor());
    else
        return (inherited::GetEnvironmentGameTimeFactor());
}

void game_cl_Single::SetEnvironmentGameTimeFactor(const float fTimeFactor)
{
    if (ai().get_alife() && ai().alife().initialized())
        Level().Server->GetGameState()->SetGameTimeFactor(fTimeFactor);
    else
        inherited::SetEnvironmentGameTimeFactor(fTimeFactor);
}

void game_cl_Single::SetEnvironmentGameTimeFactor(ALife::_TIME_ID GameTime, const float fTimeFactor)
{
    if (ai().get_alife() && ai().alife().initialized())
        Level().Server->GetGameState()->SetGameTimeFactor(GameTime, fTimeFactor);
    else
        inherited::SetEnvironmentGameTimeFactor(GameTime, fTimeFactor);
}

SCRIPT_EXPORT(CScriptGameDifficulty, (),
{
    using namespace luabind;

    class CScriptGameDifficulty
    {
    };

    module(luaState)
    [
        class_<CScriptGameDifficulty>("game_difficulty")
            .enum_("game_difficulty")
        [
            value("novice", int(egdNovice)),
            value("stalker", int(egdStalker)),
            value("veteran", int(egdVeteran)),
            value("master", int(egdMaster))
        ]
    ];
});

bool game_cl_Single::OnKeyboardPress(int key)
{
    if (kJUMP == key)
    {
        bool b_need_to_send_ready = false;

        IGameObject* curr = Level().CurrentControlEntity();
        if (!curr) return(false);

        bool is_actor = !!smart_cast<CActor*>(curr);
        bool is_spectator = !!smart_cast<CSpectator*>(curr);

        game_PlayerState* ps = local_player;

        if (is_spectator || (is_actor && ps && ps->testFlag(GAME_PLAYER_FLAG_VERY_VERY_DEAD)))
        {
            b_need_to_send_ready = true;
        }

        if (b_need_to_send_ready)
        {
            CGameObject* GO = smart_cast<CGameObject*>(curr);
            NET_Packet			P;
            GO->u_EventGen(P, GE_GAME_EVENT, GO->ID());
            P.w_u16(GAME_EVENT_PLAYER_READY);
            GO->u_EventSend(P);
            return				true;
        }
        else
        {
            return false;
        }
    };

    return inherited::OnKeyboardPress(key);
}


void game_cl_Single::OnConnected()
{
    if (GEnv.isDedicatedServer)
        return;

    inherited::OnConnected();

    luabind::functor<void>	funct;
    R_ASSERT(GEnv.ScriptEngine->functor("coop_game_cl.on_connected", funct));
    funct();
}

void game_cl_Single::net_import_state(NET_Packet& P)
{
    inherited::net_import_state(P);
}

void game_cl_Single::net_import_update(NET_Packet& P)
{
    inherited::net_import_update(P);
}
