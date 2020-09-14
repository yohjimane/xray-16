#include "StdAfx.h"
#include "game_type.h"
#include "Level.h"

bool OnServer() noexcept
{
    return g_pGameLevel != nullptr ? Level().IsServer() : false;
}

bool OnClient() noexcept
{
    return g_pGameLevel != nullptr ? Level().IsClient() : false;
}

bool IsGameTypeSingle() noexcept
{
    return g_pGamePersistent->GameType() == eGameIDSingle;
}

bool CheckGameFlag(u64 flag) noexcept
{
    return g_game_flags[flag] & g_pGamePersistent->GameType();
}
