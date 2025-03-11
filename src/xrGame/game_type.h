#pragma once

bool OnServer() noexcept;
bool OnClient() noexcept;
bool IsGameTypeSingle() noexcept;
bool CoopEnabled() noexcept;
bool CheckGameFlag(u64 flag) noexcept;
