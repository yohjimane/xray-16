#pragma once

namespace xray::render::RENDER_NAMESPACE
{
class COzzKinematicsVisual;

ENGINE_API void EnableOzzPaletteDebugDump(bool enabled);
ENGINE_API bool IsOzzPaletteDebugDumpEnabled();
ENGINE_API void RequestOzzPaletteDebugDump();
} // namespace xray::render::RENDER_NAMESPACE
