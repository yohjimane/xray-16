#pragma once

#include "FHierrarhyVisual.h"

#include "xrAnimation/OzzKinematics.h"
#include "xrAnimation/OzzBundle.h"

#include "xrCore/_fbox.h"
#include "xrCore/_sphere.h"
#include "xrCore/xrDebug.h"

#include "xrCommon/xr_vector.h"

#include "ozz/base/io/archive.h"
#include "ozz/base/io/memory_stream.h"
#include "ozz/base/span.h"
#include "ozz/sample/framework/mesh.h"

#include <cstdint>
#include <filesystem>

namespace xray::render::RENDER_NAMESPACE
{
class COzzKinematicsVisual final : public FHierrarhyVisual
{
public:
    COzzKinematicsVisual();
    ~COzzKinematicsVisual() override;

    void Load(const char* N, IReader* data, u32 dwFlags) override;
    void Copy(dxRender_Visual* pFrom) override;
    void Release() override;

    bool LoadFromBundle(const char* name, const std::filesystem::path& path);

    IKinematics* dcast_PKinematics() override { return &kinematics_; }

    OzzKinematics& Kinematics() { return kinematics_; }
    const xr_vector<Fmatrix>& SkinningPalette() const { return bone_palette_; }
    const std::vector<ozz::sample::Mesh>& Meshes() const { return meshes_; }

private:
    void UpdateBounds();
    void UpdateSkinningPalette();

private:
    OzzKinematics kinematics_;
    xr_vector<std::uint8_t> skeleton_payload_;
    xr_vector<std::uint8_t> mesh_payload_;
    std::vector<ozz::sample::Mesh> meshes_;
    xr_vector<Fmatrix> bone_palette_;
};
} // namespace xray::render::RENDER_NAMESPACE
