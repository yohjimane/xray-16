#pragma once

#include "FHierrarhyVisual.h"

#include "xrAnimation/OzzKinematics.h"
#include "xrAnimation/OzzBundle.h"

#include "xrCore/_fbox.h"
#include "xrCore/_sphere.h"
#include "xrCore/xrDebug.h"

#include "xrCommon/xr_vector.h"
#include "xrCommon/xr_smart_pointers.h"

#include "ozz/base/io/archive.h"
#include "ozz/base/io/memory_stream.h"
#include "ozz/base/span.h"
#include "ozz/sample/framework/mesh.h"

#include <cstdint>
#include <filesystem>
#include <memory>
#include <vector>

namespace xray::render::RENDER_NAMESPACE
{
class COzzSkinnedSurface;

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
    bool HasGeometry() const { return !meshes_.empty(); }
    const xr_vector<Fmatrix>& SkinningPalette();
    const xr_vector<ozz::sample::Mesh>& Meshes() const { return meshes_; }

    void EnsureSkinningPalette();
    void OnPoseUpdated();
    void UpdateSkinningPalette();
    static void HandleKinematicsUpdated(IKinematics* kin);

private:
    void UpdateBounds();

private:
    OzzKinematics kinematics_;
    xr_vector<std::uint8_t> skeleton_payload_;
    xr_vector<std::uint8_t> mesh_payload_;
    xr_vector<ozz::sample::Mesh> meshes_;
    xr_vector<xr_unique_ptr<COzzSkinnedSurface>> surfaces_;
    xr_vector<Fmatrix> bone_palette_;
    bool palette_dirty_ = true;
};
} // namespace xray::render::RENDER_NAMESPACE
