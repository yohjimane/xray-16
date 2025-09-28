#pragma once

#include "FHierrarhyVisual.h"

#include "xrAnimation/OzzKinematics.h"
#include "xrAnimation/OzzBundle.h"
#include "xrCore/_fbox.h"
#include "xrCore/_sphere.h"
#include "xrCore/xrDebug.h"

#include "xrCommon/xr_vector.h"

#include "ozz/base/io/archive.h"
#include "ozz/base/io/stream.h"
#include "ozz/base/span.h"
#include "framework/mesh.h"

#include <cstdint>
#include <filesystem>
#include <memory>
#include <vector>

namespace xray::render::RENDER_NAMESPACE
{
using XRay::Animation::OzzKinematics;
class COzzSkinnedSurface;

class COzzKinematicsVisual final : public FHierrarhyVisual
{
public:
    COzzKinematicsVisual();
    ~COzzKinematicsVisual() override;

    void Load(const char* N, IReader* data, u32 dwFlags) override;
    void Copy(dxRender_Visual* pFrom) override;
    void Spawn() override;
    void Depart() override;
    void Release() override;

    bool LoadFromBundle(const char* name, const std::filesystem::path& path);

    IKinematics* dcast_PKinematics() override { return &kinematics_; }
    IKinematicsAnimated* dcast_PKinematicsAnimated() override { return &kinematics_; }

    OzzKinematics& Kinematics() { return kinematics_; }
    bool HasGeometry() const { return !meshes_.empty(); }
    bool IsKinematicsReady() const { return initialized_ && kinematics_.IsInitialized(); }
    const xr_vector<Fmatrix>& SkinningPalette();
    const xr_vector<ozz::sample::Mesh>& Meshes() const { return meshes_; }

    bool PlayLegacyMotion(const xr_string& motion_name);
    xr_vector<xr_string> LegacyMotionNames();

    void EnsureSkinningPalette();
    void OnPoseUpdated();
    void UpdateSkinningPalette();
    static void HandleKinematicsUpdated(IKinematics* kin);

    bool LoadAnimationFromFile(const std::filesystem::path& path);
    void StopAnimation();

private:
    void UpdateBounds();
    void DestroySurfaces();
    bool UpdateAnimation(float dt);
    bool InitializeFromPayload(bool spawn_children = false);

private:
    OzzKinematics kinematics_;
    xr_vector<std::uint8_t> skeleton_payload_;
    xr_vector<std::uint8_t> mesh_payload_;
    xr_vector<ozz::sample::Mesh> meshes_;
    xr_vector<COzzSkinnedSurface*> surfaces_;
    xr_vector<Fmatrix> bone_palette_;
    xr_vector<xr_string> motion_references_;
    u32 last_animation_update_frame_ = u32(-1);
    bool initialized_ = false;
};
} // namespace xray::render::RENDER_NAMESPACE
