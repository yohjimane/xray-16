#pragma once

#include "FHierrarhyVisual.h"

#include "xrAnimation/OzzKinematics.h"
#include "xrAnimation/OzzBundle.h"
#include "xrAnimation/OzzAnimationController.h"
#include "xrAnimation/LegacyOmfConverter.h"

#include "xrCore/_fbox.h"
#include "xrCore/_sphere.h"
#include "xrCore/xrDebug.h"

#include "xrCommon/xr_unordered_map.h"
#include "xrCommon/xr_set.h"
#include "xrCommon/xr_vector.h"
#include "xrCommon/xr_smart_pointers.h"

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
    void Release() override;

    bool LoadFromBundle(const char* name, const std::filesystem::path& path);

    IKinematics* dcast_PKinematics() override { return &kinematics_; }

    OzzKinematics& Kinematics() { return kinematics_; }
    bool HasGeometry() const { return !meshes_.empty(); }
    const xr_vector<Fmatrix>& SkinningPalette();
    const xr_vector<ozz::sample::Mesh>& Meshes() const { return meshes_; }

    bool PlayLegacyMotion(const xr_string& motion_name);
    xr_vector<xr_string> LegacyMotionNames() const;

    void EnsureSkinningPalette();
    void OnPoseUpdated();
    void UpdateSkinningPalette();
    static void HandleKinematicsUpdated(IKinematics* kin);

    bool LoadAnimationFromFile(const std::filesystem::path& path);
    void StopAnimation();

private:
    void UpdateBounds();
    void DestroySurfaces();
    void UpdateAnimation(float dt);
    void BuildLegacyMotionLibrary();
    xr_vector<xr_string> ResolveMotionReference(const xr_string& reference) const;
    bool ConvertLegacyOmfFile(const xr_string& relative_path, const xr_vector<xr_string>& skeleton_bone_names);
    xr_vector<xr_string> CollectSkeletonBoneNames() const;
    bool LoadLegacyMotion(const xr_string& motion_name);

private:
    OzzKinematics kinematics_;
    xr_vector<std::uint8_t> skeleton_payload_;
    xr_vector<std::uint8_t> mesh_payload_;
    xr_vector<ozz::sample::Mesh> meshes_;
    xr_vector<COzzSkinnedSurface*> surfaces_;
    xr_vector<Fmatrix> bone_palette_;
    xr_vector<xr_string> motion_references_;
    xr_unordered_map<xr_string, XRay::Animation::LegacyMotionMetadata> legacy_motion_metadata_;
    xr_unordered_map<xr_string, std::shared_ptr<ozz::animation::Animation>> legacy_motion_library_;
    xr_set<xr_string> converted_motion_sources_;
    bool palette_dirty_ = true;
    xr_unique_ptr<XRay::Animation::OzzAnimationController> animation_controller_;
    bool animation_applied_ = false;
    u32 last_animation_update_frame_ = u32(-1);
};
} // namespace xray::render::RENDER_NAMESPACE
