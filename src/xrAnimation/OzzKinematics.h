#pragma once

#include <cstddef>

#include "Include/xrRender/Kinematics.h"
#include "Include/xrRender/KinematicsAnimated.h"
#include "xrCore/_fbox.h"
#include "xrCommon/xr_smart_pointers.h"
#include "xrCommon/xr_unordered_map.h"
#include "xrCommon/xr_set.h"

#include "OzzAnimationController.h"
#include "LegacyOmfConverter.h"

#include <filesystem>

#include "ozz/animation/runtime/sampling_job.h"
#include "ozz/animation/runtime/skeleton.h"
#include "ozz/base/maths/soa_transform.h"
#include "ozz/base/span.h"

namespace ozz
{
namespace io
{
class Stream;
} // namespace io
} // namespace ozz

namespace XRay
{
namespace Animation
{
class OzzKinematics final : public IKinematics, public IKinematicsAnimated
{
public:
    OzzKinematics();
    ~OzzKinematics() override;

    // Bootstrap from converted `.ozz` assets.
    bool InitializeFromOzz(pcstr skeleton_path);
    bool InitializeFromOzzBuffer(ozz::span<const std::byte> skeleton_data);

    // Pose management helpers.
    bool SetPoseLocals(ozz::span<const ozz::math::SoaTransform> locals);
    void ClearPose();
    const ozz::animation::Skeleton& Skeleton() const;
    ozz::animation::SamplingJob::Context& SamplingContext();

    // Builds a skinning palette from cached bone instances. When render_space is
    // true the output contains mRenderTransform (model-space skinning matrices),
    // otherwise the raw local-to-model transforms prior to render-space offsets.
    void BuildSkinningPalette(xr_vector<Fmatrix>& out_matrices, bool render_space) const;

    // Returns whether the current skeleton has any joints.
    bool HasBones() const { return initialized_ && !bone_instances_.empty(); }
    bool IsInitialized() const { return initialized_; }

    // Animation/runtime helpers mirrored from the legacy facade.
    void SetLegacyMotionReferences(const xr_vector<xr_string>& references);
    const xr_vector<xr_string>& LegacyMotionReferences() const { return motion_references_; }
    xr_vector<xr_string> LegacyMotionNames();
    bool PlayLegacyMotion(const xr_string& motion_name);
    bool LoadAnimationFromFile(const std::filesystem::path& path);
    void StopAnimation();
    bool AdvanceAnimation(float dt);
    bool HasActiveAnimation() const { return animation_applied_; }

    bool HasLoadedAnimation() const;

    // IKinematicsAnimated implementation
    void OnCalculateBones() override;
#ifdef DEBUG
    std::pair<LPCSTR, LPCSTR> LL_MotionDefName_dbg(MotionID ID) override;
    void LL_DumpBlends_dbg() override;
#endif
    u32 LL_PartBlendsCount(u32 bone_part_id) override;
    CBlend* LL_PartBlend(u32 bone_part_id, u32 n) override;
    void LL_IterateBlends(IterateBlendsCallback& callback) override;
    u16 LL_MotionsSlotCount() override;
    const shared_motions& LL_MotionsSlot(u16 idx) override;
    CMotionDef* LL_GetMotionDef(MotionID id) override;
    CMotion* LL_GetRootMotion(MotionID id) override;
    CMotion* LL_GetMotion(MotionID id, u16 bone_id) override;
    void LL_BuldBoneMatrixDequatize(const CBoneData* bd, u8 channel_mask, SKeyTable& keys) override;
    void LL_BoneMatrixBuild(CBoneInstance& bi, const Fmatrix* parent, const SKeyTable& keys) override;
    IBlendDestroyCallback* GetBlendDestroyCallback() override;
    void SetBlendDestroyCallback(IBlendDestroyCallback* cb) override;
    void SetUpdateTracksCalback(IUpdateTracksCallback* callback) override;
    IUpdateTracksCallback* GetUpdateTracksCalback() override;
    MotionID LL_MotionID(LPCSTR B) override;
    u16 LL_PartID(LPCSTR B) override;
    CBlend* LL_PlayCycle(u16 partition, MotionID motion, BOOL bMixing, float blendAccrue, float blendFalloff, float Speed,
        BOOL noloop, PlayCallback Callback, LPVOID CallbackParam, u8 channel = 0) override;
    CBlend* LL_PlayCycle(
        u16 partition, MotionID motion, BOOL bMixIn, PlayCallback Callback, LPVOID CallbackParam, u8 channel = 0) override;
    void LL_CloseCycle(u16 partition, u8 mask_channel = (1 << 0)) override;
    void LL_SetChannelFactor(u16 channel, float factor) override;
    void UpdateTracks() override;
    void LL_UpdateTracks(float dt, bool b_force, bool leave_blends) override;
    MotionID ID_Cycle(LPCSTR N) override;
    MotionID ID_Cycle_Safe(LPCSTR N) override;
    MotionID ID_Cycle(shared_str N) override;
    MotionID ID_Cycle_Safe(shared_str N) override;
    CBlend* PlayCycle(LPCSTR N, BOOL bMixIn = TRUE, PlayCallback Callback = nullptr, LPVOID CallbackParam = nullptr,
        u8 channel = 0) override;
    CBlend* PlayCycle(MotionID M, BOOL bMixIn = TRUE, PlayCallback Callback = nullptr, LPVOID CallbackParam = nullptr,
        u8 channel = 0) override;
    CBlend* PlayCycle(u16 partition, MotionID M, BOOL bMixIn = TRUE, PlayCallback Callback = nullptr,
        LPVOID CallbackParam = nullptr, u8 channel = 0) override;
    MotionID ID_FX(LPCSTR N) override;
    MotionID ID_FX_Safe(LPCSTR N) override;
    CBlend* PlayFX(LPCSTR N, float power_scale) override;
    CBlend* PlayFX(MotionID M, float power_scale) override;
    CBlend* PlayFX_Safe(cpcstr N, float power_scale) override;
    const CPartition& partitions() const override;
    float get_animation_length(MotionID motion_ID) override;

    // IKinematics implementation (stubbed for initial integration pass)
    void Bone_Calculate(CBoneData* bd, Fmatrix* parent) override;
    void Bone_GetAnimPos(Fmatrix& pos, u16 id, u8 channel_mask, bool ignore_callbacks) override;

    bool PickBone(const Fmatrix& parent_xform, pick_result& r, float dist, const Fvector& start, const Fvector& dir, u16 bone_id) override;
    void EnumBoneVertices(SEnumVerticesCallback& C, u16 bone_id) override;

    u16 LL_BoneID(LPCSTR B) override;
    u16 LL_BoneID(const shared_str& B) override;
    LPCSTR LL_BoneName_dbg(u16 ID) override;

    CInifile* LL_UserData() override;
    accel* LL_Bones() override;

    CBoneInstance& LL_GetBoneInstance(u16 bone_id) override;
    CBoneData& LL_GetData(u16 bone_id) override;
    const IBoneData& GetBoneData(u16 bone_id) const override;

    u16 LL_BoneCount() const override;
    u16 LL_VisibleBoneCount() override;

    Fmatrix& LL_GetTransform(u16 bone_id) override;
    const Fmatrix& LL_GetTransform(u16 bone_id) const override;

    Fmatrix& LL_GetTransform_R(u16 bone_id) override;
    Fobb& LL_GetBox(u16 bone_id) override;
    const Fbox& GetBox() const override;
    void LL_GetBindTransform(xr_vector<Fmatrix>& matrices) override;
    int LL_GetBoneGroups(xr_vector<xr_vector<u16>>& groups) override;

    u16 LL_GetBoneRoot() override;
    void LL_SetBoneRoot(u16 bone_id) override;

    BOOL LL_GetBoneVisible(u16 bone_id) override;
    void LL_SetBoneVisible(u16 bone_id, BOOL val, BOOL bRecursive) override;
    u64 LL_GetBonesVisible() override;
    void LL_SetBonesVisible(u64 mask) override;

    void LL_AddTransformToBone(KinematicsABT::additional_bone_transform& offset) override;
    void LL_ClearAdditionalTransform(u16 bone_id) override;

    void CalculateBones(BOOL bForceExact = FALSE) override;
    void CalculateBones_Invalidate() override;
    void Callback(UpdateCallback C, void* Param) override;

    void SetUpdateCallback(UpdateCallback pCallback) override;
    void SetUpdateCallbackParam(void* pCallbackParam) override;
    UpdateCallback GetUpdateCallback() override;
    void* GetUpdateCallbackParam() override;

    IRenderVisual* dcast_RenderVisual() override;
    IKinematics* dcast_PKinematics() override;
    IKinematicsAnimated* dcast_PKinematicsAnimated() override;

#ifdef DEBUG
    void DebugRender(Fmatrix& XFORM) override;
    shared_str getDebugName() override;
#endif

private:
    void NotImplemented(pcstr function_name) const;
    bool BuildBoneMetadata();
    void ResetRuntimeState();
    void ResetAnimationState();
    bool EnsureAnimationController();
    bool BuildLegacyMotionLibrary();
    bool LoadLegacyMotion(const xr_string& motion_name);
    xr_vector<xr_string> CollectSkeletonBoneNames() const;
    xr_vector<xr_string> ResolveMotionReference(const xr_string& reference) const;
    bool ConvertLegacyOmfFile(const xr_string& relative_path, const xr_vector<xr_string>& skeleton_bone_names);
    void ApplyAdditionalBoneTransforms(u16 bone_id, Fmatrix& transform) const;
    bool IsBoneVisible(size_t index) const;
    bool LoadSkeletonFromStream(ozz::io::Stream* stream, pcstr debug_source);
    bool FinalizeSkeletonInitialization(pcstr debug_source);

private:
    CInifile* user_data_;
    accel bone_map_by_name_;
    accel bone_map_by_ptr_;
    xr_vector<CBoneInstance> bone_instances_;
    xr_vector<CBoneData*> bones_;
    xr_vector<xr_unique_ptr<CBoneData>> bone_storage_;
    xr_vector<Fobb> bone_boxes_;
    u16 root_bone_;
    u64 visible_mask_;
    UpdateCallback update_callback_;
    void* update_callback_param_;
    u32 last_update_time_;
    s32 visibility_counter_;
    Fbox cached_box_;
    ozz::animation::Skeleton skeleton_;
    xr_vector<ozz::math::SoaTransform> sampled_locals_;
    xr_vector<ozz::math::Float4x4> model_transforms_;
    xr_vector<Fmatrix> cached_transforms_pre_callbacks_;
    xr_vector<KinematicsABT::additional_bone_transform> bone_offsets_;
    ozz::animation::SamplingJob::Context sampling_context_;
    xr_unique_ptr<OzzAnimationController> animation_controller_;
    xr_vector<xr_string> motion_references_;
    xr_unordered_map<xr_string, LegacyMotionMetadata> legacy_motion_metadata_;
    xr_unordered_map<xr_string, std::shared_ptr<ozz::animation::Animation>> legacy_motion_library_;
    xr_set<xr_string> converted_motion_sources_;
    IBlendDestroyCallback* blend_destroy_callback_ = nullptr;
    IUpdateTracksCallback* update_tracks_callback_ = nullptr;
    bool animation_applied_ = false;
    CPartition default_partition_{};
    bool initialized_ = false;
};
} // namespace Animation
} // namespace XRay
