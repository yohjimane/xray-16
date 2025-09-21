#pragma once

#include "Include/xrRender/Kinematics.h"
#include "xrCore/_fbox.h"
#include "ozz/animation/runtime/skeleton.h"

namespace XRay
{
namespace Animation
{

class OzzKinematics final : public IKinematics
{
public:
    OzzKinematics();
    ~OzzKinematics();

    // Bootstrap from converted `.ozz` assets.
    bool InitializeFromOzz(pcstr skeleton_path);

    // IKinematics implementation (stubbed for initial integration pass)
    void Bone_Calculate(CBoneData* bd, Fmatrix* parent) override;
    void Bone_GetAnimPos(Fmatrix& pos, u16 id, u8 channel_mask, bool ignore_callbacks) override;

    bool PickBone(const Fmatrix& parent_xform, pick_result& r, float dist, const Fvector& start,
        const Fvector& dir, u16 bone_id) override;
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
    IKinematicsAnimated* dcast_PKinematicsAnimated() override;

#ifdef DEBUG
    void DebugRender(Fmatrix& XFORM) override;
    shared_str getDebugName() override;
#endif

private:
    void NotImplemented(pcstr function_name) const;

private:
    CInifile* user_data_;
    accel bone_map_by_name_;
    accel bone_map_by_ptr_;
    xr_vector<CBoneInstance> bone_instances_;
    xr_vector<CBoneData*> bones_;
    xr_vector<Fobb> bone_boxes_;
    u16 root_bone_;
    u64 visible_mask_;
    UpdateCallback update_callback_;
    void* update_callback_param_;
    u32 last_update_time_;
    s32 visibility_counter_;
    Fbox cached_box_;
    ozz::animation::Skeleton skeleton_;
};

} // namespace Animation
} // namespace XRay
