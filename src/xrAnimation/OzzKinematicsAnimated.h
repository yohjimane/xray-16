#pragma once

#include "OzzAnimationSystem.h"
#include "xrCore/Animation/SkeletonMotions.hpp"
#include "Include/xrRender/KinematicsAnimated.h"
#include "Include/xrRender/animation_blend.h"

namespace XRay {
namespace Animation {

class OzzKinematicsAnimated : public IKinematicsAnimated {
private:
    std::unique_ptr<OzzAnimationSystem> animation_system_;
    
    struct XRayBlendCompat {
        OzzAnimationSystem::AnimationHandle* ozz_handle;
        MotionID motion_id;
        u16 bone_or_part;
        u8 channel;
        float blend_amount;
        float time_current;
        float time_total;
        CBlend::ECurvature blend_state;
        float blend_accrue;
        float blend_falloff;
        float blend_power;
        float speed;
        bool playing;
        bool stop_at_end_callback;
        bool stop_at_end;
        bool fall_at_end;
        u32 dwFrame;
        PlayCallback callback;
        void* callback_param;
    };
    
    std::vector<XRayBlendCompat> blend_pool_;
    std::vector<CBoneInstance> bone_instances_;
    std::vector<Fmatrix> bone_transforms_;
    
    IBlendDestroyCallback* blend_destroy_callback_;
    IUpdateTracksCallback* update_tracks_callback_;
    
    shared_motions motions_;
    CPartition partition_;
    
public:
    OzzKinematicsAnimated();
    ~OzzKinematicsAnimated() override;
    
    bool Initialize(const std::string& skeleton_path, const std::string& animations_path);
    bool LoadMotionSet(const shared_motions& motions);
    
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
    
    void LL_AddTransformToBone(KinematicsABT::additional_bone_transform& offset) override;
    void LL_ClearAdditionalTransform(u16 bone_id) override;
    
    IBlendDestroyCallback* GetBlendDestroyCallback() override;
    void SetBlendDestroyCallback(IBlendDestroyCallback* cb) override;
    void SetUpdateTracksCalback(IUpdateTracksCallback* callback) override;
    IUpdateTracksCallback* GetUpdateTracksCalback() override;
    
    MotionID LL_MotionID(LPCSTR B) override;
    u16 LL_PartID(LPCSTR B) override;
    
    CBlend* LL_PlayCycle(u16 partition, MotionID motion, BOOL bMixing, float blendAccrue, float blendFalloff,
        float Speed, BOOL noloop, PlayCallback Callback, LPVOID CallbackParam, u8 channel = 0) override;
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
    
    CBlend* PlayCycle(
        LPCSTR N, BOOL bMixIn = TRUE, PlayCallback Callback = nullptr, LPVOID CallbackParam = nullptr, u8 channel = 0) override;
    CBlend* PlayCycle(
        MotionID M, BOOL bMixIn = TRUE, PlayCallback Callback = nullptr, LPVOID CallbackParam = nullptr, u8 channel = 0) override;
    CBlend* PlayCycle(u16 partition, MotionID M, BOOL bMixIn = TRUE, PlayCallback Callback = nullptr,
        LPVOID CallbackParam = nullptr, u8 channel = 0) override;
    
    MotionID ID_FX(LPCSTR N) override;
    MotionID ID_FX_Safe(LPCSTR N) override;
    
    CBlend* PlayFX(LPCSTR N, float power_scale) override;
    CBlend* PlayFX(MotionID M, float power_scale) override;
    CBlend* PlayFX_Safe(cpcstr N, float power_scale) override;
    
    const CPartition& partitions() const override;
    
    IRenderVisual* dcast_RenderVisual() override;
    IKinematics* dcast_PKinematics() override;
    
    float get_animation_length(MotionID motion_ID) override;
    
    // Additional methods for ozz integration
    OzzAnimationSystem* GetOzzAnimationSystem() { return animation_system_.get(); }
    
private:
    XRayBlendCompat* CreateBlendFromOzz(OzzAnimationSystem::AnimationHandle* handle);
    void UpdateBlendStates(float dt);
    void ProcessBoneCallbacks();
    void ProcessBoneCallback(size_t bone_index, CBoneInstance& instance);
    void SyncBoneTransforms();
    
    CBlend* BlendCompatToCBlend(XRayBlendCompat* compat);
    XRayBlendCompat* FindBlendCompat(const CBlend* blend);
    
    void InitializeBoneInstances();
    void UpdateBoneInstances();
    
    MotionID CreateMotionID(const std::string& name);
    std::string GetMotionName(MotionID id);
    
    bool IsValidMotionID(MotionID id) const;
    bool IsValidBoneID(u16 bone_id) const;
    bool IsValidPartitionID(u16 partition_id) const;
};

} // namespace Animation
} // namespace XRay