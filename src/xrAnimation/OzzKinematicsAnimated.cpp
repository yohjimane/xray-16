#include "stdafx.h"
#include "OzzKinematicsAnimated.h"
#include "AnimationConverter.h"
#include "xrCore/FS.h"

namespace XRay {
namespace Animation {

OzzKinematicsAnimated::OzzKinematicsAnimated()
    : animation_system_(std::make_unique<OzzAnimationSystem>())
    , blend_destroy_callback_(nullptr)
    , update_tracks_callback_(nullptr)
{
}

OzzKinematicsAnimated::~OzzKinematicsAnimated() = default;

bool OzzKinematicsAnimated::Initialize(const std::string& skeleton_path, const std::string& animations_path) {
    if (!animation_system_->LoadSkeleton(skeleton_path)) {
        Msg("! OzzKinematicsAnimated: Failed to load skeleton from %s", skeleton_path.c_str());
        return false;
    }

    // Load animations directory
    FS_FileSet file_set;
    FS.file_list(file_set, animations_path.c_str(), FS_ListFiles, "*.ozz");

    for (const auto& file : file_set) {
        std::string full_path = animations_path + "/" + file.name.c_str();
        std::string name = file.name.c_str();

        // Remove extension
        size_t dot_pos = name.find_last_of('.');
        if (dot_pos != std::string::npos) {
            name = name.substr(0, dot_pos);
        }

        if (!animation_system_->LoadAnimation(full_path, name)) {
            Msg("! OzzKinematicsAnimated: Failed to load animation %s", full_path.c_str());
        }
    }

    // Load metadata
    std::string metadata_path = animations_path + "/metadata.ini";
    animation_system_->LoadMetadata(metadata_path);

    InitializeBoneInstances();

    return true;
}

bool OzzKinematicsAnimated::LoadMotionSet(const shared_motions& motions) {
    motions_ = motions;
    partition_ = *motions.partition();
    return true;
}

void OzzKinematicsAnimated::OnCalculateBones() {
    // This is called by the engine when bone matrices need to be computed
    // The ozz animation system handles this internally
}

#ifdef DEBUG
std::pair<LPCSTR, LPCSTR> OzzKinematicsAnimated::LL_MotionDefName_dbg(MotionID ID) {
    static std::string motion_name = GetMotionName(ID);
    return std::make_pair(motion_name.c_str(), "ozz_motion");
}

void OzzKinematicsAnimated::LL_DumpBlends_dbg() {
    Msg("* OzzKinematicsAnimated: Active blends count: %d", blend_pool_.size());
    for (size_t i = 0; i < blend_pool_.size(); ++i) {
        const auto& blend = blend_pool_[i];
        Msg("  [%d] Motion: %s, Weight: %.3f, Time: %.3f/%.3f, Playing: %s",
            i, GetMotionName(blend.motion_id).c_str(), blend.blend_amount,
            blend.time_current, blend.time_total, blend.playing ? "YES" : "NO");
    }
}
#endif

u32 OzzKinematicsAnimated::LL_PartBlendsCount(u32 bone_part_id) {
    u32 count = 0;
    for (const auto& blend : blend_pool_) {
        if (blend.bone_or_part == bone_part_id) {
            count++;
        }
    }
    return count;
}

CBlend* OzzKinematicsAnimated::LL_PartBlend(u32 bone_part_id, u32 n) {
    u32 count = 0;
    for (auto& blend : blend_pool_) {
        if (blend.bone_or_part == bone_part_id) {
            if (count == n) {
                return BlendCompatToCBlend(&blend);
            }
            count++;
        }
    }
    return nullptr;
}

void OzzKinematicsAnimated::LL_IterateBlends(IterateBlendsCallback& callback) {
    for (auto& blend : blend_pool_) {
        CBlend* cblend = BlendCompatToCBlend(&blend);
        if (cblend) {
            callback(*cblend);
        }
    }
}

u16 OzzKinematicsAnimated::LL_MotionsSlotCount() {
    return 1; // We have one motion slot for ozz animations
}

const shared_motions& OzzKinematicsAnimated::LL_MotionsSlot(u16 idx) {
    return motions_;
}

CMotionDef* OzzKinematicsAnimated::LL_GetMotionDef(MotionID id) {
    if (!IsValidMotionID(id) || !motions_.motion_defs()) {
        return nullptr;
    }

    if (id.idx < motions_.motion_defs()->size()) {
        return &(*motions_.motion_defs())[id.idx];
    }

    return nullptr;
}

CMotion* OzzKinematicsAnimated::LL_GetRootMotion(MotionID id) {
    return nullptr; // ozz animations don't separate root motion
}

CMotion* OzzKinematicsAnimated::LL_GetMotion(MotionID id, u16 bone_id) {
    return nullptr; // ozz animations don't separate per-bone motion
}

void OzzKinematicsAnimated::LL_BuldBoneMatrixDequatize(const CBoneData* bd, u8 channel_mask, SKeyTable& keys) {
    // This is handled internally by ozz animation system
}

void OzzKinematicsAnimated::LL_BoneMatrixBuild(CBoneInstance& bi, const Fmatrix* parent, const SKeyTable& keys) {
    // This is handled internally by ozz animation system
}

void OzzKinematicsAnimated::LL_AddTransformToBone(KinematicsABT::additional_bone_transform& offset) {
    // Additional bone transforms not implemented for ozz yet
    Msg("! OzzKinematicsAnimated: Additional bone transforms not implemented");
}

void OzzKinematicsAnimated::LL_ClearAdditionalTransform(u16 bone_id) {
    // Additional bone transforms not implemented for ozz yet
}

IBlendDestroyCallback* OzzKinematicsAnimated::GetBlendDestroyCallback() {
    return blend_destroy_callback_;
}

void OzzKinematicsAnimated::SetBlendDestroyCallback(IBlendDestroyCallback* cb) {
    blend_destroy_callback_ = cb;
}

void OzzKinematicsAnimated::SetUpdateTracksCalback(IUpdateTracksCallback* callback) {
    update_tracks_callback_ = callback;
}

IUpdateTracksCallback* OzzKinematicsAnimated::GetUpdateTracksCalback() {
    return update_tracks_callback_;
}

MotionID OzzKinematicsAnimated::LL_MotionID(LPCSTR B) {
    return CreateMotionID(B);
}

u16 OzzKinematicsAnimated::LL_PartID(LPCSTR B) {
    return partition_.part_id(B);
}

CBlend* OzzKinematicsAnimated::LL_PlayCycle(u16 partition, MotionID motion, BOOL bMixing, float blendAccrue,
    float blendFalloff, float Speed, BOOL noloop, PlayCallback Callback, LPVOID CallbackParam, u8 channel) {

    std::string motion_name = GetMotionName(motion);
    if (motion_name.empty()) {
        Msg("! OzzKinematicsAnimated: Invalid motion ID");
        return nullptr;
    }

    auto* ozz_handle = animation_system_->PlayAnimation(motion_name, 1.0f, !noloop);
    if (!ozz_handle) {
        Msg("! OzzKinematicsAnimated: Failed to play animation %s", motion_name.c_str());
        return nullptr;
    }

    // Set animation parameters
    animation_system_->SetAnimationSpeed(ozz_handle, Speed);

    // Create blend compatibility wrapper
    XRayBlendCompat* blend_compat = CreateBlendFromOzz(ozz_handle);
    if (!blend_compat) {
        animation_system_->StopAnimation(ozz_handle);
        return nullptr;
    }

    // Set blend parameters
    blend_compat->motion_id = motion;
    blend_compat->bone_or_part = partition;
    blend_compat->channel = channel;
    blend_compat->blend_accrue = blendAccrue;
    blend_compat->blend_falloff = blendFalloff;
    blend_compat->speed = Speed;
    blend_compat->callback = Callback;
    blend_compat->callback_param = CallbackParam;

    return BlendCompatToCBlend(blend_compat);
}

CBlend* OzzKinematicsAnimated::LL_PlayCycle(u16 partition, MotionID motion, BOOL bMixIn,
    PlayCallback Callback, LPVOID CallbackParam, u8 channel) {

    return LL_PlayCycle(partition, motion, bMixIn, 0.2f, 0.2f, 1.0f, FALSE, Callback, CallbackParam, channel);
}

void OzzKinematicsAnimated::LL_CloseCycle(u16 partition, u8 mask_channel) {
    // Stop all animations for the specified partition and channel
    for (auto& blend : blend_pool_) {
        if (blend.bone_or_part == partition && (mask_channel & (1 << blend.channel))) {
            animation_system_->StopAnimation(blend.ozz_handle);
        }
    }
}

void OzzKinematicsAnimated::LL_SetChannelFactor(u16 channel, float factor) {
    // Set weight for all animations in channel
    for (auto& blend : blend_pool_) {
        if (blend.channel == channel) {
            animation_system_->SetAnimationWeight(blend.ozz_handle, factor);
        }
    }
}

void OzzKinematicsAnimated::UpdateTracks() {
    LL_UpdateTracks(Device.fTimeDelta, false, false);
}

void OzzKinematicsAnimated::LL_UpdateTracks(float dt, bool b_force, bool leave_blends) {
    // Call update tracks callback if set
    if (update_tracks_callback_) {
        if (!update_tracks_callback_->operator()(dt, *this)) {
            return;
        }
    }

    // Update ozz animation system
    animation_system_->Update(dt);

    // Update blend states
    UpdateBlendStates(dt);

    // Sync bone transforms
    SyncBoneTransforms();

    // Process bone callbacks
    ProcessBoneCallbacks();

    // Clean up finished blends
    if (!leave_blends) {
        blend_pool_.erase(
            std::remove_if(blend_pool_.begin(), blend_pool_.end(),
                [](const XRayBlendCompat& blend) {
                    return !blend.playing;
                }),
            blend_pool_.end()
        );
    }
}

MotionID OzzKinematicsAnimated::ID_Cycle(LPCSTR N) {
    return CreateMotionID(N);
}

MotionID OzzKinematicsAnimated::ID_Cycle_Safe(LPCSTR N) {
    return CreateMotionID(N);
}

MotionID OzzKinematicsAnimated::ID_Cycle(shared_str N) {
    return CreateMotionID(N.c_str());
}

MotionID OzzKinematicsAnimated::ID_Cycle_Safe(shared_str N) {
    return CreateMotionID(N.c_str());
}

CBlend* OzzKinematicsAnimated::PlayCycle(LPCSTR N, BOOL bMixIn, PlayCallback Callback, LPVOID CallbackParam, u8 channel) {
    MotionID motion_id = CreateMotionID(N);
    return LL_PlayCycle(0, motion_id, bMixIn, Callback, CallbackParam, channel);
}

CBlend* OzzKinematicsAnimated::PlayCycle(MotionID M, BOOL bMixIn, PlayCallback Callback, LPVOID CallbackParam, u8 channel) {
    return LL_PlayCycle(0, M, bMixIn, Callback, CallbackParam, channel);
}

CBlend* OzzKinematicsAnimated::PlayCycle(u16 partition, MotionID M, BOOL bMixIn, PlayCallback Callback, LPVOID CallbackParam, u8 channel) {
    return LL_PlayCycle(partition, M, bMixIn, Callback, CallbackParam, channel);
}

MotionID OzzKinematicsAnimated::ID_FX(LPCSTR N) {
    return CreateMotionID(N);
}

MotionID OzzKinematicsAnimated::ID_FX_Safe(LPCSTR N) {
    return CreateMotionID(N);
}

CBlend* OzzKinematicsAnimated::PlayFX(LPCSTR N, float power_scale) {
    MotionID motion_id = CreateMotionID(N);
    return PlayFX(motion_id, power_scale);
}

CBlend* OzzKinematicsAnimated::PlayFX(MotionID M, float power_scale) {
    return LL_PlayCycle(0, M, FALSE, 0.0f, 0.0f, 1.0f, TRUE, nullptr, nullptr, 0);
}

CBlend* OzzKinematicsAnimated::PlayFX_Safe(cpcstr N, float power_scale) {
    return PlayFX(N, power_scale);
}

const CPartition& OzzKinematicsAnimated::partitions() const {
    return partition_;
}

IRenderVisual* OzzKinematicsAnimated::dcast_RenderVisual() {
    return nullptr; // Not implemented for ozz
}

IKinematics* OzzKinematicsAnimated::dcast_PKinematics() {
    return nullptr; // Not implemented for ozz
}

float OzzKinematicsAnimated::get_animation_length(MotionID motion_ID) {
    std::string motion_name = GetMotionName(motion_ID);
    if (motion_name.empty()) {
        return 0.0f;
    }

    const auto& animations = animation_system_->GetAnimations();
    for (const auto& animation : animations) {
        if (animation) {
            return animation->duration();
        }
    }

    return 0.0f;
}

OzzKinematicsAnimated::XRayBlendCompat* OzzKinematicsAnimated::CreateBlendFromOzz(OzzAnimationSystem::AnimationHandle* handle) {
    if (!handle) {
        return nullptr;
    }

    XRayBlendCompat blend;
    blend.ozz_handle = handle;
    blend.blend_amount = handle->weight;
    blend.time_current = handle->current_time;
    blend.time_total = 0.0f; // Will be set later
    blend.blend_state = CBlend::eAccrue;
    blend.playing = handle->is_playing;
    blend.speed = handle->speed;

    blend_pool_.push_back(blend);
    return &blend_pool_.back();
}

void OzzKinematicsAnimated::UpdateBlendStates(float dt) {
    for (auto& blend : blend_pool_) {
        if (blend.ozz_handle) {
            blend.blend_amount = blend.ozz_handle->weight;
            blend.time_current = blend.ozz_handle->current_time;
            blend.playing = blend.ozz_handle->is_playing;
            blend.speed = blend.ozz_handle->speed;
        }
    }
}

void OzzKinematicsAnimated::SyncBoneTransforms() {
    if (!animation_system_) {
        return;
    }

    size_t bone_count = animation_system_->GetBoneCount();
    bone_transforms_.resize(bone_count);

    for (size_t i = 0; i < bone_count; ++i) {
        bone_transforms_[i] = animation_system_->GetBoneTransform(i);
    }
}

void OzzKinematicsAnimated::ProcessBoneCallbacks() {
    if (bone_instances_.empty()) {
        return;
    }

    for (size_t i = 0; i < bone_instances_.size(); ++i) {
        ProcessBoneCallback(i, bone_instances_[i]);
    }
}

void OzzKinematicsAnimated::ProcessBoneCallback(size_t bone_index, CBoneInstance& instance) {
    if (instance.callback()) {
        instance.callback()(&instance);
    }
}

void OzzKinematicsAnimated::InitializeBoneInstances() {
    if (!animation_system_) {
        return;
    }

    size_t bone_count = animation_system_->GetBoneCount();
    bone_instances_.resize(bone_count);
    bone_transforms_.resize(bone_count);

    for (size_t i = 0; i < bone_count; ++i) {
        bone_instances_[i].construct();
        bone_transforms_[i].identity();
    }
}

MotionID OzzKinematicsAnimated::CreateMotionID(const std::string& name) {
    MotionID id;
    id.slot = 0;
    id.idx = static_cast<u16>(std::hash<std::string>{}(name) % 65535);
    return id;
}

std::string OzzKinematicsAnimated::GetMotionName(MotionID id) {
    // In a real implementation, we'd maintain a mapping
    // For now, return empty string for invalid IDs
    return "";
}

bool OzzKinematicsAnimated::IsValidMotionID(MotionID id) const {
    return id.slot == 0 && id.idx != 0;
}

bool OzzKinematicsAnimated::IsValidBoneID(u16 bone_id) const {
    return animation_system_ && bone_id < animation_system_->GetBoneCount();
}

bool OzzKinematicsAnimated::IsValidPartitionID(u16 partition_id) const {
    return partition_id < partition_.count();
}

CBlend* OzzKinematicsAnimated::BlendCompatToCBlend(XRayBlendCompat* compat) {
    if (!compat) {
        return nullptr;
    }

    // Create a temporary CBlend for compatibility
    // In a real implementation, we'd maintain a pool of CBlend objects
    static CBlend temp_blend;

    temp_blend.blendAmount = compat->blend_amount;
    temp_blend.timeCurrent = compat->time_current;
    temp_blend.timeTotal = compat->time_total;
    temp_blend.motionID = compat->motion_id;
    temp_blend.bone_or_part = compat->bone_or_part;
    temp_blend.channel = compat->channel;
    temp_blend.blendAccrue = compat->blend_accrue;
    temp_blend.blendFalloff = compat->blend_falloff;
    temp_blend.blendPower = compat->blend_power;
    temp_blend.speed = compat->speed;
    temp_blend.playing = compat->playing;
    temp_blend.stop_at_end_callback = compat->stop_at_end_callback;
    temp_blend.stop_at_end = compat->stop_at_end;
    temp_blend.fall_at_end = compat->fall_at_end;
    temp_blend.dwFrame = compat->dwFrame;
    temp_blend.Callback = compat->callback;
    temp_blend.CallbackParam = compat->callback_param;

    return &temp_blend;
}

OzzKinematicsAnimated::XRayBlendCompat* OzzKinematicsAnimated::FindBlendCompat(const CBlend* blend) {
    if (!blend) {
        return nullptr;
    }

    for (auto& compat : blend_pool_) {
        if (compat.motion_id.slot == blend->motionID.slot &&
            compat.motion_id.idx == blend->motionID.idx) {
            return &compat;
        }
    }

    return nullptr;
}

} // namespace Animation
} // namespace XRay
