#pragma once

#include "AnimationConverter.h"
#include "xrCore/Animation/SkeletonMotions.hpp"
#include "xrCore/Animation/Bone.hpp"
#include "xrCore/Animation/Motion.hpp"
#include "xrCommon/xr_unordered_map.h"
#include "Include/xrRender/animation_motion.h"

namespace XRay {
namespace Animation {

class OzzAnimationSystem {
public:
    OzzAnimationSystem();
    ~OzzAnimationSystem();

    struct AnimationHandle {
        size_t animation_index;
        float current_time;
        float weight;
        bool is_playing;
        bool is_looping;

        float speed = 1.0f;
        float power = 1.0f;
        float accrue = 0.2f;
        float falloff = 0.2f;
        float duration = 0.0f;

        u32 dwFrame = 0;
        PlayCallback callback = nullptr;
        void* callback_param = nullptr;

        u16 partition_id = 0;
        u8 channel = 0;
    };

    bool LoadSkeleton(const shared_str& skeleton_path);
    bool LoadAnimation(const shared_str& animation_path, const shared_str& name);
    bool LoadMetadata(const shared_str& metadata_path);

    AnimationHandle* PlayAnimation(const shared_str& name, float weight = 1.0f, bool loop = true);
    void StopAnimation(AnimationHandle* handle);
    void SetAnimationWeight(AnimationHandle* handle, float weight);
    void SetAnimationSpeed(AnimationHandle* handle, float speed);

    void Update(float delta_time);

    size_t GetBoneCount() const;
    const shared_str& GetBoneName(size_t bone_index) const;
    s16 GetBoneParent(size_t bone_index) const;
    Fmatrix GetBoneTransform(size_t bone_index) const;
    Fmatrix GetBoneLocalTransform(size_t bone_index) const;

    bool IsValidBoneIndex(size_t bone_index) const;
    size_t FindBoneIndex(const shared_str& bone_name) const;

    const ozz::animation::Skeleton* GetSkeleton() const { return skeleton_.get(); }
    const xr_vector<xr_unique_ptr<ozz::animation::Animation>>& GetAnimations() const { return animations_; }
    const XRayMetadata& GetMetadata() const { return metadata_; }

    static constexpr u8 MAX_CHANNELS = 4;
    static constexpr u16 MAX_PARTITIONS = 16;

    void SetChannelFactor(u8 channel, float factor);
    float GetChannelFactor(u8 channel) const;

    AnimationHandle* PlayAnimationOnPartition(
        const shared_str& name,
        u16 partition_id,
        float weight = 1.0f,
        bool loop = true,
        u8 channel = 0,
        PlayCallback callback = nullptr,
        void* callback_param = nullptr);

    void StopAnimationsOnPartition(u16 partition_id, u8 channel_mask = 0xFF);
    void SetPartitionMask(u16 partition_id, const xr_vector<u16>& bone_indices);

    size_t GetActiveAnimationCount() const;

    void ApplyAdditionalBoneTransform(u16 bone_id, const Fmatrix& transform);
    void ClearAdditionalBoneTransform(u16 bone_id);

    MotionID GetMotionID(const shared_str& name) const;
    bool HasAnimation(const shared_str& name) const;
    float GetAnimationLength(MotionID motion_id) const;

    void UpdateWithCallbacks(float dt);
    void SetBlendThreshold(float threshold);

    void EnableRootMotionExtraction(bool enable);
    Fmatrix GetRootMotionDelta();

private:
    xr_unique_ptr<ozz::animation::Skeleton> skeleton_;
    xr_vector<xr_unique_ptr<ozz::animation::Animation>> animations_;
    xr_unordered_map<shared_str, size_t> animation_name_to_index_;

    xr_vector<AnimationHandle> active_animations_;
    xr_vector<ozz::math::SoaTransform> local_transforms_;
    xr_vector<ozz::math::Float4x4> model_transforms_;

    xr_unique_ptr<ozz::animation::SamplingJob::Context> sampling_context_;
    xr_vector<ozz::animation::BlendingJob::Layer> blend_layers_;

    xr_vector<shared_str> bone_names_;
    xr_vector<s16> parent_indices_;
    XRayMetadata metadata_;

    struct PartitionMask {
        xr_vector<u16> bone_indices;
        xr_vector<ozz::math::SimdFloat4> joint_weights;
    };

    xr_vector<PartitionMask> partition_masks_;
    xr_vector<Fmatrix> additional_transforms_;
    float channel_factors_[MAX_CHANNELS] = {1.0f, 1.0f, 1.0f, 1.0f};
    xr_unordered_map<shared_str, u16> motion_map_;
    float blend_threshold_ = 0.01f;

    bool extract_root_motion_ = false;
    bool first_root_motion_frame_ = true;
    Fmatrix root_motion_delta_;

    void SampleAnimations();
    void BlendAnimations();
    void BlendAnimationsWithPartitions();
    void ComputeModelTransforms();
    void UpdateBoneMatrices();

    void InitializeBoneData();
    void UpdateAnimationHandle(AnimationHandle& handle, float delta_time);

    Fmatrix SoaTransformToMatrix(const ozz::math::SoaTransform& soa_transform, size_t joint_index) const;
    Fmatrix Float4x4ToMatrix(const ozz::math::Float4x4& ozz_matrix) const;
    ozz::math::Float4x4 MatrixToFloat4x4(const Fmatrix& matrix) const;

    bool ValidateAnimationHandle(const AnimationHandle* handle) const;
    void CleanupFinishedAnimations();

    void ApplyAdditionalTransforms();
    void CheckAnimationMarks(AnimationHandle& handle, float old_time, float new_time);
    void ExtractRootMotion();
};

} // namespace Animation
} // namespace XRay
