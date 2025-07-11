#pragma once

#include "AnimationConverter.h"
#include "xrCore/Animation/SkeletonMotions.hpp"
#include "xrCore/Animation/Bone.hpp"

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
        
        u32 dwFrame = 0;
        PlayCallback callback = nullptr;
        void* callback_param = nullptr;
    };
    
    bool LoadSkeleton(const std::string& skeleton_path);
    bool LoadAnimation(const std::string& animation_path, const std::string& name);
    bool LoadMetadata(const std::string& metadata_path);
    
    AnimationHandle* PlayAnimation(const std::string& name, float weight = 1.0f, bool loop = true);
    void StopAnimation(AnimationHandle* handle);
    void SetAnimationWeight(AnimationHandle* handle, float weight);
    void SetAnimationSpeed(AnimationHandle* handle, float speed);
    
    void Update(float delta_time);
    
    size_t GetBoneCount() const;
    const std::string& GetBoneName(size_t bone_index) const;
    s16 GetBoneParent(size_t bone_index) const;
    Fmatrix GetBoneTransform(size_t bone_index) const;
    Fmatrix GetBoneLocalTransform(size_t bone_index) const;
    
    bool IsValidBoneIndex(size_t bone_index) const;
    size_t FindBoneIndex(const std::string& bone_name) const;
    
    const ozz::animation::Skeleton* GetSkeleton() const { return skeleton_.get(); }
    const std::vector<std::unique_ptr<ozz::animation::Animation>>& GetAnimations() const { return animations_; }
    const XRayMetadata& GetMetadata() const { return metadata_; }
    
private:
    std::unique_ptr<ozz::animation::Skeleton> skeleton_;
    std::vector<std::unique_ptr<ozz::animation::Animation>> animations_;
    std::unordered_map<std::string, size_t> animation_name_to_index_;
    
    std::vector<AnimationHandle> active_animations_;
    std::vector<ozz::math::SoaTransform> local_transforms_;
    std::vector<ozz::math::Float4x4> model_transforms_;
    
    std::unique_ptr<ozz::animation::SamplingJob::Context> sampling_context_;
    std::vector<ozz::animation::BlendingJob::Layer> blend_layers_;
    
    std::vector<std::string> bone_names_;
    std::vector<s16> parent_indices_;
    XRayMetadata metadata_;
    
    void SampleAnimations();
    void BlendAnimations();
    void ComputeModelTransforms();
    void UpdateBoneMatrices();
    
    void InitializeBoneData();
    void UpdateAnimationHandle(AnimationHandle& handle, float delta_time);
    
    Fmatrix SoaTransformToMatrix(const ozz::math::SoaTransform& soa_transform, size_t joint_index) const;
    Fmatrix Float4x4ToMatrix(const ozz::math::Float4x4& ozz_matrix) const;
    
    bool ValidateAnimationHandle(const AnimationHandle* handle) const;
    void CleanupFinishedAnimations();
};

} // namespace Animation
} // namespace XRay