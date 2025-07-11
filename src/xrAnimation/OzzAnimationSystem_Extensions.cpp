#include "stdafx.h"
#include "OzzAnimationSystem.h"
#include "xrCore/FS.h"
#include "Include/xrRender/animation_blend.h"

namespace XRay {
namespace Animation {

// Extended functionality for OzzAnimationSystem

void OzzAnimationSystem::SetChannelFactor(u8 channel, float factor) {
    if (channel >= MAX_CHANNELS) {
        Msg("! OzzAnimationSystem: Invalid channel %d", channel);
        return;
    }
    
    channel_factors_[channel] = factor;
    Msg("* OzzAnimationSystem: Set channel %d factor to %.2f", channel, factor);
}

float OzzAnimationSystem::GetChannelFactor(u8 channel) const {
    if (channel >= MAX_CHANNELS) {
        return 1.0f;
    }
    
    return channel_factors_[channel];
}

OzzAnimationSystem::AnimationHandle* OzzAnimationSystem::PlayAnimationOnPartition(
    const std::string& name, 
    u16 partition_id, 
    float weight, 
    bool loop,
    u8 channel,
    PlayCallback callback,
    void* callback_param) 
{
    auto* handle = PlayAnimation(name, weight, loop);
    if (!handle) {
        return nullptr;
    }
    
    // Set partition and channel info
    handle->partition_id = partition_id;
    handle->channel = channel;
    handle->callback = callback;
    handle->callback_param = callback_param;
    
    // Apply channel factor to weight
    handle->weight *= GetChannelFactor(channel);
    
    return handle;
}

void OzzAnimationSystem::StopAnimationsOnPartition(u16 partition_id, u8 channel_mask) {
    for (auto& handle : active_animations_) {
        if (handle.partition_id == partition_id) {
            if (channel_mask & (1 << handle.channel)) {
                handle.is_playing = false;
                Msg("* OzzAnimationSystem: Stopped animation on partition %d, channel %d", 
                    partition_id, handle.channel);
            }
        }
    }
}

void OzzAnimationSystem::SetPartitionMask(u16 partition_id, const xr_vector<u16>& bone_indices) {
    if (partition_masks_.size() <= partition_id) {
        partition_masks_.resize(partition_id + 1);
    }
    
    auto& mask = partition_masks_[partition_id];
    mask.bone_indices = bone_indices;
    
    // Create ozz joint mask for efficient blending
    if (skeleton_) {
        const int num_joints = skeleton_->num_joints();
        const int num_soa_joints = skeleton_->num_soa_joints();
        
        mask.joint_weights.resize(num_soa_joints);
        
        // Initialize all weights to 0
        for (auto& soa_weight : mask.joint_weights) {
            soa_weight = ozz::math::simd_float4::zero();
        }
        
        // Set weight values for specified bones
        for (u16 bone_idx : bone_indices) {
            if (bone_idx < num_joints) {
                const int soa_index = bone_idx / 4;
                const int soa_lane = bone_idx % 4;
                
                // Set the appropriate weight in the SIMD vector
                float weight_values[4] = {0.0f, 0.0f, 0.0f, 0.0f};
                weight_values[soa_lane] = 1.0f;  // Full weight for this bone
                
                mask.joint_weights[soa_index] = ozz::math::simd_float4::Load(
                    weight_values[0], weight_values[1], weight_values[2], weight_values[3]);
            }
        }
    }
    
    Msg("* OzzAnimationSystem: Set partition %d mask with %d bones", 
        partition_id, bone_indices.size());
}

size_t OzzAnimationSystem::GetActiveAnimationCount() const {
    size_t count = 0;
    for (const auto& handle : active_animations_) {
        if (handle.is_playing) {
            count++;
        }
    }
    return count;
}

void OzzAnimationSystem::BlendAnimationsWithPartitions() {
    if (active_animations_.empty()) {
        return;
    }
    
    // Group animations by partition
    xr_map<u16, xr_vector<const AnimationHandle*>> partition_animations;
    
    for (const auto& handle : active_animations_) {
        if (!handle.is_playing) continue;
        partition_animations[handle.partition_id].push_back(&handle);
    }
    
    // Process each partition separately
    for (const auto& [partition_id, handles] : partition_animations) {
        if (handles.empty()) continue;
        
        // Set up blending layers for this partition
        blend_layers_.clear();
        blend_layers_.reserve(handles.size());
        
        for (const auto* handle : handles) {
            ozz::animation::BlendingJob::Layer layer;
            layer.transform = ozz::make_span(local_transforms_);
            layer.weight = handle->weight * channel_factors_[handle->channel];
            
            // Apply partition mask if available
            if (partition_id < partition_masks_.size() && 
                !partition_masks_[partition_id].joint_weights.empty()) {
                layer.joint_weights = ozz::make_span(partition_masks_[partition_id].joint_weights);
            }
            
            blend_layers_.push_back(layer);
        }
        
        // Run blending job for this partition
        ozz::animation::BlendingJob blending_job;
        blending_job.layers = ozz::make_span(blend_layers_);
        blending_job.rest_pose = skeleton_->joint_rest_poses();
        blending_job.output = ozz::make_span(local_transforms_);
        blending_job.threshold = blend_threshold_;
        
        if (!blending_job.Run()) {
            Msg("! OzzAnimationSystem: Failed to blend animations for partition %d", partition_id);
        }
    }
}

void OzzAnimationSystem::ApplyAdditionalBoneTransform(u16 bone_id, const Fmatrix& transform) {
    if (bone_id >= additional_transforms_.size()) {
        additional_transforms_.resize(bone_id + 1);
    }
    
    additional_transforms_[bone_id] = transform;
}

void OzzAnimationSystem::ClearAdditionalBoneTransform(u16 bone_id) {
    if (bone_id < additional_transforms_.size()) {
        additional_transforms_[bone_id].identity();
    }
}

void OzzAnimationSystem::ApplyAdditionalTransforms() {
    if (additional_transforms_.empty()) {
        return;
    }
    
    // Apply additional transforms after model space conversion
    for (u16 bone_id = 0; bone_id < additional_transforms_.size(); ++bone_id) {
        if (bone_id >= model_transforms_.size()) {
            break;
        }
        
        const Fmatrix& additional = additional_transforms_[bone_id];
        if (!fis_zero(additional._14) || !fis_zero(additional._24) || !fis_zero(additional._34) ||
            !fis_zero(additional._41) || !fis_zero(additional._42) || !fis_zero(additional._43) ||
            !fsimilar(additional._11, 1.0f) || !fsimilar(additional._22, 1.0f) || !fsimilar(additional._33, 1.0f) ||
            !fsimilar(additional._44, 1.0f)) {
            // Convert ozz transform to X-Ray matrix
            Fmatrix bone_transform = Float4x4ToMatrix(model_transforms_[bone_id]);
            
            // Apply additional transform
            bone_transform.mulB_43(additional);
            
            // Convert back to ozz format
            model_transforms_[bone_id] = MatrixToFloat4x4(bone_transform);
        }
    }
}

ozz::math::Float4x4 OzzAnimationSystem::MatrixToFloat4x4(const Fmatrix& matrix) const {
    ozz::math::Float4x4 result;
    
    // X-Ray matrices are row-major, ozz matrices are column-major
    result.cols[0] = ozz::math::simd_float4::Load(matrix._11, matrix._21, matrix._31, matrix._41);
    result.cols[1] = ozz::math::simd_float4::Load(matrix._12, matrix._22, matrix._32, matrix._42);
    result.cols[2] = ozz::math::simd_float4::Load(matrix._13, matrix._23, matrix._33, matrix._43);
    result.cols[3] = ozz::math::simd_float4::Load(matrix._14, matrix._24, matrix._34, matrix._44);
    
    return result;
}

MotionID OzzAnimationSystem::GetMotionID(const shared_str& name) const {
    auto it = motion_map_.find(name);
    if (it != motion_map_.end()) {
        return MotionID(0, it->second);  // slot 0 for ozz
    }
    
    return MotionID();  // Invalid
}

bool OzzAnimationSystem::HasAnimation(const shared_str& name) const {
    return motion_map_.find(name) != motion_map_.end();
}

float OzzAnimationSystem::GetAnimationLength(MotionID motion_id) const {
    if (!motion_id.valid() || motion_id.idx >= animations_.size()) {
        return 0.0f;
    }
    
    return animations_[motion_id.idx]->duration();
}

void OzzAnimationSystem::UpdateWithCallbacks(float dt) {
    // Update animation handles
    for (auto& handle : active_animations_) {
        if (!handle.is_playing) continue;
        
        float old_time = handle.current_time;
        UpdateAnimationHandle(handle, dt);
        
        // Check for animation marks/events
        if (handle.callback && handle.is_playing) {
            // Check if we passed any marks
            CheckAnimationMarks(handle, old_time, handle.current_time);
        }
    }
    
    // Continue with normal update
    CleanupFinishedAnimations();
    SampleAnimations();
    BlendAnimationsWithPartitions();
    ComputeModelTransforms();
    ApplyAdditionalTransforms();
}

void OzzAnimationSystem::CheckAnimationMarks(AnimationHandle& handle, float old_time, float new_time) {
    // This is where we would check for animation events/marks
    // For now, just check for loop points
    
    if (handle.is_looping && new_time < old_time) {
        // We looped - fire callback
        if (handle.callback) {
            CBlend blend;
            blend.timeCurrent = new_time;
            blend.timeTotal = handle.duration;
            blend.blendAmount = handle.weight;
            blend.speed = handle.speed;
            blend.playing = handle.is_playing;
            blend.CallbackParam = handle.callback_param;
            blend.motionID = MotionID(0, (u16)handle.animation_index);
            blend.bone_or_part = handle.partition_id;
            blend.channel = handle.channel;
            
            handle.callback(&blend);
        }
    }
}

void OzzAnimationSystem::SetBlendThreshold(float threshold) {
    blend_threshold_ = threshold;
    Msg("* OzzAnimationSystem: Set blend threshold to %.3f", threshold);
}

void OzzAnimationSystem::EnableRootMotionExtraction(bool enable) {
    extract_root_motion_ = enable;
    if (enable) {
        root_motion_delta_.identity();
    }
}

Fmatrix OzzAnimationSystem::GetRootMotionDelta() {
    Fmatrix result = root_motion_delta_;
    root_motion_delta_.identity();  // Reset after reading
    return result;
}

void OzzAnimationSystem::ExtractRootMotion() {
    if (!extract_root_motion_ || model_transforms_.empty()) {
        return;
    }
    
    // Extract motion from root bone (index 0)
    static Fmatrix last_root_transform;
    Fmatrix current_root = Float4x4ToMatrix(model_transforms_[0]);
    
    if (!first_root_motion_frame_) {
        // Calculate delta
        root_motion_delta_ = current_root;
        root_motion_delta_.mulB_43(last_root_transform.invert());
    } else {
        first_root_motion_frame_ = false;
    }
    
    last_root_transform = current_root;
}

} // namespace Animation
} // namespace XRay