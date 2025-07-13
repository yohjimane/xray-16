#include "stdafx.h"
#include "OzzAnimationSystem.h"
#include "xrCore/FS.h"
#include "ozz/base/io/archive.h"
#include "ozz/base/io/stream.h"
#include "Include/xrRender/animation_blend.h"

namespace XRay {
namespace Animation {

OzzAnimationSystem::OzzAnimationSystem() {
    sampling_context_ = xr_make_unique<ozz::animation::SamplingJob::Context>();
}

OzzAnimationSystem::~OzzAnimationSystem() = default;

bool OzzAnimationSystem::LoadSkeleton(const shared_str& skeleton_path) {
    // For testing, check if file exists directly first
    FILE* test = fopen(skeleton_path.c_str(), "rb");
    if (test) {
        fclose(test);
    } else if (!FS.exist(skeleton_path.c_str())) {
        Msg("! OzzAnimationSystem: Skeleton file not found: %s", skeleton_path.c_str());
        return false;
    }

    try {
        // Load skeleton from file
        ozz::io::File file(skeleton_path.c_str(), "rb");
        if (!file.opened()) {
            Msg("! OzzAnimationSystem: Failed to open skeleton file: %s", skeleton_path.c_str());
            return false;
        }

        ozz::io::IArchive archive(&file);
        if (!archive.TestTag<ozz::animation::Skeleton>()) {
            Msg("! OzzAnimationSystem: Invalid skeleton file format: %s", skeleton_path.c_str());
            return false;
        }

        skeleton_ = xr_make_unique<ozz::animation::Skeleton>();
        archive >> *skeleton_;

        if (!skeleton_->num_joints()) {
            Msg("! OzzAnimationSystem: Skeleton has no joints: %s", skeleton_path.c_str());
            return false;
        }

        // Initialize bone data
        InitializeBoneData();

        // Allocate transform buffers
        local_transforms_.resize(skeleton_->num_soa_joints());
        model_transforms_.resize(skeleton_->num_joints());

        // Resize sampling context
        sampling_context_->Resize(skeleton_->num_joints());

        Msg("* OzzAnimationSystem: Loaded skeleton with %d joints from %s",
            skeleton_->num_joints(), skeleton_path.c_str());

        return true;

    } catch (...) {
        Msg("! OzzAnimationSystem: Exception loading skeleton");
        return false;
    }
}

bool OzzAnimationSystem::LoadAnimation(const shared_str& animation_path, const shared_str& name) {
    // For testing, check if file exists directly first
    FILE* test = fopen(animation_path.c_str(), "rb");
    if (test) {
        fclose(test);
    } else if (!FS.exist(animation_path.c_str())) {
        Msg("! OzzAnimationSystem: Animation file not found: %s", animation_path.c_str());
        return false;
    }

    if (!skeleton_) {
        Msg("! OzzAnimationSystem: Skeleton must be loaded before animations");
        return false;
    }

    try {
        // Load animation from file
        ozz::io::File file(animation_path.c_str(), "rb");
        if (!file.opened()) {
            Msg("! OzzAnimationSystem: Failed to open animation file: %s", animation_path.c_str());
            return false;
        }

        ozz::io::IArchive archive(&file);
        if (!archive.TestTag<ozz::animation::Animation>()) {
            Msg("! OzzAnimationSystem: Invalid animation file format: %s", animation_path.c_str());
            return false;
        }

        auto animation = xr_make_unique<ozz::animation::Animation>();
        archive >> *animation;

        if (animation->num_tracks() != skeleton_->num_joints()) {
            Msg("! OzzAnimationSystem: Animation track count (%d) doesn't match skeleton joint count (%d)",
                animation->num_tracks(), skeleton_->num_joints());
            return false;
        }

        // Store animation
        size_t animation_index = animations_.size();
        animations_.push_back(std::move(animation));
        animation_name_to_index_[name] = animation_index;

        // Also store in motion map for X-Ray compatibility
        motion_map_[shared_str(name.c_str())] = static_cast<u16>(animation_index);

        Msg("* OzzAnimationSystem: Loaded animation '%s' with duration %.2fs from %s",
            name.c_str(), animations_[animation_index]->duration(), animation_path.c_str());

        return true;

    } catch (...) {
        Msg("! OzzAnimationSystem: Exception loading animation");
        return false;
    }
}

bool OzzAnimationSystem::LoadMetadata(const shared_str& metadata_path) {
    return metadata_.Load(metadata_path);
}

OzzAnimationSystem::AnimationHandle* OzzAnimationSystem::PlayAnimation(const shared_str& name, float weight, bool loop) {
    auto it = animation_name_to_index_.find(name);
    if (it == animation_name_to_index_.end()) {
        Msg("! OzzAnimationSystem: Animation '%s' not found", name.c_str());
        return nullptr;
    }

    if (it->second >= animations_.size()) {
        Msg("! OzzAnimationSystem: Invalid animation index for '%s'", name.c_str());
        return nullptr;
    }

    // Create new animation handle
    AnimationHandle handle;
    handle.animation_index = it->second;
    handle.current_time = 0.0f;
    handle.weight = weight;
    handle.is_playing = true;
    handle.is_looping = loop;
    handle.duration = animations_[it->second]->duration();

    // Apply metadata if available
    auto metadata_it = metadata_.motion_params.find(name);
    if (metadata_it != metadata_.motion_params.end()) {
        const auto& params = metadata_it->second;
        handle.speed = params.speed;
        handle.power = params.power;
        handle.accrue = params.accrue;
        handle.falloff = params.falloff;
    }

    active_animations_.push_back(handle);

    Msg("* OzzAnimationSystem: Playing animation '%s' with weight %.2f", name.c_str(), weight);

    return &active_animations_.back();
}

void OzzAnimationSystem::StopAnimation(AnimationHandle* handle) {
    if (!ValidateAnimationHandle(handle)) {
        return;
    }

    handle->is_playing = false;
    Msg("* OzzAnimationSystem: Stopped animation");
}

void OzzAnimationSystem::SetAnimationWeight(AnimationHandle* handle, float weight) {
    if (!ValidateAnimationHandle(handle)) {
        return;
    }

    handle->weight = weight;
}

void OzzAnimationSystem::SetAnimationSpeed(AnimationHandle* handle, float speed) {
    if (!ValidateAnimationHandle(handle)) {
        return;
    }

    handle->speed = speed;
}

void OzzAnimationSystem::Update(float delta_time) {
    if (!skeleton_ || animations_.empty()) {
        return;
    }

    // Update animation handles
    for (auto& handle : active_animations_) {
        if (handle.is_playing) {
            UpdateAnimationHandle(handle, delta_time);
        }
    }

    // Clean up finished animations
    CleanupFinishedAnimations();

    // Sample animations
    SampleAnimations();

    // Blend animations
    BlendAnimations();

    // Compute model transforms
    ComputeModelTransforms();
}

void OzzAnimationSystem::UpdateAnimationHandle(AnimationHandle& handle, float delta_time) {
    if (handle.animation_index >= animations_.size()) {
        return;
    }

    const auto& animation = animations_[handle.animation_index];

    // Update time
    handle.current_time += delta_time * handle.speed;

    // Handle looping
    if (handle.current_time >= animation->duration()) {
        if (handle.is_looping) {
            handle.current_time = fmod(handle.current_time, animation->duration());
        } else {
            handle.current_time = animation->duration();
            handle.is_playing = false;

            // Call callback if set
            if (handle.callback) {
                // Convert to X-Ray CBlend format for callback compatibility
                CBlend blend;
                blend.timeCurrent = handle.current_time;
                blend.timeTotal = animation->duration();
                blend.blendAmount = handle.weight;
                blend.speed = handle.speed;
                blend.playing = handle.is_playing;
                blend.CallbackParam = handle.callback_param;

                handle.callback(&blend);
            }
        }
    }
}

void OzzAnimationSystem::SampleAnimations() {
    // Sample each active animation
    for (const auto& handle : active_animations_) {
        if (!handle.is_playing || handle.animation_index >= animations_.size()) {
            continue;
        }

        const auto& animation = animations_[handle.animation_index];

        // Set up sampling job
        ozz::animation::SamplingJob sampling_job;
        sampling_job.animation = animation.get();
        sampling_job.context = sampling_context_.get();
        sampling_job.ratio = handle.current_time / animation->duration();
        sampling_job.output = ozz::make_span(local_transforms_);

        // Sample the animation
        if (!sampling_job.Run()) {
            Msg("! OzzAnimationSystem: Failed to sample animation");
        }
    }
}

void OzzAnimationSystem::BlendAnimations() {
    // Set up blending layers
    blend_layers_.clear();
    
    // Count active animations
    size_t active_count = 0;
    for (const auto& handle : active_animations_) {
        if (handle.is_playing) {
            active_count++;
        }
    }
    
    if (active_count == 0) {
        // No active animations - use bind pose
        // Copy rest pose to local transforms
        const auto& rest_poses = skeleton_->joint_rest_poses();
        std::copy(rest_poses.begin(), rest_poses.end(), local_transforms_.begin());
        return;
    }

    blend_layers_.reserve(active_count);

    for (const auto& handle : active_animations_) {
        if (!handle.is_playing) {
            continue;
        }

        ozz::animation::BlendingJob::Layer layer;
        layer.transform = ozz::make_span(local_transforms_);
        layer.weight = handle.weight;

        blend_layers_.push_back(layer);
    }

    // Set up blending job
    ozz::animation::BlendingJob blending_job;
    blending_job.layers = ozz::make_span(blend_layers_);
    blending_job.rest_pose = skeleton_->joint_rest_poses();
    blending_job.output = ozz::make_span(local_transforms_);

    // Blend animations
    if (!blending_job.Run()) {
        Msg("! OzzAnimationSystem: Failed to blend animations");
    }
}

void OzzAnimationSystem::ComputeModelTransforms() {
    // Set up local to model job
    ozz::animation::LocalToModelJob local_to_model_job;
    local_to_model_job.skeleton = skeleton_.get();
    local_to_model_job.input = ozz::make_span(local_transforms_);
    local_to_model_job.output = ozz::make_span(model_transforms_);

    // Compute model transforms
    if (!local_to_model_job.Run()) {
        Msg("! OzzAnimationSystem: Failed to compute model transforms");
    }
}

void OzzAnimationSystem::InitializeBoneData() {
    if (!skeleton_) {
        return;
    }

    // Extract bone names and parent indices
    bone_names_.clear();
    parent_indices_.clear();
    bone_names_.reserve(skeleton_->num_joints());
    parent_indices_.reserve(skeleton_->num_joints());

    for (int i = 0; i < skeleton_->num_joints(); ++i) {
        bone_names_.push_back(shared_str(skeleton_->joint_names()[i]));
        parent_indices_.push_back(skeleton_->joint_parents()[i]);
    }
}

size_t OzzAnimationSystem::GetBoneCount() const {
    return skeleton_ ? skeleton_->num_joints() : 0;
}

const shared_str& OzzAnimationSystem::GetBoneName(size_t bone_index) const {
    static const shared_str empty_name;

    if (!IsValidBoneIndex(bone_index)) {
        return empty_name;
    }

    return bone_names_[bone_index];
}

s16 OzzAnimationSystem::GetBoneParent(size_t bone_index) const {
    if (!IsValidBoneIndex(bone_index)) {
        return -1;
    }

    return parent_indices_[bone_index];
}

Fmatrix OzzAnimationSystem::GetBoneTransform(size_t bone_index) const {
    if (!IsValidBoneIndex(bone_index) || bone_index >= model_transforms_.size()) {
        Fmatrix identity;
        identity.identity();
        return identity;
    }

    return Float4x4ToMatrix(model_transforms_[bone_index]);
}

Fmatrix OzzAnimationSystem::GetBoneLocalTransform(size_t bone_index) const {
    if (!IsValidBoneIndex(bone_index)) {
        Fmatrix identity;
        identity.identity();
        return identity;
    }

    return SoaTransformToMatrix(local_transforms_[bone_index / 4], bone_index % 4);
}

bool OzzAnimationSystem::IsValidBoneIndex(size_t bone_index) const {
    return skeleton_ && bone_index < static_cast<size_t>(skeleton_->num_joints());
}

size_t OzzAnimationSystem::FindBoneIndex(const shared_str& bone_name) const {
    for (size_t i = 0; i < bone_names_.size(); ++i) {
        if (bone_names_[i].equal(bone_name)) {
            return i;
        }
    }
    return static_cast<size_t>(-1);
}

Fmatrix OzzAnimationSystem::SoaTransformToMatrix(const ozz::math::SoaTransform& soa_transform, size_t joint_index) const {
    Fmatrix result;
    result.identity();

    // SoA stores 4 joints per SimdFloat4, access based on joint_index % 4
    const size_t soa_index = joint_index % 4;

    // Extract translation
    float tx = (soa_index == 0) ? ozz::math::GetX(soa_transform.translation.x) :
               (soa_index == 1) ? ozz::math::GetY(soa_transform.translation.x) :
               (soa_index == 2) ? ozz::math::GetZ(soa_transform.translation.x) :
                                  ozz::math::GetW(soa_transform.translation.x);
    float ty = (soa_index == 0) ? ozz::math::GetX(soa_transform.translation.y) :
               (soa_index == 1) ? ozz::math::GetY(soa_transform.translation.y) :
               (soa_index == 2) ? ozz::math::GetZ(soa_transform.translation.y) :
                                  ozz::math::GetW(soa_transform.translation.y);
    float tz = (soa_index == 0) ? ozz::math::GetX(soa_transform.translation.z) :
               (soa_index == 1) ? ozz::math::GetY(soa_transform.translation.z) :
               (soa_index == 2) ? ozz::math::GetZ(soa_transform.translation.z) :
                                  ozz::math::GetW(soa_transform.translation.z);
    result.c.set(tx, ty, tz);

    // Extract rotation
    float qx = (soa_index == 0) ? ozz::math::GetX(soa_transform.rotation.x) :
               (soa_index == 1) ? ozz::math::GetY(soa_transform.rotation.x) :
               (soa_index == 2) ? ozz::math::GetZ(soa_transform.rotation.x) :
                                  ozz::math::GetW(soa_transform.rotation.x);
    float qy = (soa_index == 0) ? ozz::math::GetX(soa_transform.rotation.y) :
               (soa_index == 1) ? ozz::math::GetY(soa_transform.rotation.y) :
               (soa_index == 2) ? ozz::math::GetZ(soa_transform.rotation.y) :
                                  ozz::math::GetW(soa_transform.rotation.y);
    float qz = (soa_index == 0) ? ozz::math::GetX(soa_transform.rotation.z) :
               (soa_index == 1) ? ozz::math::GetY(soa_transform.rotation.z) :
               (soa_index == 2) ? ozz::math::GetZ(soa_transform.rotation.z) :
                                  ozz::math::GetW(soa_transform.rotation.z);
    float qw = (soa_index == 0) ? ozz::math::GetX(soa_transform.rotation.w) :
               (soa_index == 1) ? ozz::math::GetY(soa_transform.rotation.w) :
               (soa_index == 2) ? ozz::math::GetZ(soa_transform.rotation.w) :
                                  ozz::math::GetW(soa_transform.rotation.w);
    Fquaternion quat;
    quat.set(qx, qy, qz, qw);
    result.rotation(quat);

    // Extract scale
    float sx = (soa_index == 0) ? ozz::math::GetX(soa_transform.scale.x) :
               (soa_index == 1) ? ozz::math::GetY(soa_transform.scale.x) :
               (soa_index == 2) ? ozz::math::GetZ(soa_transform.scale.x) :
                                  ozz::math::GetW(soa_transform.scale.x);
    float sy = (soa_index == 0) ? ozz::math::GetX(soa_transform.scale.y) :
               (soa_index == 1) ? ozz::math::GetY(soa_transform.scale.y) :
               (soa_index == 2) ? ozz::math::GetZ(soa_transform.scale.y) :
                                  ozz::math::GetW(soa_transform.scale.y);
    float sz = (soa_index == 0) ? ozz::math::GetX(soa_transform.scale.z) :
               (soa_index == 1) ? ozz::math::GetY(soa_transform.scale.z) :
               (soa_index == 2) ? ozz::math::GetZ(soa_transform.scale.z) :
                                  ozz::math::GetW(soa_transform.scale.z);
    Fvector scale;
    scale.set(sx, sy, sz);
    result.i.mul(scale.x);
    result.j.mul(scale.y);
    result.k.mul(scale.z);

    return result;
}

Fmatrix OzzAnimationSystem::Float4x4ToMatrix(const ozz::math::Float4x4& ozz_matrix) const {
    Fmatrix result;

    // ozz matrices are column-major, X-Ray matrices are row-major
    // Rotation/scale is in the 3x3 upper-left portion
    result.i.set(ozz::math::GetX(ozz_matrix.cols[0]), ozz::math::GetX(ozz_matrix.cols[1]), ozz::math::GetX(ozz_matrix.cols[2]));
    result.j.set(ozz::math::GetY(ozz_matrix.cols[0]), ozz::math::GetY(ozz_matrix.cols[1]), ozz::math::GetY(ozz_matrix.cols[2]));
    result.k.set(ozz::math::GetZ(ozz_matrix.cols[0]), ozz::math::GetZ(ozz_matrix.cols[1]), ozz::math::GetZ(ozz_matrix.cols[2]));
    // Translation is in the 4th column
    result.c.set(ozz::math::GetX(ozz_matrix.cols[3]), ozz::math::GetY(ozz_matrix.cols[3]), ozz::math::GetZ(ozz_matrix.cols[3]));

    return result;
}

bool OzzAnimationSystem::ValidateAnimationHandle(const AnimationHandle* handle) const {
    if (!handle) {
        Msg("! OzzAnimationSystem: Invalid animation handle (null)");
        return false;
    }

    // Check if handle is in our active animations vector
    bool found = false;
    for (const auto& active_handle : active_animations_) {
        if (&active_handle == handle) {
            found = true;
            break;
        }
    }

    if (!found) {
        Msg("! OzzAnimationSystem: Animation handle not found in active animations");
        return false;
    }

    if (handle->animation_index >= animations_.size()) {
        Msg("! OzzAnimationSystem: Invalid animation index in handle");
        return false;
    }

    return true;
}

void OzzAnimationSystem::CleanupFinishedAnimations() {
    active_animations_.erase(
        std::remove_if(active_animations_.begin(), active_animations_.end(),
            [](const AnimationHandle& handle) {
                return !handle.is_playing && !handle.is_looping;
            }),
        active_animations_.end()
    );
}

} // namespace Animation
} // namespace XRay
