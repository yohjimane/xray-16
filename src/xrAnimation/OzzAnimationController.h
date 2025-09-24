#pragma once

#include "ozz/animation/runtime/animation.h"
#include "ozz/animation/runtime/sampling_job.h"
#include "ozz/animation/runtime/skeleton.h"
#include "ozz/base/maths/soa_transform.h"
#include "ozz/base/span.h"

#include <filesystem>
#include <vector>

namespace XRay
{
namespace Animation
{
class OzzAnimationController
{
public:
    OzzAnimationController();
    ~OzzAnimationController();

    bool Initialize(const ozz::animation::Skeleton& skeleton);
    bool LoadAnimation(const std::filesystem::path& path);
    void ClearAnimation();

    void SetLooping(bool loop) { loop_ = loop; }
    void SetPlaybackSpeed(float speed) { playback_speed_ = speed; }

    bool HasAnimation() const { return animation_loaded_; }
    float Duration() const { return animation_loaded_ ? animation_.duration() : 0.f; }

    bool Update(float dt);

    ozz::span<const ozz::math::SoaTransform> SampledLocals() const;

private:
    void ResetLocals();

private:
    const ozz::animation::Skeleton* skeleton_;
    ozz::animation::Animation animation_;
    ozz::animation::SamplingJob::Context sampling_context_;
    std::vector<ozz::math::SoaTransform> locals_;
    bool animation_loaded_;
    bool loop_;
    float playback_speed_;
    float time_;
};
} // namespace Animation
} // namespace XRay
