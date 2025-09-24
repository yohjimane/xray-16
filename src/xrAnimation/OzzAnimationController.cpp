#include "stdafx.h"

#include "OzzAnimationController.h"

#include "xrCore/xrDebug.h"

#include "ozz/base/memory/allocator.h"
#include "ozz/base/io/archive.h"
#include "ozz/base/io/stream.h"

#include <vector>

namespace XRay
{
namespace Animation
{
namespace
{
void SkipOzzString(ozz::io::IArchive& archive)
{
    uint32_t length = 0;
    archive >> length;
    if (length == 0)
        return;

    std::vector<char> buffer(length);
    archive >> ozz::io::MakeArray(buffer.data(), length);
}

void SkipOzzAnimationMetadata(ozz::io::IArchive& archive)
{
    SkipOzzString(archive);

    uint32_t flags = 0;
    uint16_t bone_or_part = 0;
    uint16_t motion_id = 0;
    float speed = 0.f;
    float power = 0.f;
    float accrue = 0.f;
    float falloff = 0.f;

    archive >> flags;
    archive >> bone_or_part;
    archive >> motion_id;
    archive >> speed;
    archive >> power;
    archive >> accrue;
    archive >> falloff;

    uint32_t mark_count = 0;
    archive >> mark_count;
    for (uint32_t mark_index = 0; mark_index < mark_count; ++mark_index)
    {
        SkipOzzString(archive);

        uint32_t interval_count = 0;
        archive >> interval_count;
        for (uint32_t interval_index = 0; interval_index < interval_count; ++interval_index)
        {
            float first = 0.f;
            float second = 0.f;
            archive >> first;
            archive >> second;
        }
    }
}
} // namespace

OzzAnimationController::OzzAnimationController()
    : skeleton_(nullptr), animation_loaded_(false), loop_(true), playback_speed_(1.f), time_(0.f)
{
}

OzzAnimationController::~OzzAnimationController() = default;

bool OzzAnimationController::Initialize(const ozz::animation::Skeleton& skeleton)
{
    skeleton_ = &skeleton;
    sampling_context_.Resize(0);
    animation_loaded_ = false;
    time_ = 0.f;
    ResetLocals();
    return true;
}

void OzzAnimationController::ResetLocals()
{
    if (!skeleton_)
        return;
    const size_t soa_count = static_cast<size_t>(skeleton_->num_soa_joints());
    locals_.resize(soa_count);
    for (size_t idx = 0; idx < soa_count; ++idx)
        locals_[idx] = ozz::math::SoaTransform::identity();
}

bool OzzAnimationController::LoadAnimation(const std::filesystem::path& path)
{
    if (!skeleton_)
    {
        Msg("[ozz] animation controller missing skeleton when loading %s", path.u8string().c_str());
        return false;
    }

    ozz::io::File file(path.u8string().c_str(), "rb");
    if (!file.opened())
    {
        Msg("[ozz] failed to open animation %s", path.u8string().c_str());
        return false;
    }

    ozz::io::IArchive archive(&file);
    std::shared_ptr<ozz::animation::Animation> animation = std::make_shared<ozz::animation::Animation>();
    if (archive.TestTag<ozz::animation::Animation>())
    {
        archive >> *animation;
    }
    else
    {
        file.Seek(0, ozz::io::Stream::kSet);

        ozz::io::IArchive aggregate(&file);

        uint32_t animation_count = 0;
        aggregate >> animation_count;
        if (animation_count == 0)
        {
            Msg("[ozz] file %s does not contain any animations", path.u8string().c_str());
            return false;
        }

        aggregate >> *animation;
        SkipOzzAnimationMetadata(aggregate);

        for (uint32_t idx = 1; idx < animation_count; ++idx)
        {
            ozz::animation::Animation discarded;
            aggregate >> discarded;
            SkipOzzAnimationMetadata(aggregate);
        }
    }

    return LoadAnimation(std::move(animation));
}

bool OzzAnimationController::LoadAnimation(std::shared_ptr<ozz::animation::Animation> animation)
{
    if (!skeleton_ || !animation)
        return false;

    if (animation->num_tracks() != skeleton_->num_joints())
    {
        Msg("[ozz] animation has %d tracks, expected %d for skeleton", animation->num_tracks(),
            skeleton_->num_joints());
        return false;
    }

    animation_ = std::move(animation);
    sampling_context_.Resize(animation_->num_tracks());
    ResetLocals();
    animation_loaded_ = true;
    time_ = 0.f;
    return true;
}

void OzzAnimationController::ClearAnimation()
{
    animation_.reset();
    animation_loaded_ = false;
    time_ = 0.f;
    sampling_context_.Resize(0);
    ResetLocals();
}

bool OzzAnimationController::Update(float dt)
{
    if (!animation_loaded_ || !skeleton_ || !animation_)
        return false;

    if (animation_->duration() <= 0.f)
        return false;

    time_ += dt * playback_speed_;

    if (loop_)
    {
        const float duration = animation_->duration();
        if (duration > 0.f)
        {
            while (time_ < 0.f)
                time_ += duration;
            while (time_ > duration)
                time_ -= duration;
        }
    }
    else
    {
        if (time_ < 0.f)
            time_ = 0.f;
        if (time_ > animation_->duration())
            time_ = animation_->duration();
    }

    const float duration = animation_->duration();
    float ratio = duration > 0.f ? time_ / duration : 0.f;
    if (!loop_)
    {
        if (ratio > 1.f)
            ratio = 1.f;
        if (ratio < 0.f)
            ratio = 0.f;
    }

    if (locals_.size() != static_cast<size_t>(skeleton_->num_soa_joints()))
        ResetLocals();

    ozz::animation::SamplingJob job;
    job.animation = animation_.get();
    job.context = &sampling_context_;
    job.ratio = ratio;
    job.output = ozz::span<ozz::math::SoaTransform>(locals_.data(), locals_.size());

    if (!job.Run())
    {
        Msg("[ozz] sampling job failed during animation update");
        return false;
    }

    return true;
}

ozz::span<const ozz::math::SoaTransform> OzzAnimationController::SampledLocals() const
{
    if (!animation_loaded_ || !animation_)
        return {};
    return ozz::span<const ozz::math::SoaTransform>(locals_.data(), locals_.size());
}
} // namespace Animation
} // namespace XRay
