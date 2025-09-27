#include "stdafx.h"

#include "OzzKinematics.h"

#include "OzzConversion.h"

#include "xrEngine/device.h"
#include "xrCore/FS.h"
#include "xrCore/FS_impl.h"

#include "ozz/animation/runtime/skeleton_utils.h"
#include "ozz/base/io/archive.h"
#include "ozz/base/io/stream.h"
#include "ozz/base/maths/simd_math.h"
#include "ozz/base/span.h"

#include <algorithm>
#include <cstring>
#include <vector>

namespace XRay
{
namespace Animation
{
namespace
{
struct BoneInstanceStub
{
    CBoneInstance instance;

    BoneInstanceStub()
    {
        instance.construct();
    }
};

CBoneInstance& StubBoneInstance()
{
    static BoneInstanceStub stub;
    return stub.instance;
}

CBoneData& StubBoneData()
{
    static CBoneData stub{ 0 };
    return stub;
}

Fmatrix& StubMatrix()
{
    static Fmatrix stub = Fidentity;
    return stub;
}

Fobb& StubObb()
{
    static Fobb stub;
    return stub;
}
} // namespace

OzzKinematics::OzzKinematics()
    : user_data_(nullptr), root_bone_(BI_NONE), visible_mask_(0), update_callback_(nullptr), update_callback_param_(nullptr), last_update_time_(0),
      visibility_counter_(0)
{
    cached_box_.invalidate();
}

OzzKinematics::~OzzKinematics() = default;

bool OzzKinematics::InitializeFromOzz(pcstr skeleton_path)
{
    if (!skeleton_path || !skeleton_path[0])
    {
        Msg("[OzzKinematics] InitializeFromOzz received empty skeleton path");
        return false;
    }

    ResetRuntimeState();

    ozz::io::File file(skeleton_path, "rb");
    if (!file.opened())
    {
        Msg("[OzzKinematics] Failed to open skeleton file: %s", skeleton_path);
        return false;
    }

    return LoadSkeletonFromStream(&file, skeleton_path);
}

bool OzzKinematics::InitializeFromOzzBuffer(ozz::span<const std::byte> skeleton_data)
{
    ResetRuntimeState();

    if (skeleton_data.empty())
    {
        Msg("[OzzKinematics] InitializeFromOzzBuffer received empty skeleton buffer");
        return false;
    }

    ozz::io::MemoryStream memory_stream;
    if (!memory_stream.Write(skeleton_data.data(), skeleton_data.size()))
    {
        Msg("[OzzKinematics] Failed to copy skeleton buffer into memory stream");
        ResetRuntimeState();
        return false;
    }

    memory_stream.Seek(0, ozz::io::Stream::kSet);

    if (!LoadSkeletonFromStream(&memory_stream, "memory buffer"))
    {
        ResetRuntimeState();
        return false;
    }

    return true;
}

void OzzKinematics::NotImplemented(pcstr function_name) const
{
    Msg("[OzzKinematics] %s is not implemented yet", function_name);
}

void OzzKinematics::ResetRuntimeState()
{
    bone_instances_.clear();
    bone_boxes_.clear();
    bones_.clear();
    bone_storage_.clear();
    bone_map_by_name_.clear();
    bone_map_by_ptr_.clear();
    cached_transforms_pre_callbacks_.clear();
    bone_offsets_.clear();
    sampled_locals_.clear();
    model_transforms_.clear();
    sampling_context_.Resize(0);
    user_data_ = nullptr;
    root_bone_ = BI_NONE;
    visible_mask_ = 0;
    cached_box_.invalidate();
    last_update_time_ = 0;
    visibility_counter_ = 0;
    skeleton_ = ozz::animation::Skeleton();

    ResetAnimationState();
    initialized_ = false;
}

void OzzKinematics::ResetAnimationState()
{
    animation_controller_.reset();
    motion_references_.clear();
    legacy_motion_metadata_.clear();
    legacy_motion_library_.clear();
    converted_motion_sources_.clear();
    blend_destroy_callback_ = nullptr;
    update_tracks_callback_ = nullptr;
    animation_applied_ = false;
}

bool OzzKinematics::EnsureAnimationController()
{
    if (!initialized_)
        return false;

    if (!animation_controller_)
    {
        animation_controller_ = xr_make_unique<OzzAnimationController>();
        if (!animation_controller_->Initialize(skeleton_))
        {
            Msg("[OzzKinematics] Failed to initialize animation controller for skeleton");
            animation_controller_.reset();
            return false;
        }
    }

    return true;
}

void OzzKinematics::SetLegacyMotionReferences(const xr_vector<xr_string>& references)
{
    motion_references_ = references;
    legacy_motion_metadata_.clear();
    legacy_motion_library_.clear();
    converted_motion_sources_.clear();
}

xr_vector<xr_string> OzzKinematics::LegacyMotionNames()
{
    if (legacy_motion_metadata_.empty() && !motion_references_.empty())
    {
        if (legacy_motion_library_.empty())
            BuildLegacyMotionLibrary();
    }

    xr_vector<xr_string> names;
    names.reserve(legacy_motion_metadata_.size());
    for (const auto& entry : legacy_motion_metadata_)
        names.push_back(entry.first);

    std::sort(names.begin(), names.end(),
        [](const xr_string& lhs, const xr_string& rhs)
        {
            return xr_stricmp(lhs.c_str(), rhs.c_str()) < 0;
        });
    return names;
}

bool OzzKinematics::BuildLegacyMotionLibrary()
{
    if (motion_references_.empty())
        return false;

    const auto bone_names = CollectSkeletonBoneNames();
    if (bone_names.empty())
        return false;

    bool converted_any = false;
    for (const auto& reference : motion_references_)
    {
        const auto candidates = ResolveMotionReference(reference);
        for (const auto& candidate : candidates)
            converted_any |= ConvertLegacyOmfFile(candidate, bone_names);
    }

    return converted_any;
}

bool OzzKinematics::LoadLegacyMotion(const xr_string& motion_name)
{
    if (!EnsureAnimationController())
        return false;

    if (legacy_motion_library_.empty() && !motion_references_.empty())
        BuildLegacyMotionLibrary();

    const auto it = legacy_motion_library_.find(motion_name);
    if (it == legacy_motion_library_.end() || !it->second)
    {
        Msg("[OzzKinematics] Legacy motion '%s' not available", motion_name.c_str());
        return false;
    }

    if (!animation_controller_->LoadAnimation(it->second))
    {
        Msg("[OzzKinematics] Failed to bind legacy motion '%s'", motion_name.c_str());
        return false;
    }

    animation_applied_ = false;
    CalculateBones_Invalidate();
    return true;
}

bool OzzKinematics::PlayLegacyMotion(const xr_string& motion_name)
{
    return LoadLegacyMotion(motion_name);
}

bool OzzKinematics::LoadAnimationFromFile(const std::filesystem::path& path)
{
    if (!EnsureAnimationController())
        return false;

    if (!animation_controller_->LoadAnimation(path))
        return false;

    animation_applied_ = false;
    CalculateBones_Invalidate();
    return true;
}

void OzzKinematics::StopAnimation()
{
    if (!animation_controller_)
        return;

    animation_controller_->ClearAnimation();
    animation_applied_ = false;
    ClearPose();
    CalculateBones_Invalidate();
}

bool OzzKinematics::AdvanceAnimation(float dt)
{
    if (!EnsureAnimationController())
        return false;

    if (!animation_controller_->HasAnimation())
    {
        if (animation_applied_)
        {
            ClearPose();
            animation_applied_ = false;
            return true;
        }
        return false;
    }

    if (!animation_controller_->Update(dt))
        return false;

    const auto locals = animation_controller_->SampledLocals();
    if (locals.empty())
        return false;

    if (!SetPoseLocals(locals))
        return false;

    animation_applied_ = true;
    return true;
}

bool OzzKinematics::HasLoadedAnimation() const
{
    return animation_controller_ && animation_controller_->HasAnimation();
}

xr_vector<xr_string> OzzKinematics::CollectSkeletonBoneNames() const
{
    xr_vector<xr_string> names;
    if (!initialized_)
        return names;

    const int bone_count = skeleton_.num_joints();
    if (bone_count <= 0)
        return names;

    names.reserve(static_cast<size_t>(bone_count));
    const auto joint_names = skeleton_.joint_names();
    for (int idx = 0; idx < bone_count; ++idx)
    {
        const char* name = (static_cast<size_t>(idx) < joint_names.size() && joint_names[idx]) ? joint_names[idx] : "";
        names.emplace_back(name ? name : "");
    }
    return names;
}

xr_vector<xr_string> OzzKinematics::ResolveMotionReference(const xr_string& reference) const
{
    xr_vector<xr_string> results;
    if (reference.empty())
        return results;

    if (reference.find('*') != xr_string::npos)
    {
        FS_FileSet files;
        FS.file_list(files, "$level$", FS_ListFiles, reference.c_str());
        FS.file_list(files, "$game_meshes$", FS_ListFiles, reference.c_str());

        xr_set<xr_string> unique;
        for (const auto& entry : files)
            unique.emplace(entry.name.c_str());

        results.reserve(unique.size());
        for (const auto& name : unique)
            results.emplace_back(name);
        return results;
    }

    xr_string candidate = reference;
    constexpr size_t ext_length = 4;
    if (candidate.size() < ext_length ||
        xr_stricmp(candidate.c_str() + candidate.size() - ext_length, ".omf") != 0)
    {
        candidate += ".omf";
    }

    results.emplace_back(std::move(candidate));
    return results;
}

bool OzzKinematics::ConvertLegacyOmfFile(const xr_string& relative_path, const xr_vector<xr_string>& skeleton_bone_names)
{
    if (converted_motion_sources_.find(relative_path) != converted_motion_sources_.end())
        return true;

    string_path resolved;
    const char* search_roots[] = { "$level$", "$game_meshes$" };
    const char* located_root = nullptr;
    FileStatus status(false, false);

    for (const char* root : search_roots)
    {
        status = FS.exist(resolved, root, relative_path.c_str());
        if (status)
        {
            located_root = root;
            break;
        }
    }

    if (!located_root)
    {
        Msg("[ozz] legacy animation source '%s' not found", relative_path.c_str());
        return false;
    }

    xr_vector<ConvertedOmfAnimation> converted;
    bool converted_ok = false;

    if (status.External)
    {
        converted_ok = ConvertLegacyOmf(std::filesystem::path(resolved), skeleton_bone_names, skeleton_, converted);
    }
    else
    {
        IReader* reader = FS.r_open(located_root, relative_path.c_str());
        if (!reader)
        {
            Msg("[ozz] legacy animation source '%s' could not be opened", relative_path.c_str());
            return false;
        }

        const size_t payload_size = static_cast<size_t>(reader->length());
        xr_vector<std::byte> payload(payload_size);
        if (payload_size > 0)
            std::memcpy(payload.data(), reader->pointer(), payload_size);
        FS.r_close(reader);

        converted_ok = ConvertLegacyOmf(payload.data(), payload.size(), skeleton_bone_names, skeleton_, converted);
    }

    if (!converted_ok)
    {
        Msg("[ozz] failed to convert legacy animation '%s'", relative_path.c_str());
        return false;
    }

    for (auto& entry : converted)
    {
        if (!entry.animation)
            continue;

        const xr_string motion_name = entry.name;
        if (legacy_motion_library_.find(motion_name) != legacy_motion_library_.end())
        {
            Msg("[ozz] duplicate legacy motion '%s' encountered in '%s'", motion_name.c_str(), relative_path.c_str());
            continue;
        }

        legacy_motion_metadata_[motion_name] = entry.metadata;
        auto deleter = entry.animation.get_deleter();
        legacy_motion_library_[motion_name] =
            std::shared_ptr<ozz::animation::Animation>(entry.animation.release(), deleter);
    }

    converted_motion_sources_.insert(relative_path);
    return true;
}

bool OzzKinematics::LoadSkeletonFromStream(ozz::io::Stream* stream, pcstr debug_source)
{
    if (!stream)
    {
        Msg("[OzzKinematics] LoadSkeletonFromStream received null stream for %s", debug_source ? debug_source : "<unknown>");
        return false;
    }

    ozz::io::IArchive archive(stream);
    if (!archive.TestTag<ozz::animation::Skeleton>())
    {
        Msg("[OzzKinematics] Stream does not contain a skeleton: %s", debug_source ? debug_source : "<unknown>");
        return false;
    }

    archive >> skeleton_;

    if (!FinalizeSkeletonInitialization(debug_source))
    {
        ResetRuntimeState();
        return false;
    }

    return true;
}

bool OzzKinematics::FinalizeSkeletonInitialization(pcstr debug_source)
{
    const int joint_count = skeleton_.num_joints();
    if (joint_count <= 0)
    {
        Msg("[OzzKinematics] Skeleton contains no joints: %s", debug_source ? debug_source : "<unknown>");
        return false;
    }

    bone_instances_.resize(joint_count);
    for (CBoneInstance& instance : bone_instances_)
        instance.construct();

    bone_boxes_.resize(joint_count);
    for (Fobb& box : bone_boxes_)
        box.invalidate();

    bones_.assign(joint_count, nullptr);
    bone_storage_.clear();
    bone_storage_.reserve(joint_count);

    model_transforms_.clear();
    model_transforms_.shrink_to_fit();
    model_transforms_.resize(static_cast<size_t>(joint_count));

    cached_transforms_pre_callbacks_.clear();
    cached_transforms_pre_callbacks_.resize(static_cast<size_t>(joint_count));

    bone_map_by_name_.clear();
    bone_map_by_ptr_.clear();
    const auto joint_names = skeleton_.joint_names();
    bone_map_by_name_.reserve(static_cast<size_t>(joint_count));
    bone_map_by_ptr_.reserve(static_cast<size_t>(joint_count));
    for (int joint = 0; joint < joint_count; ++joint)
    {
        const size_t joint_index = static_cast<size_t>(joint);
        const char* joint_name = (joint_index < joint_names.size() && joint_names[joint_index]) ? joint_names[joint_index] : "";
        shared_str shared_name(joint_name ? joint_name : "");
        const u16 bone_id = static_cast<u16>(joint);
        bone_map_by_name_.emplace_back(shared_name, bone_id);
        bone_map_by_ptr_.emplace_back(shared_name, bone_id);
    }

    std::sort(bone_map_by_name_.begin(), bone_map_by_name_.end(),
        [](const auto& lhs, const auto& rhs)
        {
            return xr_strcmp(lhs.first.c_str(), rhs.first.c_str()) < 0;
        });

    std::sort(bone_map_by_ptr_.begin(), bone_map_by_ptr_.end(),
        [](const auto& lhs, const auto& rhs)
        {
            return lhs.first._get() < rhs.first._get();
        });

    root_bone_ = joint_count > 0 ? 0 : BI_NONE;

    if (joint_count >= 64)
        visible_mask_ = u64(-1);
    else if (joint_count == 0)
        visible_mask_ = 0;
    else
        visible_mask_ = (u64(1) << joint_count) - 1;

    if (!BuildBoneMetadata())
        return false;

    bone_offsets_.clear();
    sampled_locals_.clear();
    sampling_context_.Resize(0);

    cached_box_.invalidate();
    last_update_time_ = 0;
    visibility_counter_ = 0;

    initialized_ = true;

    animation_controller_.reset();
    animation_applied_ = false;
    EnsureAnimationController();

    return true;
}

using XRay::Animation::ConvertOzzMatrixToXRay;
bool OzzKinematics::BuildBoneMetadata()
{
    const int joint_count = skeleton_.num_joints();
    if (joint_count <= 0)
        return false;

    const ozz::span<const int16_t> parents = skeleton_.joint_parents();
    const auto joint_names = skeleton_.joint_names();

    bone_storage_.clear();
    bone_storage_.reserve(static_cast<size_t>(joint_count));

    for (int joint = 0; joint < joint_count; ++joint)
    {
        const u16 bone_id = static_cast<u16>(joint);
        auto bone = xr_make_unique<CBoneData>(bone_id);

    const int16_t parent_index = (static_cast<size_t>(joint) < parents.size()) ? parents[joint] : static_cast<int16_t>(-1);
        bone->SetParentID(parent_index >= 0 ? static_cast<u16>(parent_index) : BI_NONE);

    const char* joint_name = (static_cast<size_t>(joint) < joint_names.size()) ? joint_names[joint] : nullptr;
        bone->name = joint_name && joint_name[0] ? shared_str(joint_name) : shared_str();

        bone->shape.Reset();
        bone->obb.invalidate();
        bone->game_mtl_name = shared_str();
        bone->game_mtl_idx = 0;
        bone->mass = 0.f;
        bone->center_of_mass.set(0.f, 0.f, 0.f);
        bone->IK_data.Reset();

        const ozz::math::Transform rest_pose = ozz::animation::GetJointLocalRestPose(skeleton_, joint);
        const ozz::math::Float4x4 rest_matrix = ozz::math::Float4x4::FromAffine(rest_pose);
        bone->bind_transform = ConvertOzzMatrixToXRay(rest_matrix);
        bone->m2b_transform.identity();

        bone_storage_.push_back(std::move(bone));
    }

    for (size_t idx = 0; idx < bone_storage_.size(); ++idx)
        bones_[idx] = bone_storage_[idx].get();

    // Build children relationships.
    for (int joint = 0; joint < joint_count; ++joint)
    {
        const int16_t parent_index = (static_cast<size_t>(joint) < parents.size()) ? parents[joint] : static_cast<int16_t>(-1);
        if (parent_index >= 0 && parent_index < joint_count)
            bones_[parent_index]->children.push_back(bones_[joint]);
    }

    if (!bones_.empty() && root_bone_ != BI_NONE && root_bone_ < bones_.size())
        bones_[root_bone_]->CalculateM2B(Fidentity);

    return true;
}

bool OzzKinematics::IsBoneVisible(size_t index) const
{
    if (index >= bone_instances_.size())
        return false;
    if (index >= 64)
        return true;
    const u64 mask = u64(1) << index;
    return (visible_mask_ & mask) != 0;
}

void OzzKinematics::ApplyAdditionalBoneTransforms(u16 bone_id, Fmatrix& transform) const
{
    for (const auto& offset : bone_offsets_)
    {
        if (offset.m_bone_id != bone_id)
            continue;

        const Fvector original_position = transform.c;
        transform.mulB_43(offset.m_transform);
        transform.c.add(original_position, offset.m_transform.c);
    }
}

bool OzzKinematics::SetPoseLocals(ozz::span<const ozz::math::SoaTransform> locals)
{
    if (!initialized_)
        return false;

    if (locals.empty())
    {
        sampled_locals_.clear();
        CalculateBones_Invalidate();
        return true;
    }

    if (static_cast<int>(locals.size()) != skeleton_.num_soa_joints())
    {
        Msg("[OzzKinematics] SetPoseLocals received %zu soa joints, expected %d", locals.size(), skeleton_.num_soa_joints());
        return false;
    }

    sampled_locals_.assign(locals.begin(), locals.end());
    CalculateBones_Invalidate();
    return true;
}

void OzzKinematics::ClearPose()
{
    if (!initialized_)
        return;

    sampled_locals_.clear();
    CalculateBones_Invalidate();
}

const ozz::animation::Skeleton& OzzKinematics::Skeleton() const
{
    return skeleton_;
}

ozz::animation::SamplingJob::Context& OzzKinematics::SamplingContext()
{
    return sampling_context_;
}

void OzzKinematics::BuildSkinningPalette(xr_vector<Fmatrix>& out_matrices, bool render_space) const
{
    if (!initialized_)
    {
        out_matrices.clear();
        return;
    }

    const size_t expected = static_cast<size_t>(skeleton_.num_joints());
    if (expected == 0)
    {
        out_matrices.clear();
        return;
    }

    const size_t instance_count = bone_instances_.size();
    const size_t model_count = model_transforms_.size();
    const size_t available = std::min(instance_count, std::min(model_count, expected));

    out_matrices.resize(expected);

    if (available != expected)
    {
        Msg("[OzzKinematics] BuildSkinningPalette requested before bone buffers ready (instances=%zu, model=%zu, expected=%zu)",
            instance_count, model_count, expected);

        for (size_t idx = 0; idx < expected; ++idx)
            out_matrices[idx] = Fidentity;
        return;
    }

    if (render_space)
    {
        for (size_t idx = 0; idx < expected; ++idx)
            out_matrices[idx] = bone_instances_[idx].mRenderTransform;
    }
    else
    {
        for (size_t idx = 0; idx < expected; ++idx)
            out_matrices[idx] = bone_instances_[idx].mTransform;
    }
}

void OzzKinematics::Bone_Calculate(CBoneData* /*bd*/, Fmatrix* /*parent*/)
{
    CalculateBones(TRUE);
}

void OzzKinematics::OnCalculateBones()
{
    CalculateBones(TRUE);
}

void OzzKinematics::Bone_GetAnimPos(Fmatrix& pos, u16 id, u8 /*channel_mask*/, bool ignore_callbacks)
{
    if (id >= bone_instances_.size())
    {
        pos.identity();
        return;
    }

    CalculateBones(TRUE);

    if (ignore_callbacks && id < cached_transforms_pre_callbacks_.size())
    {
        pos = cached_transforms_pre_callbacks_[id];
    }
    else
    {
        pos = LL_GetTransform(id);
    }
}

bool OzzKinematics::PickBone(const Fmatrix& /*parent_xform*/, pick_result& /*r*/, float /*dist*/, const Fvector& /*start*/, const Fvector& /*dir*/, u16 /*bone_id*/)
{
    return false;
}

void OzzKinematics::EnumBoneVertices(SEnumVerticesCallback& /*C*/, u16 /*bone_id*/)
{
}

u16 OzzKinematics::LL_BoneID(LPCSTR B)
{
    if (!B)
        return BI_NONE;

    const auto it = std::lower_bound(bone_map_by_name_.begin(), bone_map_by_name_.end(), B,
        [](const accel::value_type& entry, LPCSTR value)
        {
            return xr_strcmp(entry.first.c_str(), value) < 0;
        });

    if (it == bone_map_by_name_.end())
        return BI_NONE;

    return xr_strcmp(it->first.c_str(), B) == 0 ? it->second : BI_NONE;
}

u16 OzzKinematics::LL_BoneID(const shared_str& B)
{
    if (!B._get())
        return BI_NONE;

    const auto it = std::lower_bound(bone_map_by_ptr_.begin(), bone_map_by_ptr_.end(), B,
        [](const accel::value_type& entry, const shared_str& value)
        {
            return entry.first._get() < value._get();
        });

    if (it == bone_map_by_ptr_.end())
        return BI_NONE;

    return it->first._get() == B._get() ? it->second : BI_NONE;
}

LPCSTR OzzKinematics::LL_BoneName_dbg(u16 ID)
{
    if (ID >= bone_instances_.size())
        return nullptr;

    const auto it = std::find_if(bone_map_by_name_.begin(), bone_map_by_name_.end(),
        [ID](const accel::value_type& entry)
        {
            return entry.second == ID;
        });

    return it != bone_map_by_name_.end() ? it->first.c_str() : nullptr;
}

CInifile* OzzKinematics::LL_UserData()
{
    return user_data_;
}

IKinematics::accel* OzzKinematics::LL_Bones()
{
    return &bone_map_by_name_;
}

CBoneInstance& OzzKinematics::LL_GetBoneInstance(u16 bone_id)
{
    if (bone_id < bone_instances_.size())
        return bone_instances_[bone_id];

    NotImplemented(__FUNCTION__);
    return StubBoneInstance();
}

CBoneData& OzzKinematics::LL_GetData(u16 bone_id)
{
    if (bone_id < bones_.size() && bones_[bone_id])
        return *bones_[bone_id];
    return StubBoneData();
}

const IBoneData& OzzKinematics::GetBoneData(u16 bone_id) const
{
    if (bone_id < bones_.size() && bones_[bone_id])
        return *bones_[bone_id];
    return StubBoneData();
}

u16 OzzKinematics::LL_BoneCount() const
{
    return static_cast<u16>(bone_instances_.size());
}

u16 OzzKinematics::LL_VisibleBoneCount()
{
    u16 count = 0;
    const u16 total = LL_BoneCount();
    for (u16 i = 0; i < total; ++i)
        if (LL_GetBoneVisible(i))
            ++count;
    return count;
}

Fmatrix& OzzKinematics::LL_GetTransform(u16 bone_id)
{
    if (bone_id < bone_instances_.size())
        return bone_instances_[bone_id].mTransform;

    NotImplemented(__FUNCTION__);
    return StubMatrix();
}

const Fmatrix& OzzKinematics::LL_GetTransform(u16 bone_id) const
{
    if (bone_id < bone_instances_.size())
        return bone_instances_[bone_id].mTransform;

    const_cast<OzzKinematics*>(this)->NotImplemented(__FUNCTION__);
    return StubMatrix();
}

Fmatrix& OzzKinematics::LL_GetTransform_R(u16 bone_id)
{
    if (bone_id < bone_instances_.size())
        return bone_instances_[bone_id].mRenderTransform;

    NotImplemented(__FUNCTION__);
    return StubMatrix();
}

Fobb& OzzKinematics::LL_GetBox(u16 bone_id)
{
    if (bone_id < bone_boxes_.size())
        return bone_boxes_[bone_id];

    NotImplemented(__FUNCTION__);
    return StubObb();
}

const Fbox& OzzKinematics::GetBox() const
{
    return cached_box_;
}

void OzzKinematics::LL_GetBindTransform(xr_vector<Fmatrix>& matrices)
{
    matrices.clear();
    matrices.reserve(bones_.size());
    for (CBoneData* bone : bones_)
    {
        if (bone)
            matrices.push_back(bone->bind_transform);
    }
}

int OzzKinematics::LL_GetBoneGroups(xr_vector<xr_vector<u16>>& groups)
{
    groups.clear();
    return 0;
}

u16 OzzKinematics::LL_GetBoneRoot()
{
    return root_bone_;
}

void OzzKinematics::LL_SetBoneRoot(u16 bone_id)
{
    root_bone_ = bone_id;
}

BOOL OzzKinematics::LL_GetBoneVisible(u16 bone_id)
{
    if (bone_id >= 64)
        return TRUE;
    const u64 mask = u64(1) << bone_id;
    return (visible_mask_ & mask) ? TRUE : FALSE;
}

void OzzKinematics::LL_SetBoneVisible(u16 bone_id, BOOL val, BOOL /*bRecursive*/)
{
    if (bone_id < 64)
    {
        const u64 mask = u64(1) << bone_id;
        if (val)
            visible_mask_ |= mask;
        else
            visible_mask_ &= ~mask;
    }

    if (bone_id < bone_instances_.size())
    {
        if (val)
        {
            CalculateBones_Invalidate();
        }
        else
        {
            bone_instances_[bone_id].mTransform.scale(0.f, 0.f, 0.f);
            bone_instances_[bone_id].mRenderTransform.scale(0.f, 0.f, 0.f);
            if (bone_id < cached_transforms_pre_callbacks_.size())
                cached_transforms_pre_callbacks_[bone_id].scale(0.f, 0.f, 0.f);
        }
    }
}

u64 OzzKinematics::LL_GetBonesVisible()
{
    return visible_mask_;
}

void OzzKinematics::LL_SetBonesVisible(u64 mask)
{
    visible_mask_ = mask;

    const size_t count = bone_instances_.size();
    for (size_t idx = 0; idx < count && idx < 64; ++idx)
    {
        const u64 bit = u64(1) << idx;
        if ((visible_mask_ & bit) == 0)
        {
            bone_instances_[idx].mTransform.scale(0.f, 0.f, 0.f);
            bone_instances_[idx].mRenderTransform.scale(0.f, 0.f, 0.f);
            if (idx < cached_transforms_pre_callbacks_.size())
                cached_transforms_pre_callbacks_[idx].scale(0.f, 0.f, 0.f);
        }
    }
    CalculateBones_Invalidate();
}

void OzzKinematics::LL_AddTransformToBone(KinematicsABT::additional_bone_transform& offset)
{
    bone_offsets_.push_back(offset);
    CalculateBones_Invalidate();
}

void OzzKinematics::LL_ClearAdditionalTransform(u16 bone_id)
{
    if (bone_id == BI_NONE)
    {
        bone_offsets_.clear();
    }
    else
    {
        bone_offsets_.erase(
            std::remove_if(bone_offsets_.begin(), bone_offsets_.end(),
                [bone_id](const KinematicsABT::additional_bone_transform& entry)
                {
                    return entry.m_bone_id == bone_id;
                }),
            bone_offsets_.end());
    }

    CalculateBones_Invalidate();
}

void OzzKinematics::CalculateBones(BOOL bForceExact)
{
    if (!initialized_)
        return;

    if (skeleton_.num_joints() == 0 || bone_instances_.empty())
        return;

    const u32 current_time = Device.dwTimeGlobal;
    if (!bForceExact && current_time == last_update_time_)
        return;
    if (!bForceExact && last_update_time_ != 0 && current_time < last_update_time_ + UCalc_Interval)
        return;

    const size_t expected_joint_count = static_cast<size_t>(skeleton_.num_joints());
    if (bone_instances_.size() != expected_joint_count)
    {
        Msg("[OzzKinematics] Bone instance array size mismatch (have %zu, expected %zu). Reinitializing.",
            bone_instances_.size(), expected_joint_count);
        bone_instances_.resize(expected_joint_count);
        for (CBoneInstance& instance : bone_instances_)
            instance.construct();
    }

    if (model_transforms_.size() != expected_joint_count)
    {
        model_transforms_.resize(expected_joint_count);
        std::fill(model_transforms_.begin(), model_transforms_.end(), ozz::math::Float4x4::identity());
    }
    if (cached_transforms_pre_callbacks_.size() != expected_joint_count)
    {
        cached_transforms_pre_callbacks_.resize(expected_joint_count);
        std::fill(cached_transforms_pre_callbacks_.begin(), cached_transforms_pre_callbacks_.end(), Fidentity);
    }

    ozz::animation::LocalToModelJob job;
    job.skeleton = &skeleton_;
    if (!sampled_locals_.empty() && sampled_locals_.size() == static_cast<size_t>(skeleton_.num_soa_joints()))
        job.input = ozz::span<const ozz::math::SoaTransform>(sampled_locals_.data(), sampled_locals_.size());
    else
        job.input = skeleton_.joint_rest_poses();

    job.output = ozz::span<ozz::math::Float4x4>(model_transforms_.data(), model_transforms_.size());

    if (!job.Run())
    {
        Msg("[OzzKinematics] LocalToModelJob failed during CalculateBones");
        return;
    }

    const size_t bone_count = bone_instances_.size();
    Fbox box;
    box.invalidate();

    for (size_t i = 0; i < bone_count; ++i)
    {
        CBoneInstance& instance = bone_instances_[i];
        const u16 bone_id = static_cast<u16>(i);
        const bool visible = IsBoneVisible(i);

        Fmatrix transform = ConvertOzzMatrixToXRay(model_transforms_[i]);
        ApplyAdditionalBoneTransforms(bone_id, transform);

        if (!visible)
        {
            instance.mTransform.scale(0.f, 0.f, 0.f);
            instance.mRenderTransform.scale(0.f, 0.f, 0.f);
            if (i < cached_transforms_pre_callbacks_.size())
                cached_transforms_pre_callbacks_[i].scale(0.f, 0.f, 0.f);
            continue;
        }

        cached_transforms_pre_callbacks_[i] = transform;

        const bool has_callback = instance.callback() != nullptr;
        if (!instance.callback_overwrite() || !has_callback)
            instance.mTransform = transform;

        if (has_callback)
            instance.callback()(&instance);
        else if (instance.callback_overwrite())
            instance.mTransform = transform;

        if (i < bones_.size() && bones_[i])
            instance.mRenderTransform.mul_43(instance.mTransform, bones_[i]->m2b_transform);
        else
            instance.mRenderTransform = instance.mTransform;

        box.modify(instance.mTransform.c);
    }

    cached_box_ = box;
    last_update_time_ = current_time;

    if (update_callback_)
        update_callback_(this);
}

void OzzKinematics::CalculateBones_Invalidate()
{
    last_update_time_ = 0;
    visibility_counter_ = 0;
    cached_box_.invalidate();
}

void OzzKinematics::Callback(UpdateCallback C, void* Param)
{
    update_callback_ = C;
    update_callback_param_ = Param;
}

void OzzKinematics::SetUpdateCallback(UpdateCallback pCallback)
{
    update_callback_ = pCallback;
}

void OzzKinematics::SetUpdateCallbackParam(void* pCallbackParam)
{
    update_callback_param_ = pCallbackParam;
}

UpdateCallback OzzKinematics::GetUpdateCallback()
{
    return update_callback_;
}

void* OzzKinematics::GetUpdateCallbackParam()
{
    return update_callback_param_;
}

u32 OzzKinematics::LL_PartBlendsCount(u32 /*bone_part_id*/)
{
    return 0;
}

CBlend* OzzKinematics::LL_PartBlend(u32 /*bone_part_id*/, u32 /*n*/)
{
    return nullptr;
}

void OzzKinematics::LL_IterateBlends(IterateBlendsCallback& /*callback*/)
{
}

u16 OzzKinematics::LL_MotionsSlotCount()
{
    return 0;
}

const shared_motions& OzzKinematics::LL_MotionsSlot(u16 /*idx*/)
{
    static shared_motions dummy;
    return dummy;
}

CMotionDef* OzzKinematics::LL_GetMotionDef(MotionID /*id*/)
{
    return nullptr;
}

CMotion* OzzKinematics::LL_GetRootMotion(MotionID /*id*/)
{
    return nullptr;
}

CMotion* OzzKinematics::LL_GetMotion(MotionID /*id*/, u16 /*bone_id*/)
{
    return nullptr;
}

void OzzKinematics::LL_BuldBoneMatrixDequatize(const CBoneData* /*bd*/, u8 /*channel_mask*/, SKeyTable& /*keys*/)
{
    NotImplemented(__FUNCTION__);
}

void OzzKinematics::LL_BoneMatrixBuild(CBoneInstance& /*bi*/, const Fmatrix* /*parent*/, const SKeyTable& /*keys*/)
{
    NotImplemented(__FUNCTION__);
}

IBlendDestroyCallback* OzzKinematics::GetBlendDestroyCallback()
{
    return blend_destroy_callback_;
}

void OzzKinematics::SetBlendDestroyCallback(IBlendDestroyCallback* cb)
{
    blend_destroy_callback_ = cb;
}

void OzzKinematics::SetUpdateTracksCalback(IUpdateTracksCallback* callback)
{
    update_tracks_callback_ = callback;
}

IUpdateTracksCallback* OzzKinematics::GetUpdateTracksCalback()
{
    return update_tracks_callback_;
}

MotionID OzzKinematics::LL_MotionID(LPCSTR /*B*/)
{
    return MotionID();
}

u16 OzzKinematics::LL_PartID(LPCSTR /*B*/)
{
    return BI_NONE;
}

CBlend* OzzKinematics::LL_PlayCycle(u16 /*partition*/, MotionID /*motion*/, BOOL /*bMixing*/, float /*blendAccrue*/,
    float /*blendFalloff*/, float /*Speed*/, BOOL /*noloop*/, PlayCallback /*Callback*/, LPVOID /*CallbackParam*/, u8 /*channel*/)
{
    return nullptr;
}

CBlend* OzzKinematics::LL_PlayCycle(
    u16 /*partition*/, MotionID /*motion*/, BOOL /*bMixIn*/, PlayCallback /*Callback*/, LPVOID /*CallbackParam*/, u8 /*channel*/)
{
    return nullptr;
}

void OzzKinematics::LL_CloseCycle(u16 /*partition*/, u8 /*mask_channel*/)
{
}

void OzzKinematics::LL_SetChannelFactor(u16 /*channel*/, float /*factor*/)
{
}

void OzzKinematics::UpdateTracks()
{
    LL_UpdateTracks(Device.fTimeDelta, true, false);
}

void OzzKinematics::LL_UpdateTracks(float dt, bool /*b_force*/, bool /*leave_blends*/)
{
    const bool advanced = AdvanceAnimation(dt);
    if (update_tracks_callback_)
        (*update_tracks_callback_)(dt, *this);

    if (advanced && blend_destroy_callback_)
    {
        // Ozz runtime currently does not maintain blend instances, so nothing to flush here.
    }
}

MotionID OzzKinematics::ID_Cycle(LPCSTR /*N*/)
{
    return MotionID();
}

MotionID OzzKinematics::ID_Cycle_Safe(LPCSTR /*N*/)
{
    return MotionID();
}

MotionID OzzKinematics::ID_Cycle(shared_str /*N*/)
{
    return MotionID();
}

MotionID OzzKinematics::ID_Cycle_Safe(shared_str /*N*/)
{
    return MotionID();
}

CBlend* OzzKinematics::PlayCycle(LPCSTR /*N*/, BOOL /*bMixIn*/, PlayCallback /*Callback*/, LPVOID /*CallbackParam*/, u8 /*channel*/)
{
    return nullptr;
}

CBlend* OzzKinematics::PlayCycle(MotionID /*M*/, BOOL /*bMixIn*/, PlayCallback /*Callback*/, LPVOID /*CallbackParam*/, u8 /*channel*/)
{
    return nullptr;
}

CBlend* OzzKinematics::PlayCycle(u16 /*partition*/, MotionID /*M*/, BOOL /*bMixIn*/, PlayCallback /*Callback*/,
    LPVOID /*CallbackParam*/, u8 /*channel*/)
{
    return nullptr;
}

MotionID OzzKinematics::ID_FX(LPCSTR /*N*/)
{
    return MotionID();
}

MotionID OzzKinematics::ID_FX_Safe(LPCSTR /*N*/)
{
    return MotionID();
}

CBlend* OzzKinematics::PlayFX(LPCSTR /*N*/, float /*power_scale*/)
{
    return nullptr;
}

CBlend* OzzKinematics::PlayFX(MotionID /*M*/, float /*power_scale*/)
{
    return nullptr;
}

CBlend* OzzKinematics::PlayFX_Safe(cpcstr /*N*/, float /*power_scale*/)
{
    return nullptr;
}

const CPartition& OzzKinematics::partitions() const
{
    return default_partition_;
}

float OzzKinematics::get_animation_length(MotionID /*motion_ID*/)
{
    if (animation_controller_)
        return animation_controller_->Duration();
    return 0.f;
}

IRenderVisual* OzzKinematics::dcast_RenderVisual()
{
    NotImplemented(__FUNCTION__);
    return nullptr;
}

IKinematics* OzzKinematics::dcast_PKinematics()
{
    return this;
}

IKinematicsAnimated* OzzKinematics::dcast_PKinematicsAnimated()
{
    return this;
}

#ifdef DEBUG
std::pair<LPCSTR, LPCSTR> OzzKinematics::LL_MotionDefName_dbg(MotionID /*ID*/)
{
    static xr_string empty;
    return { empty.c_str(), empty.c_str() };
}

void OzzKinematics::LL_DumpBlends_dbg()
{
    Msg("[OzzKinematics] LL_DumpBlends_dbg not implemented");
}

void OzzKinematics::DebugRender(Fmatrix& /*XFORM*/)
{
    NotImplemented(__FUNCTION__);
}

shared_str OzzKinematics::getDebugName()
{
    NotImplemented(__FUNCTION__);
    return shared_str("OzzKinematics");
}
#endif
} // namespace Animation
} // namespace XRay
