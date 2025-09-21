#include "stdafx.h"

#include "OzzKinematics.h"

#include "ozz/base/io/archive.h"
#include "ozz/base/io/stream.h"
#include "ozz/base/maths/simd_math.h"
#include "ozz/base/span.h"

#include <algorithm>
#include <vector>

#include <algorithm>

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

Fvector3 ConvertOzzVectorToXRay(float x, float y, float z)
{
    Fvector3 result;
    result.x = x;
    result.y = y;
    result.z = -z;
    return result;
}

Fmatrix ConvertOzzMatrixToXRay(const ozz::math::Float4x4& matrix)
{
    float column[4];

    Fmatrix out;
    out.identity();

    ozz::math::Store3PtrU(matrix.cols[0], column);
    out.i = ConvertOzzVectorToXRay(column[0], column[1], column[2]);

    ozz::math::Store3PtrU(matrix.cols[1], column);
    out.j = ConvertOzzVectorToXRay(column[0], column[1], column[2]);

    ozz::math::Store3PtrU(matrix.cols[2], column);
    out.k = ConvertOzzVectorToXRay(column[0], column[1], column[2]);

    ozz::math::Store3PtrU(matrix.cols[3], column);
    out.c = ConvertOzzVectorToXRay(column[0], column[1], column[2]);

    out._14_ = 0.0f;
    out._24_ = 0.0f;
    out._34_ = 0.0f;
    out._44_ = 1.0f;

    return out;
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

    ozz::io::File file(skeleton_path, "rb");
    if (!file.opened())
    {
        Msg("[OzzKinematics] Failed to open skeleton file: %s", skeleton_path);
        return false;
    }

    ozz::io::IArchive archive(&file);
    if (!archive.TestTag<ozz::animation::Skeleton>())
    {
        Msg("[OzzKinematics] File does not contain a skeleton: %s", skeleton_path);
        return false;
    }

    archive >> skeleton_;

    const int joint_count = skeleton_.num_joints();
    if (joint_count <= 0)
    {
        Msg("[OzzKinematics] Skeleton contains no joints: %s", skeleton_path);
        return false;
    }

    bone_instances_.resize(joint_count);
    for (CBoneInstance& instance : bone_instances_)
        instance.construct();

    bone_boxes_.resize(joint_count);
    for (Fobb& box : bone_boxes_)
        box.invalidate();

    bones_.assign(joint_count, nullptr);

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

    cached_box_.invalidate();
    last_update_time_ = 0;
    visibility_counter_ = 0;

    return true;
}

void OzzKinematics::NotImplemented(pcstr function_name) const
{
    Msg("[OzzKinematics] %s is not implemented yet", function_name);
}

void OzzKinematics::Bone_Calculate(CBoneData* /*bd*/, Fmatrix* /*parent*/)
{
    NotImplemented(__FUNCTION__);
}

void OzzKinematics::Bone_GetAnimPos(Fmatrix& pos, u16 /*id*/, u8 /*channel_mask*/, bool /*ignore_callbacks*/)
{
    NotImplemented(__FUNCTION__);
    pos.identity();
}

bool OzzKinematics::PickBone(const Fmatrix& /*parent_xform*/, pick_result& /*r*/, float /*dist*/, const Fvector& /*start*/, const Fvector& /*dir*/,
    u16 /*bone_id*/)
{
    NotImplemented(__FUNCTION__);
    return false;
}

void OzzKinematics::EnumBoneVertices(SEnumVerticesCallback& /*C*/, u16 /*bone_id*/)
{
    NotImplemented(__FUNCTION__);
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

    NotImplemented(__FUNCTION__);
    return StubBoneData();
}

const IBoneData& OzzKinematics::GetBoneData(u16 bone_id) const
{
    if (bone_id < bones_.size() && bones_[bone_id])
        return *bones_[bone_id];

    const_cast<OzzKinematics*>(this)->NotImplemented(__FUNCTION__);
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
    NotImplemented(__FUNCTION__);
    matrices.clear();
}

int OzzKinematics::LL_GetBoneGroups(xr_vector<xr_vector<u16>>& groups)
{
    NotImplemented(__FUNCTION__);
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
    if (bone_id >= 64)
        return;
    const u64 mask = u64(1) << bone_id;
    if (val)
        visible_mask_ |= mask;
    else
        visible_mask_ &= ~mask;
}

u64 OzzKinematics::LL_GetBonesVisible()
{
    return visible_mask_;
}

void OzzKinematics::LL_SetBonesVisible(u64 mask)
{
    visible_mask_ = mask;
}

void OzzKinematics::LL_AddTransformToBone(KinematicsABT::additional_bone_transform& /*offset*/)
{
    NotImplemented(__FUNCTION__);
}

void OzzKinematics::LL_ClearAdditionalTransform(u16 /*bone_id*/)
{
    NotImplemented(__FUNCTION__);
}

void OzzKinematics::CalculateBones(BOOL /*bForceExact*/)
{
    if (skeleton_.num_joints() == 0 || bone_instances_.empty())
        return;

    std::vector<ozz::math::Float4x4> model_matrices(bone_instances_.size());

    ozz::animation::LocalToModelJob job;
    job.input = skeleton_.joint_rest_poses();
    job.output = ozz::span<ozz::math::Float4x4>(model_matrices.data(), model_matrices.size());
    job.skeleton = &skeleton_;

    if (!job.Run())
    {
        Msg("[OzzKinematics] LocalToModelJob failed during CalculateBones");
        return;
    }

    const size_t bone_count = bone_instances_.size();
    for (size_t i = 0; i < bone_count; ++i)
    {
        const Fmatrix transform = ConvertOzzMatrixToXRay(model_matrices[i]);
        bone_instances_[i].mTransform = transform;
        bone_instances_[i].mRenderTransform = transform;
    }
}

void OzzKinematics::CalculateBones_Invalidate()
{
    last_update_time_ = 0;
    visibility_counter_ = 0;
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

IRenderVisual* OzzKinematics::dcast_RenderVisual()
{
    NotImplemented(__FUNCTION__);
    return nullptr;
}

IKinematicsAnimated* OzzKinematics::dcast_PKinematicsAnimated()
{
    NotImplemented(__FUNCTION__);
    return nullptr;
}

#ifdef DEBUG
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
