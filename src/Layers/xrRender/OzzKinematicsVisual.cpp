#include "stdafx.h"

#include "OzzKinematicsVisual.h"

#include "Layers/xrRender_R2/r2.h"

#include "BufferUtils.h"
#include "FVisual.h"
#include "xrEngine/Render.h"

#include "xrAnimation/OzzConversion.h"
#include "xrCore/FMesh.hpp"
#include "xrCore/xrMemory.h"

#include "xrCommon/xr_string.h"

#include <algorithm>
#include <array>
#include <cstring>
#include <string>

namespace xray::render::RENDER_NAMESPACE
{
using inherited = FHierrarhyVisual;
using XRay::Animation::ConvertOzzMatrixToXRay;

namespace
{
constexpr u32 kMaxDumpedBones = 4;

bool ShouldDumpPalette()
{
    if (RImplementation.IsOzzPaletteDebugDumpEnabled())
        return true;

    return RImplementation.ConsumeOzzPaletteDebugDumpRequest();
}

void DebugDumpPalette(const COzzKinematicsVisual& visual, const xr_vector<Fmatrix>& palette)
{
    if (!ShouldDumpPalette())
        return;

    const char* label = "<ozz_visual>";
#ifdef DEBUG
    if (visual.dbg_name.size())
        label = visual.dbg_name.c_str();
#endif

    const u32 bone_count = static_cast<u32>(palette.size());
    Msg("[ozz][palette] visual=%s bones=%u", label, bone_count);

    const u32 print_count = std::min(bone_count, kMaxDumpedBones);
    for (u32 idx = 0; idx < print_count; ++idx)
    {
        const Fmatrix& bone = palette[idx];
        Msg("[ozz][palette] bone[%u] i(%.3f %.3f %.3f) j(%.3f %.3f %.3f) k(%.3f %.3f %.3f) c(%.3f %.3f %.3f)", idx,
            bone.i.x, bone.i.y, bone.i.z,
            bone.j.x, bone.j.y, bone.j.z,
            bone.k.x, bone.k.y, bone.k.z,
            bone.c.x, bone.c.y, bone.c.z);
    }

    if (bone_count > print_count)
        Msg("[ozz][palette] ... (omitted %u bones)", bone_count - print_count);
}
} // namespace

namespace
{
struct OzzGpuVertex
{
    Fvector position;
    Fvector normal;
    Fvector4 tangent;
    Fvector2 uv;
};

constexpr VertexElement OzzVertexDecl[] =
{
    { 0, 0,  D3DDECLTYPE_FLOAT3, D3DDECLMETHOD_DEFAULT, D3DDECLUSAGE_POSITION, 0 },
    { 0, 12, D3DDECLTYPE_FLOAT3, D3DDECLMETHOD_DEFAULT, D3DDECLUSAGE_NORMAL,   0 },
    { 0, 24, D3DDECLTYPE_FLOAT4, D3DDECLMETHOD_DEFAULT, D3DDECLUSAGE_TANGENT,  0 },
    { 0, 40, D3DDECLTYPE_FLOAT2, D3DDECLMETHOD_DEFAULT, D3DDECLUSAGE_TEXCOORD, 0 },
    D3DDECL_END()
};
} // namespace

class COzzSkinnedSurface final : public dxRender_Visual
{
public:
    COzzSkinnedSurface(COzzKinematicsVisual& owner, const ozz::sample::Mesh& mesh);
    ~COzzSkinnedSurface() override = default;

    void Render(CBackend& cmd_list, float, bool) override;
    void Copy(dxRender_Visual* from) override;
    void Release() override;

    void MarkDirty() { dirty_ = true; }

private:
    struct Influence
    {
        u16 bone_index = 0;
        u16 remap_index = 0;
        float weight = 0.f;
    };

    struct SourceVertex
    {
        Fvector position;
        Fvector normal;
        Fvector4 tangent;
        Fvector2 uv;
        std::array<Influence, 4> influences{};
        u8 influence_count = 0;
    };

private:
    void InitializeGeometry(const ozz::sample::Mesh& mesh);
    void UpdateGeometry();

private:
    COzzKinematicsVisual& owner_;
    xr_vector<SourceVertex> source_vertices_;
    xr_vector<u16> indices_;
    xr_vector<Fmatrix> inverse_bind_poses_;
    xr_vector<u16> joint_remaps_;
    xr_unique_ptr<VertexStagingBuffer> vertex_buffer_;
    xr_unique_ptr<IndexStagingBuffer> index_buffer_;
    ref_geom geom_;
    u32 vertex_count_ = 0;
    u32 primitive_count_ = 0;
    bool dirty_ = true;
};

static inline Fvector2 ReadUV(const ozz::vector<float>& uvs, int index)
{
    if (uvs.empty())
        return Fvector2().set(0.f, 0.f);
    const int offset = index * ozz::sample::Mesh::Part::kUVsCpnts;
    return Fvector2().set(uvs[offset + 0], uvs[offset + 1]);
}

static inline Fvector ReadVector3(const ozz::vector<float>& data, int index, int components)
{
    if (data.empty())
        return Fvector().set(0.f, 0.f, 0.f);
    const int offset = index * components;
    return Fvector().set(data[offset + 0], data[offset + 1], data[offset + 2]);
}

static inline Fvector4 ReadVector4(const ozz::vector<float>& data, int index, int components)
{
    if (data.empty())
        return Fvector4().set(0.f, 0.f, 0.f, 1.f);
    const int offset = index * components;
    return Fvector4().set(data[offset + 0], data[offset + 1], data[offset + 2], data[offset + 3]);
}

COzzSkinnedSurface::COzzSkinnedSurface(COzzKinematicsVisual& owner, const ozz::sample::Mesh& mesh)
    : owner_(owner)
{
    Type = MT_OZZ_BUNDLE;
    InitializeGeometry(mesh);
}

void COzzSkinnedSurface::InitializeGeometry(const ozz::sample::Mesh& mesh)
{
    vertex_count_ = static_cast<u32>(mesh.vertex_count());
    primitive_count_ = static_cast<u32>(mesh.triangle_index_count() / 3);

    joint_remaps_.assign(mesh.joint_remaps.begin(), mesh.joint_remaps.end());
    inverse_bind_poses_.resize(mesh.inverse_bind_poses.size());
    for (size_t idx = 0; idx < mesh.inverse_bind_poses.size(); ++idx)
        inverse_bind_poses_[idx] = ConvertOzzMatrixToXRay(mesh.inverse_bind_poses[idx]);
    if (inverse_bind_poses_.empty())
        inverse_bind_poses_.push_back(Fidentity);
    if (joint_remaps_.empty())
        joint_remaps_.push_back(0);

    source_vertices_.resize(vertex_count_);

    const char* shader_id = mesh.xray_metadata.shader_name.empty() ? "default_object" : mesh.xray_metadata.shader_name.c_str();
    const char* texture_id = mesh.xray_metadata.texture_path.empty() ? shader_id : mesh.xray_metadata.texture_path.c_str();
    shader.create(shader_id, texture_id);

    int vertex_base = 0;
    for (const auto& part : mesh.parts)
    {
        const int influences = std::min(part.influences_count(), 4);
        const int vertex_count = part.vertex_count();

        for (int local = 0; local < vertex_count; ++local)
        {
            const int vertex_index = vertex_base + local;
            SourceVertex& dst = source_vertices_[vertex_index];

            dst.position = ReadVector3(part.positions, local, ozz::sample::Mesh::Part::kPositionsCpnts);
            dst.normal = ReadVector3(part.normals, local, ozz::sample::Mesh::Part::kNormalsCpnts);
            dst.tangent = ReadVector4(part.tangents, local, ozz::sample::Mesh::Part::kTangentsCpnts);
            dst.uv = ReadUV(part.uvs, local);

            dst.influence_count = static_cast<u8>(influences == 0 ? 1 : influences);

            float accumulated_weight = 0.f;
            for (int influence = 0; influence < dst.influence_count; ++influence)
            {
                Influence data{};
                const int joint_stride = part.influences_count();
                const int weight_stride = std::max(0, joint_stride - 1);

                if (joint_stride == 0)
                {
                    data.bone_index = 0;
                    data.remap_index = 0;
                    data.weight = 1.f;
                }
                else
                {
                    const int joint_index = part.joint_indices[local * joint_stride + influence];
                    data.remap_index = static_cast<u16>(joint_index);
                    const u16 bone_index = (joint_index < static_cast<int>(mesh.joint_remaps.size())) ? mesh.joint_remaps[joint_index] : 0;
                    data.bone_index = bone_index;

                    if (influence < weight_stride)
                    {
                        data.weight = part.joint_weights[local * weight_stride + influence];
                        accumulated_weight += data.weight;
                    }
                    else
                    {
                        data.weight = std::max(0.f, 1.f - accumulated_weight);
                    }
                }

                dst.influences[influence] = data;
            }

            for (int influence = dst.influence_count; influence < 4; ++influence)
                dst.influences[influence] = Influence{};
        }

        vertex_base += vertex_count;
    }

    indices_.assign(mesh.triangle_indices.begin(), mesh.triangle_indices.end());

    vertex_buffer_ = xr_make_unique<VertexStagingBuffer>();
    vertex_buffer_->Create(static_cast<size_t>(vertex_count_) * sizeof(OzzGpuVertex), true);

    index_buffer_ = xr_make_unique<IndexStagingBuffer>();
    index_buffer_->Create(static_cast<size_t>(indices_.size()) * sizeof(u16), false, true);
    if (!indices_.empty())
    {
        auto* dst = static_cast<u16*>(index_buffer_->Map());
        std::memcpy(dst, indices_.data(), indices_.size() * sizeof(u16));
        index_buffer_->Unmap(true);
    }

    geom_.create(OzzVertexDecl, *vertex_buffer_, *index_buffer_);

    dirty_ = true;
}

void COzzSkinnedSurface::UpdateGeometry()
{
    if (!dirty_ || source_vertices_.empty())
        return;

    owner_.EnsureSkinningPalette();
    const xr_vector<Fmatrix>& palette = owner_.SkinningPalette();

    xr_vector<Fmatrix> skin_matrices(inverse_bind_poses_.size());
    for (size_t idx = 0; idx < inverse_bind_poses_.size(); ++idx)
    {
        const u16 bone_index = (idx < joint_remaps_.size()) ? joint_remaps_[idx] : 0;
        if (bone_index < palette.size())
            skin_matrices[idx].mul_43(palette[bone_index], inverse_bind_poses_[idx]);
        else
            skin_matrices[idx] = inverse_bind_poses_[idx];
    }

    auto* gpu_vertices = static_cast<OzzGpuVertex*>(vertex_buffer_->Map());
    for (u32 vertex = 0; vertex < vertex_count_; ++vertex)
    {
        const SourceVertex& src = source_vertices_[vertex];
        Fvector skinned_pos = { 0.f, 0.f, 0.f };
        Fvector skinned_normal = { 0.f, 0.f, 0.f };
        Fvector skinned_tangent = { 0.f, 0.f, 0.f };

        for (u8 influence = 0; influence < src.influence_count; ++influence)
        {
            const Influence& data = src.influences[influence];
            if (data.weight <= 0.f)
                continue;

            const Fmatrix& skin = (data.remap_index < skin_matrices.size()) ? skin_matrices[data.remap_index] : Fidentity;

            Fvector contribution;
            skin.transform_tiny(contribution, src.position);
            skinned_pos.mad(contribution, data.weight);

            Fvector tmp_normal = src.normal;
            skin.transform_dir(tmp_normal);
            skinned_normal.mad(tmp_normal, data.weight);

            Fvector tmp_tangent;
            tmp_tangent.set(src.tangent.x, src.tangent.y, src.tangent.z);
            skin.transform_dir(tmp_tangent);
            skinned_tangent.mad(tmp_tangent, data.weight);
        }

        if (skinned_normal.square_magnitude() > EPS_S)
            skinned_normal.normalize();
        if (skinned_tangent.square_magnitude() > EPS_S)
            skinned_tangent.normalize();

        OzzGpuVertex& dst = gpu_vertices[vertex];
        dst.position = skinned_pos;
        dst.normal = skinned_normal;
        dst.tangent.set(skinned_tangent.x, skinned_tangent.y, skinned_tangent.z, src.tangent.w);
        dst.uv = src.uv;
    }

    vertex_buffer_->Unmap(true);
    dirty_ = false;
}

void COzzSkinnedSurface::Render(CBackend& cmd_list, float, bool)
{
    UpdateGeometry();

    cmd_list.set_Geometry(geom_);
    cmd_list.set_xform_world(cmd_list.xforms.m_w);
    cmd_list.Render(D3DPT_TRIANGLELIST, 0, 0, vertex_count_, 0, primitive_count_);
    cmd_list.stat.r.s_dynamic.add(vertex_count_);
}

void COzzSkinnedSurface::Copy(dxRender_Visual*)
{
    R_ASSERT(false && "COzzSkinnedSurface::Copy should not be invoked directly");
}

void COzzSkinnedSurface::Release()
{
    geom_.destroy();
    vertex_buffer_.reset();
    index_buffer_.reset();
}

COzzKinematicsVisual::COzzKinematicsVisual()
{
    Type = MT_OZZ_BUNDLE;
}

COzzKinematicsVisual::~COzzKinematicsVisual()
{
    DestroySurfaces();
}

void COzzKinematicsVisual::DestroySurfaces()
{
    for (auto* surface : surfaces_)
    {
        if (!surface)
            continue;

        surface->Release();
        xr_delete(surface);
    }
    surfaces_.clear();
}

void COzzKinematicsVisual::Load(const char* N, IReader* data, u32)
{
    R_ASSERT2(false, "COzzKinematicsVisual::Load should not be invoked directly. Use LoadFromBundle().");
}

bool COzzKinematicsVisual::LoadFromBundle(const char* name, const std::filesystem::path& path)
{
#ifdef DEBUG
    dbg_name = name;
#endif

    XRay::Animation::OzzxBundle bundle;
    xr_string error_msg = "Failed to read .ozzx bundle: ";
    error_msg += path.string().c_str();
    R_ASSERT2(XRay::Animation::ReadOzzxBundle(path, bundle), error_msg.c_str());

    R_ASSERT2(!bundle.skeleton.empty(), "Ozz bundle missing skeleton payload");

    skeleton_payload_.assign(bundle.skeleton.begin(), bundle.skeleton.end());
    mesh_payload_.assign(bundle.mesh.begin(), bundle.mesh.end());

    ozz::span<const std::byte> skeleton_span(reinterpret_cast<const std::byte*>(skeleton_payload_.data()), skeleton_payload_.size());
    xr_string init_error = "Failed to initialize OzzKinematics from bundle: ";
    init_error += path.string().c_str();
    R_ASSERT2(kinematics_.InitializeFromOzzBuffer(skeleton_span), init_error.c_str());

    animation_controller_ = xr_make_unique<XRay::Animation::OzzAnimationController>();
    animation_controller_->Initialize(kinematics_.Skeleton());
    animation_applied_ = false;
    last_animation_update_frame_ = u32(-1);

    meshes_.clear();
    if (!mesh_payload_.empty())
    {
        ozz::io::MemoryStream mesh_stream;
        R_ASSERT2(mesh_stream.Write(mesh_payload_.data(), mesh_payload_.size()), "Failed to seed mesh stream from bundle payload");
        mesh_stream.Seek(0, ozz::io::Stream::kSet);

        ozz::io::IArchive archive(&mesh_stream);
        while (archive.TestTag<ozz::sample::Mesh>())
        {
            meshes_.emplace_back();
            archive >> meshes_.back();
        }
    }

    DestroySurfaces();
    children.clear();
    surfaces_.reserve(meshes_.size());
    for (const auto& mesh : meshes_)
    {
        auto* surface = xr_new<COzzSkinnedSurface>(*this, mesh);
        children.push_back(surface);
        surfaces_.push_back(surface);
    }
    bDontDelete = TRUE;

    kinematics_.SetUpdateCallback(&COzzKinematicsVisual::HandleKinematicsUpdated);
    kinematics_.SetUpdateCallbackParam(this);

    kinematics_.CalculateBones(TRUE);
    OnPoseUpdated();
    EnsureSkinningPalette();

    return true;
}

void COzzKinematicsVisual::Copy(dxRender_Visual* pFrom)
{
    dxRender_Visual::Copy(pFrom);

    if (auto* other = dynamic_cast<COzzKinematicsVisual*>(pFrom))
    {
        skeleton_payload_ = other->skeleton_payload_;
        mesh_payload_ = other->mesh_payload_;
        meshes_ = other->meshes_;

        if (!skeleton_payload_.empty())
        {
            ozz::span<const std::byte> span(reinterpret_cast<const std::byte*>(skeleton_payload_.data()), skeleton_payload_.size());
            R_ASSERT2(kinematics_.InitializeFromOzzBuffer(span), "Failed to copy OzzKinematicsVisual state");
        }

        DestroySurfaces();
        children.clear();
        surfaces_.reserve(meshes_.size());
        for (const auto& mesh : meshes_)
        {
            auto* surface = xr_new<COzzSkinnedSurface>(*this, mesh);
            children.push_back(surface);
            surfaces_.push_back(surface);
        }
        bDontDelete = TRUE;

        kinematics_.SetUpdateCallback(&COzzKinematicsVisual::HandleKinematicsUpdated);
        kinematics_.SetUpdateCallbackParam(this);

        kinematics_.CalculateBones(TRUE);
        OnPoseUpdated();
        EnsureSkinningPalette();
        last_animation_update_frame_ = u32(-1);
    }
}

void COzzKinematicsVisual::Release()
{
    kinematics_.SetUpdateCallback(nullptr);
    kinematics_.SetUpdateCallbackParam(nullptr);
    skeleton_payload_.clear();
    mesh_payload_.clear();
    DestroySurfaces();
    children.clear();
    bone_palette_.clear();
    animation_controller_.reset();
    animation_applied_ = false;
    last_animation_update_frame_ = u32(-1);
    palette_dirty_ = true;
    inherited::Release();
}

void COzzKinematicsVisual::UpdateBounds()
{
    const Fbox& box = kinematics_.GetBox();
    vis.box = box;

    if (box.is_valid())
    {
        Fvector center;
        float radius = 0.f;
        vis.box.getsphere(center, radius);
        vis.sphere.P = center;
        vis.sphere.R = radius;
    }
    else
    {
        vis.sphere.P.set(0.f, 0.f, 0.f);
        vis.sphere.R = 0.f;
    }
}

void COzzKinematicsVisual::UpdateSkinningPalette()
{
    palette_dirty_ = true;
    for (auto* surface : surfaces_)
    {
        if (surface)
            surface->MarkDirty();
    }
}

void COzzKinematicsVisual::EnsureSkinningPalette()
{
    if (!palette_dirty_ && !bone_palette_.empty())
        return;

    const u32 frame_id = Device.dwFrame;
    if (frame_id != last_animation_update_frame_)
    {
        UpdateAnimation(Device.fTimeDelta);
        last_animation_update_frame_ = frame_id;
    }

    if (!kinematics_.HasBones())
    {
        bone_palette_.clear();
        palette_dirty_ = false;
        return;
    }

    kinematics_.CalculateBones(TRUE);
    kinematics_.BuildSkinningPalette(bone_palette_, true);
    palette_dirty_ = false;

    DebugDumpPalette(*this, bone_palette_);
}

const xr_vector<Fmatrix>& COzzKinematicsVisual::SkinningPalette()
{
    EnsureSkinningPalette();
    return bone_palette_;
}

void COzzKinematicsVisual::OnPoseUpdated()
{
    UpdateBounds();
    UpdateSkinningPalette();
}

void COzzKinematicsVisual::HandleKinematicsUpdated(IKinematics* kin)
{
    auto* visual = static_cast<COzzKinematicsVisual*>(kin ? kin->GetUpdateCallbackParam() : nullptr);
    if (!visual)
        return;
    visual->OnPoseUpdated();
}

bool COzzKinematicsVisual::LoadAnimationFromFile(const std::filesystem::path& path)
{
    if (!animation_controller_)
    {
        animation_controller_ = xr_make_unique<XRay::Animation::OzzAnimationController>();
        animation_controller_->Initialize(kinematics_.Skeleton());
    }

    if (!animation_controller_->LoadAnimation(path))
        return false;

    animation_applied_ = false;
    last_animation_update_frame_ = u32(-1);
    UpdateSkinningPalette();
    return true;
}

void COzzKinematicsVisual::StopAnimation()
{
    if (!animation_controller_)
        return;
    animation_controller_->ClearAnimation();
    animation_applied_ = false;
    kinematics_.ClearPose();
    UpdateSkinningPalette();
    last_animation_update_frame_ = u32(-1);
}

void COzzKinematicsVisual::UpdateAnimation(float dt)
{
    if (!animation_controller_)
        return;

    if (!animation_controller_->HasAnimation())
    {
        if (animation_applied_)
        {
            kinematics_.ClearPose();
            animation_applied_ = false;
            UpdateSkinningPalette();
        }
        return;
    }

    if (!animation_controller_->Update(dt))
        return;

    const auto locals = animation_controller_->SampledLocals();
    if (locals.empty())
        return;

    if (!kinematics_.SetPoseLocals(locals))
        return;

    animation_applied_ = true;
    UpdateSkinningPalette();
}
} // namespace xray::render::RENDER_NAMESPACE
