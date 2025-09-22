#include "stdafx.h"

#include "OzzKinematicsVisual.h"

#include "xrCore/FMesh.hpp"

namespace xray::render::RENDER_NAMESPACE
{
COzzKinematicsVisual::COzzKinematicsVisual()
{
    Type = MT_OZZ_BUNDLE;
}

COzzKinematicsVisual::~COzzKinematicsVisual() = default;

void COzzKinematicsVisual::Load(const char* N, IReader* data, u32 dwFlags)
{
    // .ozzx visuals are loaded through LoadFromBundle to preserve file context.
    R_ASSERT2(false, "COzzKinematicsVisual::Load should not be invoked directly. Use LoadFromBundle().");
}

bool COzzKinematicsVisual::LoadFromBundle(const char* name, const std::filesystem::path& path)
{
#ifdef DEBUG
    dbg_name = name;
#endif

    XRay::Animation::OzzxBundle bundle;
    std::string error_msg = "Failed to read .ozzx bundle: " + path.string();
    R_ASSERT2(XRay::Animation::ReadOzzxBundle(path, bundle), error_msg.c_str());

    R_ASSERT2(!bundle.skeleton.empty(), "Ozz bundle missing skeleton payload");

    skeleton_payload_.assign(bundle.skeleton.begin(), bundle.skeleton.end());
    mesh_payload_.assign(bundle.mesh.begin(), bundle.mesh.end());

    ozz::span<const std::byte> skeleton_span(reinterpret_cast<const std::byte*>(skeleton_payload_.data()), skeleton_payload_.size());
    std::string init_error = "Failed to initialize OzzKinematics from bundle: " + path.string();
    R_ASSERT2(kinematics_.InitializeFromOzzBuffer(skeleton_span), init_error.c_str());

    kinematics_.CalculateBones(TRUE);
    UpdateBounds();

    return true;
}

void COzzKinematicsVisual::Copy(dxRender_Visual* pFrom)
{
    inherited::Copy(pFrom);

    if (auto* other = dynamic_cast<COzzKinematicsVisual*>(pFrom))
    {
        skeleton_payload_ = other->skeleton_payload_;
        mesh_payload_ = other->mesh_payload_;

        if (!skeleton_payload_.empty())
        {
            ozz::span<const std::byte> span(reinterpret_cast<const std::byte*>(skeleton_payload_.data()), skeleton_payload_.size());
            R_ASSERT2(kinematics_.InitializeFromOzzBuffer(span), "Failed to copy OzzKinematicsVisual state");
        }

        kinematics_.CalculateBones(TRUE);
        UpdateBounds();
    }
}

void COzzKinematicsVisual::Release()
{
    skeleton_payload_.clear();
    mesh_payload_.clear();
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
} // namespace xray::render::RENDER_NAMESPACE
