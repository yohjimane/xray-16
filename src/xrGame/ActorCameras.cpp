#include "StdAfx.h"
#include "Actor.h"
#include "xrEngine/CameraBase.h"
#ifdef DEBUG
#include "PHDebug.h"
#endif
#include "Hit.h"
#include "PHDestroyable.h"
#include "Car.h"

#include "Weapon.h"
#include "Inventory.h"

#include "SleepEffector.h"
#include "ActorEffector.h"
#include "Level.h"
#include "xrCDB/Intersect.hpp"

#include "CharacterPhysicsSupport.h"
#include "EffectorShot.h"

#include "PHMovementControl.h"
#include "xrPhysics/IElevatorState.h"
#include "xrPhysics/ActorCameraCollision.h"
#include "IKLimbsController.h"
#include "GamePersistent.h"
#include "player_hud.h"

void CActor::cam_Set(EActorCameras style)
{
    CCameraBase* old_cam = cam_Active();
    cam_active = style;
    old_cam->OnDeactivate();
    cam_Active()->OnActivate(old_cam);
}
float CActor::f_Ladder_cam_limit = 1.f;
void CActor::cam_SetLadder()
{
    CCameraBase* C = cameras[eacFirstEye];
    g_LadderOrient();
    float yaw = (-XFORM().k.getH());
    float& cam_yaw = C->yaw;
    float delta_yaw = angle_difference_signed(yaw, cam_yaw);

    if (-f_Ladder_cam_limit < delta_yaw && f_Ladder_cam_limit > delta_yaw)
    {
        yaw = cam_yaw + delta_yaw;
        float lo = (yaw - f_Ladder_cam_limit);
        float hi = (yaw + f_Ladder_cam_limit);
        C->lim_yaw[0] = lo;
        C->lim_yaw[1] = hi;
        C->bClampYaw = true;
    }
}
void CActor::camUpdateLadder(float dt)
{
    if (!character_physics_support()->movement()->ElevatorState())
        return;
    if (cameras[eacFirstEye]->bClampYaw)
        return;
    float yaw = (-XFORM().k.getH());

    float& cam_yaw = cameras[eacFirstEye]->yaw;
    float delta = angle_difference_signed(yaw, cam_yaw);

    if (-0.05f < delta && 0.05f > delta)
    {
        yaw = cam_yaw + delta;
        float lo = (yaw - f_Ladder_cam_limit);
        float hi = (yaw + f_Ladder_cam_limit);
        cameras[eacFirstEye]->lim_yaw[0] = lo;
        cameras[eacFirstEye]->lim_yaw[1] = hi;
        cameras[eacFirstEye]->bClampYaw = true;
    }
    else
    {
        cam_yaw += delta * std::min(dt * 10.f, 1.f);
    }

    IElevatorState* es = character_physics_support()->movement()->ElevatorState();
    if (es && es->State() == clbClimbingDown)
    {
        float& cam_pitch = cameras[eacFirstEye]->pitch;
        const float ldown_pitch = cameras[eacFirstEye]->lim_pitch.y;
        float delta = angle_difference_signed(ldown_pitch, cam_pitch);
        if (delta > 0.f)
            cam_pitch += delta * std::min(dt * 10.f, 1.f);
    }
}

void CActor::cam_UnsetLadder()
{
    CCameraBase* C = cameras[eacFirstEye];
    C->lim_yaw[0] = 0;
    C->lim_yaw[1] = 0;
    C->bClampYaw = false;
}
float cammera_into_collision_shift = 0.05f;
float CActor::CameraHeight()
{
    Fvector R;
    character_physics_support()->movement()->Box().getsize(R);
    return m_fCamHeightFactor * (R.y - cammera_into_collision_shift);
}

IC float viewport_near(float& w, float& h)
{
    w = 2.f * VIEWPORT_NEAR * tan(deg2rad(Device.fFOV) / 2.f);
    h = w * Device.fASPECT;
    float c = _sqrt(w * w + h * h);
    return _max(_max(VIEWPORT_NEAR, _max(w, h)), c);
}

ICF void calc_point(Fvector& pt, float radius, float depth, float alpha)
{
    pt.x = radius * _sin(alpha);
    pt.y = radius + radius * _cos(alpha);
    pt.z = depth;
}
ICF void calc_gl_point(Fvector& pt, const Fmatrix& xform, float radius, float angle)
{
    calc_point(pt, radius, VIEWPORT_NEAR / 2, angle);
    xform.transform_tiny(pt);
}
ICF BOOL test_point(const Fvector& pt, xrXRC& xrc, const Fmatrix33& mat, const Fvector& ext)
{
    for (auto &it : *xrc.r_get())
    {
        CDB::RESULT& O = it;
        if (GMLib.GetMaterialByIdx(O.material)->Flags.is(SGameMtl::flPassable))
            continue;
        if (CDB::TestBBoxTri(mat, pt, ext, O.verts, FALSE))
            return TRUE;
    }
    return FALSE;
}

IC bool test_point(const Fvector& pt, const Fmatrix33& mat, const Fvector& ext, CActor* actor)
{
    Fmatrix fmat = Fidentity;
    fmat.i.set(mat.i);
    fmat.j.set(mat.j);
    fmat.k.set(mat.k);
    fmat.c.set(pt);
    // IPhysicsShellHolder * ve = smart_cast<IPhysicsShellHolder*> ( Level().CurrentEntity() ) ;
    VERIFY(actor);
    return test_camera_box(ext, fmat, actor);
}

#ifdef DEBUG
template <typename T>
void dbg_draw_viewport(const T& cam_info, float _viewport_near)
{
    VERIFY(_viewport_near > 0.f);
    const Fvector near_plane_center = Fvector().mad(cam_info.Position(), cam_info.Direction(), _viewport_near);
    float h_w, h_h;
    viewport_size(_viewport_near, cam_info, h_w, h_h);
    const Fvector right = Fvector().mul(cam_info.Right(), h_w);
    const Fvector up = Fvector().mul(cam_info.Up(), h_h);

    const Fvector top_left = Fvector().sub(near_plane_center, right).add(up);
    const Fvector top_right = Fvector().add(near_plane_center, right).add(up);
    const Fvector bottom_left = Fvector().sub(near_plane_center, right).sub(up);
    const Fvector bottom_right = Fvector().add(near_plane_center, right).sub(up);

    DBG_DrawLine(cam_info.Position(), top_left, color_xrgb(255, 0, 0));
    DBG_DrawLine(cam_info.Position(), top_right, color_xrgb(255, 0, 0));
    DBG_DrawLine(cam_info.Position(), bottom_left, color_xrgb(255, 0, 0));
    DBG_DrawLine(cam_info.Position(), bottom_right, color_xrgb(255, 0, 0));

    DBG_DrawLine(top_right, top_left, color_xrgb(255, 0, 0));
    DBG_DrawLine(bottom_right, top_right, color_xrgb(255, 0, 0));
    DBG_DrawLine(top_left, bottom_left, color_xrgb(255, 0, 0));
    DBG_DrawLine(bottom_left, bottom_right, color_xrgb(255, 0, 0));
}
#endif
IC void get_box_mat(Fmatrix33& mat, float alpha, const SRotation& r_torso)
{
    float dZ = ((PI_DIV_2 - ((PI + alpha) / 2)));
    Fmatrix xformR;
    xformR.setXYZ(-r_torso.pitch, r_torso.yaw, -dZ);
    mat.i = xformR.i;
    mat.j = xformR.j;
    mat.k = xformR.k;
}
IC void get_q_box(Fbox& xf, float c, float alpha, float radius)
{
    Fvector src_pt, tgt_pt;
    calc_point(tgt_pt, radius, 0, alpha);
    src_pt.set(0, tgt_pt.y, 0);
    xf.invalidate();
    xf.modify(src_pt);
    xf.modify(tgt_pt);
    xf.grow(c);
}

IC void get_cam_oob(Fvector& bc, Fvector& bd, Fmatrix33& mat, const Fmatrix& xform, const SRotation& r_torso,
    float alpha, float radius, float c)
{
    get_box_mat(mat, alpha, r_torso);
    Fbox xf;
    get_q_box(xf, c, alpha, radius);
    xf.xform(Fbox().set(xf), xform);
    // query
    xf.get_CD(bc, bd);
}
IC void get_cam_oob(
    Fvector& bd, Fmatrix& mat, const Fmatrix& xform, const SRotation& r_torso, float alpha, float radius, float c)
{
    Fmatrix33 mat3;
    Fvector bc;
    get_cam_oob(bc, bd, mat3, xform, r_torso, alpha, radius, c);
    mat.set(Fidentity);
    mat.i.set(mat3.i);
    mat.j.set(mat3.j);
    mat.k.set(mat3.k);
    mat.c.set(bc);
}
void CActor::cam_Lookout(const Fmatrix& xform, float camera_height)
{
    if (!fis_zero(r_torso_tgt_roll))
    {
        float w, h;
        float c = viewport_near(w, h);
        w /= 2.f;
        h /= 2.f;
        float alpha = r_torso_tgt_roll / 2.f;
        float radius = camera_height * 0.5f;
        // init valid angle
        float valid_angle = alpha;
        Fvector bc, bd;
        Fmatrix33 mat;
        get_cam_oob(bc, bd, mat, xform, r_torso, alpha, radius, c);

        /*
        xrXRC				xrc			;
        xrc.box_options		(0)			;
        xrc.box_query		(Level().ObjectSpace.GetStaticModel(), bc, bd)		;
        u32 tri_count		= xrc.r_count();

        */
        // if (tri_count)
        {
            float da = 0.f;
            BOOL bIntersect = FALSE;
            Fvector ext = {w, h, VIEWPORT_NEAR / 2};
            Fvector pt;
            calc_gl_point(pt, xform, radius, alpha);
            if (test_point(pt, mat, ext, this))
            {
                da = PI / 1000.f;
                if (!fis_zero(r_torso.roll))
                    da *= r_torso.roll / _abs(r_torso.roll);
                float angle;
                for (angle = 0.f; _abs(angle) < _abs(alpha); angle += da)
                {
                    Fvector pt;
                    calc_gl_point(pt, xform, radius, angle);
                    if (test_point(pt, mat, ext, this))
                    {
                        bIntersect = TRUE;
                        break;
                    }
                }
                valid_angle = bIntersect ? angle : alpha;
            }
        }
        r_torso.roll = valid_angle * 2.f;
        r_torso_tgt_roll = r_torso.roll;
    }
    else
    {
        r_torso_tgt_roll = 0.f;
        r_torso.roll = 0.f;
    }
}

BOOL ik_cam_shift = true;
float ik_cam_shift_tolerance = 0.2f;
float ik_cam_shift_speed = 0.01f;
float ik_cam_shift_interpolation = 4.f;

void CActor::cam_Update(float dt, float fFOV)
{
    if (m_holder)
        return;
    
    if ((mstate_real & mcClimb) && (cam_active != eacFreeLook))
        camUpdateLadder(dt);
    on_weapon_shot_update();
    float y_shift = 0;

    if (GamePersistent().GameType() != eGameIDSingle && ik_cam_shift && character_physics_support() &&
        character_physics_support()->ik_controller())
    {
        y_shift = character_physics_support()->ik_controller()->Shift();
        float cam_smooth_k = 1.f;
        if (_abs(y_shift - current_ik_cam_shift) > ik_cam_shift_tolerance)
        {
            cam_smooth_k = 1.f - ik_cam_shift_speed * dt / 0.01f;
        }

        if (_abs(y_shift) < ik_cam_shift_tolerance / 2.f)
            cam_smooth_k = 1.f - ik_cam_shift_speed * 1.f / 0.01f * dt;
        clamp(cam_smooth_k, 0.f, 1.f);
        current_ik_cam_shift = cam_smooth_k * current_ik_cam_shift + y_shift * (1.f - cam_smooth_k);
    }
    else
        current_ik_cam_shift = 0;

    // Alex ADD: smooth crouch fix
    const float targetHeight = CameraHeight();
    if (CurrentHeight < 0.0f)
        CurrentHeight = targetHeight;
    else if (!fsimilar(CurrentHeight, targetHeight))
    {
        const float dti = ik_cam_shift_interpolation * dt;
        CurrentHeight = (CurrentHeight * (1.0f - dti)) + (targetHeight * dti);
    }

    Fvector point = { 0, CurrentHeight + current_ik_cam_shift, 0 };

    Fvector dangle = {0, 0, 0};
    Fmatrix xform;
    xform.setXYZ(0, r_torso.yaw, 0);
    xform.translate_over(XFORM().c);

    // lookout
    if (this == Level().CurrentControlEntity())
        cam_Lookout(xform, point.y);

    if (!fis_zero(r_torso.roll))
    {
        float radius = point.y * 0.5f;
        float valid_angle = r_torso.roll / 2.f;
        calc_point(point, radius, 0, valid_angle);
        dangle.z = (PI_DIV_2 - ((PI + valid_angle) / 2));
    }

    float flCurrentPlayerY = xform.c.y;

    // Smooth out stair step ups
    if ((character_physics_support()->movement()->Environment() == CPHMovementControl::peOnGround) &&
        (flCurrentPlayerY - fPrevCamPos > 0))
    {
        fPrevCamPos += dt * 1.5f;
        if (fPrevCamPos > flCurrentPlayerY)
            fPrevCamPos = flCurrentPlayerY;
        if (flCurrentPlayerY - fPrevCamPos > 0.2f)
            fPrevCamPos = flCurrentPlayerY - 0.2f;
        point.y += fPrevCamPos - flCurrentPlayerY;
    }
    else
    {
        fPrevCamPos = flCurrentPlayerY;
    }

    float _viewport_near = VIEWPORT_NEAR;
    // calc point
    xform.transform_tiny(point);

    CCameraBase* C = cam_Active();

    C->Update(point, dangle);
    C->f_fov = fFOV;

    if (eacFirstEye != cam_active)
    {
        cameras[eacFirstEye]->Update(point, dangle);
        cameras[eacFirstEye]->f_fov = fFOV;
    }
    if (Level().CurrentEntity() == this)
    {
        collide_camera(*cameras[eacFirstEye], _viewport_near, this);
    }
    if (psActorFlags.test(AF_PSP))
    {
        Cameras().UpdateFromCamera(C);
    }
    else
    {
        Cameras().UpdateFromCamera(cameras[eacFirstEye]);
    }

    fCurAVelocity = vPrevCamDir.sub(cameras[eacFirstEye]->vDirection).magnitude() / Device.fTimeDelta;
    vPrevCamDir = cameras[eacFirstEye]->vDirection;

#ifdef DEBUG
    if (dbg_draw_camera_collision)
    {
        dbg_draw_viewport(*cameras[eacFirstEye], _viewport_near);
        dbg_draw_viewport(Cameras(), _viewport_near);
    }
#endif

    if (Level().CurrentEntity() == this)
    {
        Level().Cameras().UpdateFromCamera(C);
        const bool allow = !Level().Cameras().GetCamEffector(cefDemo) && !Level().Cameras().GetCamEffector(cefAnsel);
        if (eacFirstEye == cam_active && allow)
        {
            Cameras().ApplyDevice();
        }

        // yohji: add extra camera vars for shaders
        Fmatrix camMatrix;
        Fvector camHPB = cam_Active()->HPB();
        Fvector realCamHPB;
        Cameras().camera_Matrix(camMatrix);
        camMatrix.getHPB(realCamHPB);

        m_camHPBMagnitude = angle_difference_signed(m_prevCamHPB, camHPB).div(Device.fTimeDelta);
        m_camHPBMagnitudeReal = angle_difference_signed(realCamHPB, m_prevCamHPBReal).div(Device.fTimeDelta); // idk why, but we need to flip the values we pull from cam matrix otherwise the sign is flipped.
        
        // Dynamic lerp rate (2x as fast when we are returning to neutral)
        auto returnRate = [this]()
        {
            if (m_camHPBMagnitude.magnitude() > 0.f || m_camHPBMagnitudeReal.magnitude() > 0.f)
                return Device.fTimeDelta * 2;
            else
                return Device.fTimeDelta * 4;
        };

        // limit angles
        float angleLimit = 0.015f;
        auto angle_limit = [angleLimit](float curMagnitude)
        {
            float sign = curMagnitude > 0.f ? 1.f : -1.f;
            return _min(abs(curMagnitude), angleLimit) * sign;
        };

        //Msg("yohji debug - camHPB: [%f, %f, %f] prevCamHPB: [%f, %f, %f]", camHPB.x, camHPB.y, camHPB.z, m_prevCamHPB.x, m_prevCamHPB.y, m_prevCamHPB.z);
        //Msg("yohji debug - angle_limit: %f cur: %f return rate: %f", angle_limit(m_camHPBMagnitudeReal.x), m_camHPBMagnitudeSmooth.x, returnRate());
        // use average of magnitudes for x/y target, using only realMagnitude is too strong since these values can change without mouse movement.
        m_camHPBMagnitudeSmooth.x = angle_lerp(angle_limit(m_camHPBMagnitudeSmooth.x), angle_limit((m_camHPBMagnitudeReal.x + m_camHPBMagnitude.x) / 2.f), returnRate()); 
        m_camHPBMagnitudeSmooth.y = angle_lerp(angle_limit(m_camHPBMagnitudeSmooth.y), angle_limit((m_camHPBMagnitudeReal.y + m_camHPBMagnitude.y) / 2.f), returnRate());
        // use real magnitude for z target, to capture ALL movement on the z axis.
        m_camHPBMagnitudeSmooth.z = angle_lerp(angle_limit(m_camHPBMagnitudeSmooth.z), angle_limit(m_camHPBMagnitudeReal.z), returnRate());

        g_pGamePersistent->m_pGShaderConstants->cam_inertia_smooth.set(-m_camHPBMagnitudeSmooth.x, -m_camHPBMagnitudeSmooth.y, -m_camHPBMagnitudeSmooth.z, 0.f); // TODO: increase raindrop speed temporarily based on cam inertia magnitude

        Cameras().camera_Matrix(camMatrix);
        camMatrix.getHPB(realCamHPB);
        m_prevCamHPBReal.set(realCamHPB);
        m_prevCamHPB.set(camHPB);
    }
}

// shot effector stuff
void CActor::update_camera(CCameraShotEffector* effector)
{
    if (!effector)
        return;
    //	if (Level().CurrentViewEntity() != this) return;

    CCameraBase* pACam = cam_FirstEye();
    if (!pACam)
        return;

    if (pACam->bClampPitch)
    {
        while (pACam->pitch < pACam->lim_pitch[0])
            pACam->pitch += PI_MUL_2;
        while (pACam->pitch > pACam->lim_pitch[1])
            pACam->pitch -= PI_MUL_2;
    }

    effector->ChangeHP(&(pACam->pitch), &(pACam->yaw));

    if (pACam->bClampYaw)
        clamp(pACam->yaw, pACam->lim_yaw[0], pACam->lim_yaw[1]);
    if (pACam->bClampPitch)
        clamp(pACam->pitch, pACam->lim_pitch[0], pACam->lim_pitch[1]);

    if (effector && !effector->IsActive())
    {
        Cameras().RemoveCamEffector(eCEShot);
    }
}

#ifdef DEBUG
void dbg_draw_frustum(float FOV, float _FAR, float A, Fvector& P, Fvector& D, Fvector& U);
extern Flags32 dbg_net_Draw_Flags;
extern BOOL g_bDrawBulletHit;

void CActor::OnRender()
{
#ifdef DEBUG
    if (inventory().ActiveItem())
        inventory().ActiveItem()->OnRender();
#endif
    if (!bDebug)
        return;

    if ((dbg_net_Draw_Flags.is_any(dbg_draw_actor_phys)))
        character_physics_support()->movement()->dbg_Draw();

    OnRender_Network();

    inherited::OnRender();
}
#endif
