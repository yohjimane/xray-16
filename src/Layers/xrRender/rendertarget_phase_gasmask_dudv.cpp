#include "stdafx.h"

void CRenderTarget::phase_gasmask_dudv()
{
	//Constants
	u32 Offset = 0;
	u32 C = color_rgba(0, 0, 0, 255);

	float d_Z = EPS_S;
	float d_W = 1.0f;
	float w = float(Device.dwWidth);
	float h = float(Device.dwHeight);

	Fvector2 p0, p1;
#if defined(USE_DX10) || defined(USE_DX11)	
	p0.set(0.0f, 0.0f);
	p1.set(1.0f, 1.0f);
#else
	p0.set(0.5f / w, 0.5f / h);
	p1.set((w + 0.5f) / w, (h + 0.5f) / h);
#endif

    ///////////////////////////////////////////////////////////////////////////////////
    // Consolidation pass
    u_setrt(RCache, rt_mask_drops_blur, nullptr, nullptr, nullptr); //i assume you want to write data to rt_mask_Blur and read it in breath fog pass

    RCache.set_CullMode(CULL_NONE);
    RCache.set_Stencil(FALSE);

    //Fill vertex buffer
    FVF::TL* pv = (FVF::TL*)RImplementation.Vertex.Lock(4, g_combine->vb_stride, Offset);
    pv->set(0, float(h), d_Z, d_W, C, p0.x, p1.y); pv++;
    pv->set(0, 0, d_Z, d_W, C, p0.x, p0.y); pv++;
    pv->set(float(w), float(h), d_Z, d_W, C, p1.x, p1.y); pv++;
    pv->set(float(w), 0, d_Z, d_W, C, p1.x, p0.y); pv++;
    RImplementation.Vertex.Unlock(4, g_combine->vb_stride);

    //Set pass
    RCache.set_Element(s_gasmask_dudv->E[1]);

    //Set geometry
    RCache.set_Geometry(g_combine);
    RCache.Render(D3DPT_TRIANGLELIST, Offset, 0, 4, 0, 2);
	
	//////////////////////////////////////////////////////////////////////////
	//Set MSAA/NonMSAA rendertarget
#if defined(USE_DX10) || defined(USE_DX11)
	ref_rt& dest_rt = RImplementation.o.msaa ? rt_Generic : rt_Color;
	u_setrt(RCache, dest_rt, nullptr, nullptr, nullptr);
#else
	u_setrt(rt_Generic_0, nullptr, nullptr, nullptr);
#endif		

	RCache.set_CullMode(CULL_NONE);
	RCache.set_Stencil(FALSE);

	//Fill vertex buffer
	pv = (FVF::TL*)RImplementation.Vertex.Lock(4, g_combine->vb_stride, Offset);
	pv->set(0, float(h), d_Z, d_W, C, p0.x, p1.y); pv++;
	pv->set(0, 0, d_Z, d_W, C, p0.x, p0.y); pv++;
	pv->set(float(w), float(h), d_Z, d_W, C, p1.x, p1.y); pv++;
	pv->set(float(w), 0, d_Z, d_W, C, p1.x, p0.y); pv++;
    RImplementation.Vertex.Unlock(4, g_combine->vb_stride);

	//Set pass
	RCache.set_Element(s_gasmask_dudv->E[0]);

	//Set paramterers
	RCache.set_c("mask_control", ps_r2_mask_control.x, ps_r2_mask_control.y, ps_r2_mask_control.z, ps_r2_mask_control.w);

	//Set geometry
	RCache.set_Geometry(g_combine);
	RCache.Render(D3DPT_TRIANGLELIST, Offset, 0, 4, 0, 2);

#if defined(USE_DX10) || defined(USE_DX11)
    HW.get_context(CHW::IMM_CTX_ID)->CopyResource(rt_Generic_0->pTexture->surface_get(), dest_rt->pTexture->surface_get());
#endif

    ///////////////////////////////////////////////////////////////////////////////////
    // Breath shader
#if defined(USE_DX10) || defined(USE_DX11)
    dest_rt = RImplementation.o.msaa ? rt_Generic : rt_Color;
    u_setrt(RCache, dest_rt, nullptr, nullptr, nullptr);
#else
    u_setrt(rt_Generic_0, nullptr, nullptr, nullptr);
#endif

    RCache.set_CullMode(CULL_NONE);
    RCache.set_Stencil(FALSE);

    //Fill vertex buffer
    pv = (FVF::TL*)RImplementation.Vertex.Lock(4, g_combine->vb_stride, Offset);
    pv->set(0, float(h), d_Z, d_W, C, p0.x, p1.y); pv++;
    pv->set(0, 0, d_Z, d_W, C, p0.x, p0.y); pv++;
    pv->set(float(w), float(h), d_Z, d_W, C, p1.x, p1.y); pv++;
    pv->set(float(w), 0, d_Z, d_W, C, p1.x, p0.y); pv++;
    RImplementation.Vertex.Unlock(4, g_combine->vb_stride);

    //Set pass
    RCache.set_Element(s_gasmask_dudv->E[2]);

    //Set paramterers
    RCache.set_c("mask_control", ps_r2_mask_control.x, ps_r2_mask_control.y, ps_r2_mask_control.z, ps_r2_mask_control.w);

    //Set geometry
    RCache.set_Geometry(g_combine);
    RCache.Render(D3DPT_TRIANGLELIST, Offset, 0, 4, 0, 2);
	
#if defined(USE_DX10) || defined(USE_DX11)
    HW.get_context(CHW::IMM_CTX_ID)->CopyResource(rt_Generic_0->pTexture->surface_get(), dest_rt->pTexture->surface_get());
#endif
};
