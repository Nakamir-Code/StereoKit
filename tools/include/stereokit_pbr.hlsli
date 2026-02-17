#ifndef _STEREOKIT_PBR_HLSLI
#define _STEREOKIT_PBR_HLSLI

#include <stereokit.hlsli>

///////////////////////////////////////////

float sk_pbr_mip_level(float ndotv) {
	float2 dx    = ddx(ndotv * sk_cubemap_i.x);
	float2 dy    = ddy(ndotv * sk_cubemap_i.y);
	float  delta = max(dot(dx, dx), dot(dy, dy));
	return 0.5 * log2(delta);
}

///////////////////////////////////////////

half3 sk_pbr_fresnel_schlick_roughness(half ndotv, half3 F0, half roughness) {
	// Sebastian approximates pow(1.0 - ndotv, 5.0) as exp2(-5.55473 * ndotv - 6.98316 * ndotv)
	// https://seblagarde.wordpress.com/2012/06/03/spherical-gaussien-approximation-for-blinn-phong-phong-and-fresnel/
	return F0 + (max(1.0h - roughness, F0) - F0) * exp2((-5.55473h * ndotv - 6.98316h) * ndotv);
}

///////////////////////////////////////////

// Karis, "Real Shading in Unreal Engine 4" (SIGGRAPH 2013)
// See: https://www.unrealengine.com/en-US/blog/physically-based-shading-on-mobile
half2 sk_pbr_brdf_appx(half roughness, half ndotv) {
	const half4 c0   = { -1, -0.0275, -0.572,  0.022 };
	const half4 c1   = {  1,  0.0425,  1.04,  -0.04  };
	half4       r    = roughness * c0 + c1;
	half        a004 = min(r.x * r.x, exp2(-9.28 * ndotv)) * r.x + r.y;
	half2       AB   = half2(-1.04, 1.04) * a004 + r.zw;
	return AB;
}

///////////////////////////////////////////

float4 sk_pbr_shade(half4 albedo, half3 irradiance, half ao, half metal, half rough, float3 view_dir, half3 surface_normal) {
	// View direction and reflection must stay float for precision
	float3 view        = normalize(view_dir);
	float3 reflection  = reflect(-view, (float3)surface_normal);
	half   ndotv       = (half)max(0, dot((float3)surface_normal, view));

	// Pre-compute specular AA kernel from screen-space normal derivatives.
	// All ddx/ddy calls stay in the same WQM region; only the sqrt is
	// deferred until after the cubemap fetch is issued.
	// Tokuyoshi, "Improved Geometric Specular Antialiasing" (JCGT 2021)
	half3 dndu            = ddx(surface_normal);
	half3 dndv            = ddy(surface_normal);
	half  variance        = 0.25h * (dot(dndu, dndu) + dot(dndv, dndv));
	half  kernelRoughness = min(variance, 0.18h);

	// Issue cubemap fetch from pre-AA roughness to shorten the dependent
	// read chain — removes v_sqrt (~16 clk) from the critical path.
	// The mip is an approximation anyway (Lagarde 2014); Tokuyoshi's
	// kernel only adds roughness at silhouette edges where the pre-filtered
	// map / BRDF lobe mismatch is already largest.
	// Lazarov, "Getting More Physical in Call of Duty: Black Ops II"
	// (SIGGRAPH 2013)
	float mip = (float)(rough * (1.7h - 0.7h * rough)) * sk_cubemap_i.z;
	mip = max(mip, sk_pbr_mip_level(ndotv));
	half3  prefilteredColor = (half3)sk_cubemap.SampleLevel(sk_cubemap_s, reflection, mip).rgb;

	// Apply specular AA after cubemap is in flight — sqrt runs hidden
	// under cubemap memory latency instead of extending the dep chain.
	rough = sqrt(saturate(rough * rough + kernelRoughness));

	half3 F0 = lerp(0.04h, albedo.rgb, metal);
	half3 F  = sk_pbr_fresnel_schlick_roughness(ndotv, F0, rough);
	half3 kS = F;

	half2  envBRDF          = sk_pbr_brdf_appx(rough, ndotv);
	half3  specular         = prefilteredColor * (F * envBRDF.x + envBRDF.y);

	// Multi-scattering energy compensation: recovers energy lost to
	// inter-reflections at high roughness, brightening rough metals.
	// Fdez-Aguera, "A Multiple-Scattering Microfacet Model for Real-Time
	// Image Based Lighting" (SIGGRAPH 2019)
	half3 energyCompensation = 1.0h + F0 * (1.0h / max(envBRDF.x + envBRDF.y, 0.001h) - 1.0h);
	specular *= energyCompensation;

	half3 kD = 1.0h - kS;
	kD *= 1.0h - metal;

	half3 diffuse = albedo.rgb * irradiance * ao;
	// Gotanda (tri-Ace, 2014): roughness-dependent retroreflection boost
	// approximating Disney/Burley diffuse for IBL. Near-free.
	// diffuse *= 1.0h + 0.5h * rough;
	half3 color   = kD * diffuse + specular * ao;

	return float4((float3)color, albedo.a);
}

///////////////////////////////////////////

#endif

