#include "stereokit.hlsli"
//--name = sk/depth_prepass

Texture2DArray depth_tex   : register(t1);
SamplerState   depth_tex_s : register(s1);

//--depth_view_proj_l = 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1
float4x4 depth_view_proj_l;
//--depth_view_proj_r = 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1
float4x4 depth_view_proj_r;

//--depth_view_proj_inv_l = 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1
float4x4 depth_view_proj_inv_l;
//--depth_view_proj_inv_r = 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1
float4x4 depth_view_proj_inv_r;

struct vsIn {
	float4 pos  : SV_Position;
	float3 norm : NORMAL0;
	float2 uv   : TEXCOORD0;
	float4 col  : COLOR0;
};
struct psIn {
	float4 pos        : SV_POSITION;
	float4 depth_clip : TEXCOORD0;
	uint   eye        : TEXCOORD1;
};
struct psOut {
	float  depth : SV_Depth;
	float4 color : SV_Target0;
};

psIn vs(vsIn input, sk_ids_t ids) {
	psIn o;
	o.pos = input.pos;
	o.eye = ids.view;

	// Unproject screen UV to view-space
	float2 ndc     = input.uv * 2.0 - 1.0;
	float4 near_vs = mul(float4(ndc.x, -ndc.y, 0, 1), sk_proj_inv[ids.view]);
	near_vs /= near_vs.w;

	// Rotate view-space ray to world space
	float3x3 view_rot = float3x3(
		sk_view[ids.view][0].xyz,
		sk_view[ids.view][1].xyz,
		sk_view[ids.view][2].xyz);
	float3 ray_ws = mul(near_vs.xyz, transpose(view_rot));

	// Project a far point along the ray into depth camera clip space
	float3   cam_pos = sk_camera_pos[ids.view].xyz;
	float4x4 dvp     = (o.eye == 0) ? depth_view_proj_l : depth_view_proj_r;
	o.depth_clip     = mul(dvp, float4(cam_pos + ray_ws * 100.0, 1));

	return o;
}

psOut ps(psIn input) {
	psOut o;

	// Reject behind depth camera
	if (input.depth_clip.w <= 0) { discard; o.depth = 0; o.color = 0; return o; }

	float2 depth_uv = input.depth_clip.xy / input.depth_clip.w * 0.5 + 0.5;
	if (any(depth_uv < 0) || any(depth_uv > 1)) { discard; o.depth = 0; o.color = 0; return o; }

	float depth_raw = depth_tex.SampleLevel(depth_tex_s, float3(depth_uv, (float)input.eye), 0).r;
	if (depth_raw <= 0 || depth_raw >= 1) { discard; o.depth = 0; o.color = 0; return o; }

	// Unproject depth texel to world space via inverse depth VP
	// depth_raw is a z-buffer value in [0,1], convert to NDC [-1,1]
	float4x4 dvp_inv = (input.eye == 0) ? depth_view_proj_inv_l : depth_view_proj_inv_r;
	float4 world_h   = mul(dvp_inv, float4(depth_uv * 2.0 - 1.0, depth_raw * 2.0 - 1.0, 1));
	float3 world_pos = world_h.xyz / world_h.w;

	// Project world position through rendering camera for z-buffer depth
	float4 render_clip = mul(float4(world_pos, 1), sk_viewproj[input.eye]);
	o.depth = render_clip.z / render_clip.w;
	o.color = float4(0, 0, 0, 0);
	return o;
}
