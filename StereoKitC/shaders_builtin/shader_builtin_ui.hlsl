#include "stereokit.hlsli"

//--name = sk/default_ui

//--color:color = 1, 1, 1, 1
//--diffuse     = white
float4       color = float4(1,1,1,1);
Texture2D    diffuse   : register(t0);
SamplerState diffuse_s : register(s0);

struct vsIn {
	float4 pos  : SV_Position;
	float3 norm : NORMAL0;
	float2 uv   : TEXCOORD0;
	float4 color: COLOR0;
};
struct psIn {
	float4 pos     : SV_POSITION;
	float2 uv      : TEXCOORD0;
	float3 world   : TEXCOORD1;
	half4  color   : COLOR0;
	SK_LAYER_OUTPUT
};

psIn vs(vsIn input, sk_input_t sys) {
	sk_ids_t ids = sk_resolve_ids(sys);

	psIn o;
	float3 normal = normalize(mul(input.norm, (float3x3) sk_inst[ids.inst].world));
	float4 world  = mul(input.pos, sk_inst[ids.inst].world);
	o.pos   = mul(world, sk_viewproj[ids.view]);
	o.world = world.xyz;
	o.uv    = input.uv;
	o.color.rgb = color.rgb * input.color.rgb * sk_inst[ids.inst].color.rgb * sk_lighting(normal);
	o.color.a   = input.color.a;
	SK_SET_LAYER(o, ids.view);
	return o;
}

float4 ps(psIn input) : SV_TARGET {
	half   glow = sk_finger_glow(input.world);
	float4 col  = float4(lerp(input.color.rgb, half3(1, 1, 1), glow), input.color.a);

	return diffuse.Sample(diffuse_s, input.uv) * col;
}