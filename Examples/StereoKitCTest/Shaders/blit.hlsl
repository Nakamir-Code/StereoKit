#include "stereokit.hlsli"

//--name = app/blit
//--source = white

Texture2D    source   : register(t0);
SamplerState source_s : register(s0);

cbuffer TransformBuffer : register(b3) {
	float sk_width;
	float sk_height;
	float sk_pixel_width;
	float sk_pixel_height;
};

struct vsIn {
	float4 pos  : SV_Position;
	float3 norm : NORMAL0;
	float2 uv   : TEXCOORD0;
	float4 col  : COLOR0;
};
struct psIn : sk_ps_input_t {
	float4 pos : SV_POSITION;
	float2 uv  : TEXCOORD0;
	SK_LAYER_OUTPUT
};

psIn vs(vsIn input, sk_input_t sys) {
	psIn o;
	sk_ids_t ids = sk_resolve_ids(sys);

	o.pos = input.pos;
	o.uv  = input.uv;
	SK_SET_LAYER(o, ids.view);
	return o;
}

float4 ps(psIn input) : SV_TARGET {
	return pow(abs(source.Sample(source_s, input.uv)), 2.2);
}