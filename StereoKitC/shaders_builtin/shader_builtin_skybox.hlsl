#include "stereokit.hlsli"

//--name = sk/cubemap

struct vsIn {
	float4 pos : SV_Position;
};
struct psIn {
	float4 pos  : SV_Position;
	float3 norm : NORMAL0;
	SK_LAYER_OUTPUT
};

psIn vs(vsIn input, sk_input_t sys) {
	sk_ids_t ids = sk_resolve_ids(sys);

	psIn o;
	o.pos = float4(input.pos.xy, 1, 1);

	float4 proj_inv = mul(o.pos, sk_proj_inv[ids.view]);
	o.norm = mul(float4(proj_inv.xyz, 0), transpose(sk_view[ids.view])).xyz;
	SK_SET_LAYER(o, ids.view);
	return o;
}

float4 ps(psIn input) : SV_TARGET {
	return sk_cubemap.Sample(sk_cubemap_s, input.norm);
}