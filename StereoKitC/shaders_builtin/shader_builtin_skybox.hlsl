#include "stereokit.hlsli"

//--name = sk/cubemap

struct vsIn {
	float4 pos : SV_Position;
};
struct psIn {
	float4 pos  : SV_Position;
	float3 norm : NORMAL0;
};

psIn vs(vsIn input, sk_ids_t ids) {
	psIn o;
	o.pos = float4(input.pos.xy, 1, 1);

	float4 proj_inv = mul(o.pos, sk_proj_inv[ids.view]);
	o.norm = mul(float4(proj_inv.xyz, 0), transpose(sk_view[ids.view])).xyz;
	return o;
}

min16float4 ps(psIn input) : SV_TARGET {
	return sk_cubemap.Sample(sk_cubemap_s, input.norm);
}