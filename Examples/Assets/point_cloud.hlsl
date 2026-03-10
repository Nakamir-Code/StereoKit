#include <stereokit.hlsli>

//--name = app/point_cloud

//--point_size = 0.01
float point_size;
//--screen_size = 0
float screen_size;

struct vsIn {
	float4 pos  : SV_POSITION;
	float3 norm : NORMAL0;
	float2 off  : TEXCOORD0;
	float4 color: COLOR0;
};
struct psIn {
	float4 pos   : SV_POSITION;
	float2 uv    : TEXCOORD0;
	float4 color : COLOR0;
};

psIn vs(vsIn input, sk_ids_t ids) {
	psIn o;

	float4 world = mul(input.pos, sk_inst[ids.inst].world);
	float4 view  = mul(world, sk_view[ids.view]);
	if (screen_size <= 0.1)
		view.xy = point_size * input.off + view.xy;
	o.pos        = mul(view, sk_proj[ids.view]);
	o.uv         = input.off + 0.5;
	o.color      = input.color;

	if (screen_size > 0.1) {
		float  aspect = sk_proj[ids.view]._m11 / sk_proj[ids.view]._m00;
		o.pos.xy = ( point_size * input.off / float2(aspect,1) ) *o.pos.w + o.pos.xy;
	}

	return o;
}
float4 ps(psIn input) : SV_TARGET{
	return input.color;
}