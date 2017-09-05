#version 150

uniform sampler2D diffuseTex;
uniform sampler2D baseMap;
uniform sampler2D blurMap;

uniform float inv_width;
uniform float inv_height;

in vec4 oUv;

out vec4 oFragColor;

const vec3 lum_rgb = vec3(0.3,0.59,0.11);

// radial blur
vec4 radial(sampler2D tex, sampler2D bgtex, vec2 texcoord, int samples, float startScale, float scaleMul)
{
    vec4 c = vec4(0.0,0.0,0.0,0.0);
    float scale = startScale;
    for(int i=0; i<samples; i++) {
        vec2 uv = ((texcoord-0.5)*scale)+0.5;
        vec4 s = texture2D(tex, uv);
		uv = ((vec2(texcoord.x, 1.0 - texcoord.y) -0.5)*scale)+0.5;
		s += texture2D(bgtex, uv);
        c += s;
        scale *= scaleMul;
    }
    c /= float(samples);
    return c;
}

vec3 ACESToneMapping(vec3 color, float adapted_lum)
{
	const float A = 2.51f;
	const float B = 0.03f;
	const float C = 2.43f;
	const float D = 0.59f;
	const float E = 0.14f;

	color *= adapted_lum;
	return (color * (A * color + B)) / (color * (C * color + D) + E);
}

void main()
{
	vec4 baseColor = texture2D(baseMap, oUv.xy);
	vec4 blur = texture2D(blurMap, vec2(oUv.x, 1.0 - oUv.y));
	int samples = 10;
	vec4 effect = radial(blurMap, diffuseTex, vec2(oUv.x, 1.0 - oUv.y), samples, 1.0, 0.999);
	
	float blurAmount = 0.5;
	float effectAmount = 1.0;
	//vec4 color = mix(baseColor, blur, blurAmount);
	//color += effect*effectAmount;
	
	float lum = dot(baseColor.rgb, lum_rgb);
	vec4 color;
	if (lum > 0.01)
		color = baseColor;
	else
		color = effect*effectAmount;
	
	oFragColor = color;//vec4(ACESToneMapping(color.rgb, 0.5), 1.0);
}
