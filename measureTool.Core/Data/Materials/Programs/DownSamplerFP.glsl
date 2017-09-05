#version 150

uniform sampler2D diffuseTex;
uniform float inv_width;
uniform float inv_height;

in vec4 oUv;

out vec4 oFragColor;

const vec3 lum_rgb = vec3(0.3,0.59,0.11);

void main()
{
	float dx = 0.5*inv_width;
    float dy = 0.5*inv_height;	
	
	vec4 color = 	texture2D(diffuseTex,oUv.xy + vec2(-dx*3.0,	-dy*3.0));	//(-3,-3)
	color += 		texture2D(diffuseTex,oUv.xy + vec2(-dx,		-dy*3.0));	//(-1,-3)
	color += 		texture2D(diffuseTex,oUv.xy + vec2(dx,		-dy*3.0));	//(1,-3)
	color += 		texture2D(diffuseTex,oUv.xy + vec2(dx*3.0,	-dy*3.0));	//(3,-3)
	color += 		texture2D(diffuseTex,oUv.xy + vec2(-dx*3.0,	-dy));		//(-3,-1)
	color += 		texture2D(diffuseTex,oUv.xy + vec2(-dx,		-dy));		//(-1,-1)
	color += 		texture2D(diffuseTex,oUv.xy + vec2(dx,		-dy));		//(1,-1)
	color += 		texture2D(diffuseTex,oUv.xy + vec2(dx*3.0,	-dy));		//(3,-1)
	color += 		texture2D(diffuseTex,oUv.xy + vec2(-dx*3.0,	dy));		//(-3,1)
	color += 		texture2D(diffuseTex,oUv.xy + vec2(-dx,		dy));		//(-1,1)
	color += 		texture2D(diffuseTex,oUv.xy + vec2(dx,		dy));		//(1,1)
	color += 		texture2D(diffuseTex,oUv.xy + vec2(dx*3.0,	dy));		//(3,1)
	color += 		texture2D(diffuseTex,oUv.xy + vec2(-dx*3.0,	dy*3.0));	//(-3,3)
	color += 		texture2D(diffuseTex,oUv.xy + vec2(-dx,		dy*3.0));	//(-1,3)
	color += 		texture2D(diffuseTex,oUv.xy + vec2(dx,		dy*3.0));	//(1,3)
	color += 		texture2D(diffuseTex,oUv.xy + vec2(dx*3.0,	dy*3.0));	//(3,3)
	
	vec3 clr = color.rgb*0.0625;
	
	float lum = dot(clr, lum_rgb);
	
	if (lum > 0.01)
		oFragColor = vec4(0.0,0.2,0.2,1.0);
	else
		discard;
}