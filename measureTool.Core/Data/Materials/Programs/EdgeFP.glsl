#version 150
uniform sampler2D diffuseTex;
//uniform vec2 inv_widthheight;
//uniform vec4 inv_whd1;
uniform float inv_width;
uniform float inv_height;

in vec4 oUv;

out vec4 oFragColor;


const vec3 lum_rgb = vec3(0.3,0.59,0.11);
const float threshold = 0.25;

void main()
{
	vec2 inv_widthheight = vec2(inv_width, inv_height);
	vec3 colorM = texture2D(diffuseTex, oUv.st).rgb;
	vec3 colorLT = texture2D(diffuseTex, oUv.st + vec2(-1.0,-1.0)*inv_widthheight).rgb;
	vec3 colorL = texture2D(diffuseTex, oUv.st + vec2(-1.0,0.0)*inv_widthheight).rgb;
	vec3 colorLB = texture2D(diffuseTex, oUv.st + vec2(-1.0,1.0)*inv_widthheight).rgb;
	vec3 colorT = texture2D(diffuseTex, oUv.st + vec2(0.0,-1.0)*inv_widthheight).rgb;
	vec3 colorB = texture2D(diffuseTex, oUv.st + vec2(0.0,1.0)*inv_widthheight).rgb;
	vec3 colorRT = texture2D(diffuseTex, oUv.st + vec2(1.0,-1.0)*inv_widthheight).rgb;
	vec3 colorR = texture2D(diffuseTex, oUv.st + vec2(1.0,0.0)*inv_widthheight).rgb;
	vec3 colorRB = texture2D(diffuseTex, oUv.st + vec2(1.0,1.0)*inv_widthheight).rgb;
	
	vec3 gx_color = (colorLB+colorB*2.0+colorRB) - (colorLT+colorT*2.0+colorRT);
	float gx = dot(gx_color, lum_rgb);
	
	vec3 gy_color = (colorRT+colorR*2.0+colorRB) - (colorLB+colorL*2.0+colorLB);
	float gy = dot(gy_color, lum_rgb);
	
	float g = sqrt(gx*gx + gy*gy);
	float lum = dot(colorM, lum_rgb);
	if (g > threshold)
	{
		oFragColor = vec4(0.0,0.2,0.2,1.0);//vec4(vec3(lum*(1.0 - g/1.414)), 1.0);//vec4(0.0,0.0,0.0,1.0);
	}else{
		oFragColor = vec4(0.0, 0.0, 0.0, 1.0);
	}
}