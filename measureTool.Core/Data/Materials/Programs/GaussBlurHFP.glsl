#version 150

uniform sampler2D diffuseTex;
//uniform float sigma;

uniform vec4 inv_texture_size;

in vec4 oUv;

out vec4 oFragColor;

//const float offset[17] = float[](-8.0, -7.0, -6.0, -5.0, -4.0, -3.0, -2.0, -1.0, 0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0);

float gaussian(float x,float s)
{
	return exp(-x*x/(2.0*s*s));
}

void main()
{
	vec2 texCrood = vec2(oUv.x, 1.0-oUv.y);
	vec4 color[17];
	for (int i = 0; i < 17; i++)
	{
		float offset = float(i)-8.0;
		color[i] = texture2D(diffuseTex, texCrood + vec2(offset*inv_texture_size.x, 0.0));
	}

	float factor[17];
	float sum = 0.0;
	const float sigma = 1.0;
	for(int i = 0;i < 17;i++)
	{
		float offset = float(i)-8.0;
		factor[i] = gaussian(offset,sigma);
		sum += factor[i];
	}
	
	vec4 clr = vec4(0.0);
	for(int i = 0;i < 17;i++)
	{
		float w = factor[i]/sum;
		clr += w*color[i];
	}
	oFragColor = clr;
}
