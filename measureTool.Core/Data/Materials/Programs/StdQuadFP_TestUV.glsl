#version 150

in vec4 oUv;

out vec4 oFragColor;

void main()
{
	oFragColor = vec4(oUv.xy, 0, 1);
}
