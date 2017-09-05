#version 150

uniform sampler2D diffuseTex;

in vec4 oUv;

out vec4 oFragColor;

void main()
{
	oFragColor = texture(diffuseTex, oUv.st);
}
