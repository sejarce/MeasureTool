#version 150

in vec4 oUv;

out vec4 oFragColor;

void main()
{
	oFragColor = vec4(oUv.st, 0, 1);
}
