#version 150

uniform mat4 worldMatrix;
uniform vec3 lightPosition;
uniform vec4 lightDiffuse;

in vec3 oNormal;
in vec3 oPosition;
in vec4 oColour;

out vec4 oFragColor;

void main()
{
	vec3 worldNormal = mat3(worldMatrix) * oNormal;
	vec3 lightDir = normalize(lightPosition - oPosition);
	float factor = max(0.0, dot(worldNormal,lightDir));
	
	oFragColor = oColour;
}
