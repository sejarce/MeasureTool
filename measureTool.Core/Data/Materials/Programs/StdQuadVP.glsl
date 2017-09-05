#version 150

in vec4 vertex;
in vec2 uv0;

out vec4 oUv;

void main()                    
{
	gl_Position = vertex;
	oUv = vec4(uv0.xy, 0, 1);
	
#ifdef	FLIP_V
	oUv.y = 1 - oUv.y;
#endif
}