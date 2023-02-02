#version 330

layout (location = 0) in vec3 inVertex;
layout (location = 1) in vec3 inNorm;
layout (location = 2) in vec2 inUV;

out vec3 worldPos, worldNorm;
out vec2 texCoord;

uniform mat4 uModel, uProj, uView, uModelInvTr;


void main()
{
    worldPos = (uModel * vec4(inVertex, 1.0)).xyz;
    worldNorm = (uModelInvTr * vec4(inNorm, 1.0)).xyz;
    texCoord = inUV;
	gl_Position = uProj * uView * vec4(worldPos, 1.0);
}