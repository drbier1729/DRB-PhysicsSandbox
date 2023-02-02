#version 330

in vec3 worldPos, worldNorm;
in vec2 texCoord;

out vec4 fragColor;

// Material Color
uniform vec3 uDiffuse, uSpecular;
uniform float uGlossiness;
uniform float uAlpha;

// Light 
uniform vec3 uLightDir, uLightColor, uViewPos;


void main()
{
	vec3 N = normalize(worldNorm);
    vec3 L = normalize(uLightDir);
    vec3 V = normalize(uViewPos - worldPos);
    vec3 H = normalize(L + V);

    float HN = max(dot(H, N), 0.0);
    float LH = max(dot(L, H), 0.0);

    vec3 Kd = uDiffuse;       
    vec3 Ks = uSpecular;
    float alpha = uGlossiness;

    vec3 Ii = uLightColor;
    vec3 Ia = vec3(0.1);
    float LN = max(dot(L, N), 0.0);

	vec3 ambient = vec3(0.1) * Kd;

    float diff = LN;
	vec3 diffuse = (diff * Kd) * uLightColor;
    float spec = pow(HN, alpha);
	vec3 specular = (Ks * spec) * uLightColor; 

	fragColor =  vec4((ambient + diffuse + specular), uAlpha);
}