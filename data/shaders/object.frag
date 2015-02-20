#ifdef Use_Bindless_Texture
layout(bindless_sampler) uniform sampler2D tex;
layout(bindless_sampler) uniform sampler2D glosstex;
#else
uniform sampler2D tex;
uniform sampler2D glosstex;
#endif

in vec3 nor;
in vec2 uv;
in vec4 color;

layout(location = 0) out vec4 EncodedNormal_Roughness_Metalness;
layout(location = 1) out vec4 Colors;
layout(location = 2) out float EmitMap;

vec2 EncodeNormal(vec3 n);

void main(void)
{
    Colors = vec4(texture(tex, uv).rgb * pow(color.rgb, vec3(2.2)), 1.);
    float glossmap = texture(glosstex, uv).r;
    float reflectance = texture(glosstex, uv).g;
    EncodedNormal_Roughness_Metalness.xy = 0.5 * EncodeNormal(normalize(nor)) + 0.5;
    EncodedNormal_Roughness_Metalness.z = 1. - glossmap;
    EncodedNormal_Roughness_Metalness.w = reflectance;
    EmitMap = texture(glosstex, uv).b;
}
