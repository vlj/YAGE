#define MAX_JOINT_NUM 48

uniform mat4 ModelMatrix =
    mat4(1., 0., 0., 0.,
         0., 1., 0., 0.,
         0., 0., 1., 0.,
         0., 0., 0., 1.);
uniform int layer;

#if __VERSION__ >= 330
layout(location = 0) in vec3 Position;
layout(location = 3) in vec2 Texcoord;

layout(location = 7) in int index0;
layout(location = 8) in float weight0;
layout(location = 9) in int index1;
layout(location = 10) in float weight1;
layout(location = 11) in int index2;
layout(location = 12) in float weight2;
layout(location = 13) in int index3;
layout(location = 14) in float weight3;
#else
in vec3 Position;
in vec3 Normal;
in vec4 Color;
in vec2 Texcoord;
in vec2 SecondTexcoord;
in vec3 Tangent;
in vec3 Bitangent;

in int index0;
in float weight0;
in int index1;
in float weight1;
in int index2;
in float weight2;
in int index3;
in float weight3;
#endif

#ifdef VSLayer
out vec2 uv;
#else
out vec2 tc;
out int layerId;
#endif

uniform mat4 JointTransform[MAX_JOINT_NUM];

void main()
{
    vec4 IdlePosition = vec4(Position, 1.);
    vec4 SkinnedPosition = vec4(0.);

    vec4 SingleBoneInfluencedPosition;
    if (index0 >= 0)
    {
        SingleBoneInfluencedPosition = JointTransform[index0] * IdlePosition;
        SingleBoneInfluencedPosition /= SingleBoneInfluencedPosition.w;
    }
    else
    {
        SingleBoneInfluencedPosition = IdlePosition;
    }
    SkinnedPosition += weight0 * SingleBoneInfluencedPosition;

    if (index1 >= 0)
    {
        SingleBoneInfluencedPosition= JointTransform[index1] * IdlePosition;
        SingleBoneInfluencedPosition /= SingleBoneInfluencedPosition.w;
    }
    else
    {
        SingleBoneInfluencedPosition = IdlePosition;
    }
    SkinnedPosition += weight1 * SingleBoneInfluencedPosition;

    if (index2 >= 0)
    {
        SingleBoneInfluencedPosition = JointTransform[index2] * IdlePosition;
        SingleBoneInfluencedPosition /= SingleBoneInfluencedPosition.w;
    }
    else
    {
        SingleBoneInfluencedPosition = IdlePosition;
    }
    SkinnedPosition += weight2 * SingleBoneInfluencedPosition;

    if (index3 >= 0)
    {
        SingleBoneInfluencedPosition = JointTransform[index3] * IdlePosition;
        SingleBoneInfluencedPosition /= SingleBoneInfluencedPosition.w;
    }
    else
    {
        SingleBoneInfluencedPosition = IdlePosition;
    }
    SkinnedPosition += weight3 * SingleBoneInfluencedPosition;

#ifdef VSLayer
    gl_Layer = layer;
    uv = Texcoord;
    gl_Position = ShadowViewProjMatrixes[gl_Layer] * ModelMatrix * vec4(SkinnedPosition.xyz, 1.);
#else
    layerId = layer;
    tc = Texcoord;
    gl_Position = ShadowViewProjMatrixes[layerId] * ModelMatrix * vec4(SkinnedPosition.xyz, 1.);
#endif
}
