#include "vaomanager.hpp"
#include "irr_driver.hpp"
#include "stkmesh.hpp"
#include "glwrap.hpp"
#include "central_settings.hpp"

VAOManager::VAOManager()
{
    // Init all instance buffer
    InstanceBuffer<InstanceDataDualTex>::getInstance();
    InstanceBuffer<InstanceDataThreeTex>::getInstance();
    InstanceBuffer<InstanceDataShadow>::getInstance();
    InstanceBuffer<InstanceDataRSM>::getInstance();
    InstanceBuffer<GlowInstanceData>::getInstance();

}

VAOManager::~VAOManager()
{
    InstanceBuffer<InstanceDataDualTex>::getInstance()->kill();
    InstanceBuffer<InstanceDataThreeTex>::getInstance()->kill();
    InstanceBuffer<InstanceDataShadow>::getInstance()->kill();
    InstanceBuffer<InstanceDataRSM>::getInstance()->kill();
    InstanceBuffer<GlowInstanceData>::getInstance()->kill();

    VertexArrayObject<FormattedVertexStorage<video::S3DVertex, SkinnedVertexData> >::getInstance()->kill();
    VertexArrayObject<FormattedVertexStorage<video::S3DVertex> >::getInstance()->kill();
    VertexArrayObject<FormattedVertexStorage<video::S3DVertex2TCoords> >::getInstance()->kill();
    VertexArrayObject<FormattedVertexStorage<video::S3DVertexTangents> >::getInstance()->kill();
}


template<>
struct VertexAttribBinder <u16>
{
public:
    static void bind()
    {}
};

template<>
struct VertexAttribBinder<video::S3DVertex>
{
public:
    static void bind()
    {
        // Position
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(video::S3DVertex), 0);
        // Normal
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(video::S3DVertex), (GLvoid*)12);
        // Color
        glEnableVertexAttribArray(2);
        glVertexAttribPointer(2, 4, GL_UNSIGNED_BYTE, GL_TRUE, sizeof(video::S3DVertex), (GLvoid*)24);
        // Texcoord
        glEnableVertexAttribArray(3);
        glVertexAttribPointer(3, 2, GL_FLOAT, GL_FALSE, sizeof(video::S3DVertex), (GLvoid*)28);
    }
};

template<>
struct VertexAttribBinder<video::S3DVertex2TCoords>
{
public:
    static void bind()
    {
        // Position
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(video::S3DVertex2TCoords), 0);
        // Normal
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(video::S3DVertex2TCoords), (GLvoid*)12);
        // Color
        glEnableVertexAttribArray(2);
        glVertexAttribPointer(2, 4, GL_UNSIGNED_BYTE, GL_TRUE, sizeof(video::S3DVertex2TCoords), (GLvoid*)24);
        // Texcoord
        glEnableVertexAttribArray(3);
        glVertexAttribPointer(3, 2, GL_FLOAT, GL_FALSE, sizeof(video::S3DVertex2TCoords), (GLvoid*)28);
        // SecondTexcoord
        glEnableVertexAttribArray(4);
        glVertexAttribPointer(4, 2, GL_FLOAT, GL_FALSE, sizeof(video::S3DVertex2TCoords), (GLvoid*)36);
    }
};

template<>
struct VertexAttribBinder<video::S3DVertexTangents>
{
public:
    static void bind()
    {
        // Position
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(video::S3DVertexTangents), 0);
        // Normal
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(video::S3DVertexTangents), (GLvoid*)12);
        // Color
        glEnableVertexAttribArray(2);
        glVertexAttribPointer(2, 4, GL_UNSIGNED_BYTE, GL_TRUE, sizeof(video::S3DVertexTangents), (GLvoid*)24);
        // Texcoord
        glEnableVertexAttribArray(3);
        glVertexAttribPointer(3, 2, GL_FLOAT, GL_FALSE, sizeof(video::S3DVertexTangents), (GLvoid*)28);
        // Tangent
        glEnableVertexAttribArray(5);
        glVertexAttribPointer(5, 3, GL_FLOAT, GL_FALSE, sizeof(video::S3DVertexTangents), (GLvoid*)36);
        // Bitangent
        glEnableVertexAttribArray(6);
        glVertexAttribPointer(6, 3, GL_FLOAT, GL_FALSE, sizeof(video::S3DVertexTangents), (GLvoid*)48);
    }
};

template<>
struct VertexAttribBinder <struct SkinnedVertexData >
{
public:
    static void bind()
    {
        glEnableVertexAttribArray(7);
        glVertexAttribIPointer(7, 1, GL_INT, 4 * 2 * sizeof(float), 0);
        glEnableVertexAttribArray(8);
        glVertexAttribPointer(8, 1, GL_FLOAT, GL_FALSE, 4 * 2 * sizeof(float), (GLvoid*)(sizeof(int)));
        glEnableVertexAttribArray(9);
        glVertexAttribIPointer(9, 1, GL_INT, 4 * 2 * sizeof(float), (GLvoid*)(sizeof(float) + sizeof(int)));
        glEnableVertexAttribArray(10);
        glVertexAttribPointer(10, 1, GL_FLOAT, GL_FALSE, 4 * 2 * sizeof(float), (GLvoid*)(sizeof(float) + 2 * sizeof(int)));
        glEnableVertexAttribArray(11);
        glVertexAttribIPointer(11, 1, GL_INT, 4 * 2 * sizeof(float), (GLvoid*)(2 * sizeof(float) + 2 * sizeof(int)));
        glEnableVertexAttribArray(12);
        glVertexAttribPointer(12, 1, GL_FLOAT, GL_FALSE, 4 * 2 * sizeof(float), (GLvoid*)(2 * sizeof(float) + 3 * sizeof(int)));
        glEnableVertexAttribArray(13);
        glVertexAttribIPointer(13, 1, GL_INT, 4 * 2 * sizeof(float), (GLvoid*)(3 * sizeof(float) + 3 * sizeof(int)));
        glEnableVertexAttribArray(14);
        glVertexAttribPointer(14, 1, GL_FLOAT, GL_FALSE, 4 * 2 * sizeof(float), (GLvoid*)(3 * sizeof(float) + 4 * sizeof(int)));
    }
};

std::pair<unsigned, unsigned> VAOManager::getBase(scene::IMeshBuffer *mb, void *extraVertexInfo)
{
    if (extraVertexInfo != nullptr)
        return VertexArrayObject<FormattedVertexStorage<video::S3DVertex, SkinnedVertexData> >::getInstance()->getBase(mb, extraVertexInfo);
    switch (mb->getVertexType())
    {
    default:
        assert(0 && "Wrong type");
    case video::EVT_STANDARD:
        return VertexArrayObject<FormattedVertexStorage<video::S3DVertex> >::getInstance()->getBase(mb);
    case video::EVT_2TCOORDS:
        return VertexArrayObject<FormattedVertexStorage<video::S3DVertex2TCoords> >::getInstance()->getBase(mb);
    case video::EVT_TANGENTS:
        return VertexArrayObject<FormattedVertexStorage<video::S3DVertexTangents> >::getInstance()->getBase(mb);
    }
}

GLuint VAOManager::getInstanceBuffer(InstanceType it)
{
    switch (it)
    {
    default:
        assert(0 && "wrong instance type");
    case InstanceTypeDualTex:
        return InstanceBuffer<InstanceDataDualTex>::getInstance()->getBuffer(0);
    case InstanceTypeThreeTex:
        return InstanceBuffer<InstanceDataThreeTex>::getInstance()->getBuffer(0);
    case InstanceTypeShadow:
        return InstanceBuffer<InstanceDataShadow>::getInstance()->getBuffer(0);
    case InstanceTypeRSM:
        return InstanceBuffer<InstanceDataRSM>::getInstance()->getBuffer(0);
    case InstanceTypeGlow:
        return InstanceBuffer<GlowInstanceData>::getInstance()->getBuffer(0);
    }
}

void *VAOManager::getInstanceBufferPtr(InstanceType it)
{
    switch (it)
    {
    default:
        assert(0 && "wrong instance type");
    case InstanceTypeDualTex:
        return InstanceBuffer<InstanceDataDualTex>::getInstance()->getPointer();
    case InstanceTypeThreeTex:
        return InstanceBuffer<InstanceDataThreeTex>::getInstance()->getPointer();
    case InstanceTypeShadow:
        return InstanceBuffer<InstanceDataShadow>::getInstance()->getPointer();
    case InstanceTypeRSM:
        return InstanceBuffer<InstanceDataRSM>::getInstance()->getPointer();
    case InstanceTypeGlow:
        return InstanceBuffer<GlowInstanceData>::getInstance()->getPointer();
    }
}

unsigned VAOManager::getVAO(irr::video::E_VERTEX_TYPE type, bool skinned)
{
    if (skinned)
        return VertexArrayObject<FormattedVertexStorage<video::S3DVertex, SkinnedVertexData> >::getInstance()->getVAO();
    switch (type)
    {
    default:
        assert(0 && "Wrong type");
    case video::EVT_STANDARD:
        return VertexArrayObject<FormattedVertexStorage<video::S3DVertex> >::getInstance()->getVAO();
    case video::EVT_2TCOORDS:
        return VertexArrayObject<FormattedVertexStorage<video::S3DVertex2TCoords> >::getInstance()->getVAO();
    case video::EVT_TANGENTS:
        return VertexArrayObject<FormattedVertexStorage<video::S3DVertexTangents> >::getInstance()->getVAO();
    }
}

unsigned VAOManager::getInstanceVAO(irr::video::E_VERTEX_TYPE vt, bool skinned, enum InstanceType it)
{
    if (skinned)
    {
        switch (it)
        {
        case InstanceTypeDualTex:
            return VertexArrayObject<FormattedVertexStorage<video::S3DVertex, SkinnedVertexData> >::getInstance()->vao_instanceDualTex;
        case InstanceTypeThreeTex:
            return VertexArrayObject<FormattedVertexStorage<video::S3DVertex, SkinnedVertexData> >::getInstance()->vao_instanceThreeTex;
        case InstanceTypeShadow:
            return VertexArrayObject<FormattedVertexStorage<video::S3DVertex, SkinnedVertexData> >::getInstance()->vao_instanceShadow;
        case InstanceTypeRSM:
            return VertexArrayObject<FormattedVertexStorage<video::S3DVertex, SkinnedVertexData> >::getInstance()->vao_instanceRSM;
        case InstanceTypeGlow:
            return VertexArrayObject<FormattedVertexStorage<video::S3DVertex, SkinnedVertexData> >::getInstance()->vao_instanceGlowData;
        }
    }
    switch (vt)
    {
    default:
        assert(0 && "Wrong type");
    case video::EVT_STANDARD:
        switch (it)
        {
        default:
            assert(0 && "wrong instance type");
        case InstanceTypeDualTex:
            return VertexArrayObject<FormattedVertexStorage<video::S3DVertex> >::getInstance()->vao_instanceDualTex;
        case InstanceTypeThreeTex:
            return VertexArrayObject<FormattedVertexStorage<video::S3DVertex> >::getInstance()->vao_instanceThreeTex;
        case InstanceTypeShadow:
            return VertexArrayObject<FormattedVertexStorage<video::S3DVertex> >::getInstance()->vao_instanceShadow;
        case InstanceTypeRSM:
            return VertexArrayObject<FormattedVertexStorage<video::S3DVertex> >::getInstance()->vao_instanceRSM;
        case InstanceTypeGlow:
            return VertexArrayObject<FormattedVertexStorage<video::S3DVertex> >::getInstance()->vao_instanceGlowData;
        }
        break;
    case video::EVT_2TCOORDS:
        switch (it)
        {
        default:
            assert(0 && "wrong instance type");
        case InstanceTypeDualTex:
            return VertexArrayObject<FormattedVertexStorage<video::S3DVertex2TCoords> >::getInstance()->vao_instanceDualTex;
        case InstanceTypeThreeTex:
            return VertexArrayObject<FormattedVertexStorage<video::S3DVertex2TCoords> >::getInstance()->vao_instanceThreeTex;
        case InstanceTypeShadow:
            return VertexArrayObject<FormattedVertexStorage<video::S3DVertex2TCoords> >::getInstance()->vao_instanceShadow;
        case InstanceTypeRSM:
            return VertexArrayObject<FormattedVertexStorage<video::S3DVertex2TCoords> >::getInstance()->vao_instanceRSM;
        case InstanceTypeGlow:
            return VertexArrayObject<FormattedVertexStorage<video::S3DVertex2TCoords> >::getInstance()->vao_instanceGlowData;
        }
        break;
    case video::EVT_TANGENTS:
        switch (it)
        {
        default:
            assert(0 && "wrong instance type");
        case InstanceTypeDualTex:
            return VertexArrayObject<FormattedVertexStorage<video::S3DVertexTangents> >::getInstance()->vao_instanceDualTex;
        case InstanceTypeThreeTex:
            return VertexArrayObject<FormattedVertexStorage<video::S3DVertexTangents> >::getInstance()->vao_instanceThreeTex;
        case InstanceTypeShadow:
            return VertexArrayObject<FormattedVertexStorage<video::S3DVertexTangents> >::getInstance()->vao_instanceShadow;
        case InstanceTypeRSM:
            return VertexArrayObject<FormattedVertexStorage<video::S3DVertexTangents> >::getInstance()->vao_instanceRSM;
        case InstanceTypeGlow:
            return VertexArrayObject<FormattedVertexStorage<video::S3DVertexTangents> >::getInstance()->vao_instanceGlowData;
        }
        break;
    }
}
