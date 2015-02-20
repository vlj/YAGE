#include "shared_object.hpp"


GLuint SharedObject::FullScreenQuadVAO = 0;
GLuint SharedObject::UIVAO = 0;
GLuint SharedObject::billboardvbo = 0;
GLuint SharedObject::skytrivbo = 0;
GLuint SharedObject::frustrumvbo = 0;
GLuint SharedObject::frustrumindexes = 0;
GLuint SharedObject::ParticleQuadVBO = 0;
GLuint SharedObject::ViewProjectionMatrixesUBO;
GLuint SharedObject::LightingDataUBO;
GLuint SharedObject::quad_buffer;
GLuint SharedObject::quad_vbo;

static void initQuadVBO()
{
    GLuint tri_vbo;

    const float quad_vertex[] = {
        -1., -1., 0., 0., // UpperLeft
        -1., 1., 0., 1., // LowerLeft
        1., -1., 1., 0., // UpperRight
        1., 1., 1., 1., // LowerRight
    };
    glGenBuffers(1, &SharedObject::quad_vbo);
    glBindBuffer(GL_ARRAY_BUFFER, SharedObject::quad_vbo);
    glBufferData(GL_ARRAY_BUFFER, 16 * sizeof(float), quad_vertex, GL_STATIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    const float tri_vertex[] = {
        -1., -1.,
        -1., 3.,
        3., -1.,
    };
    glGenBuffers(1, &tri_vbo);
    glBindBuffer(GL_ARRAY_BUFFER, tri_vbo);
    glBufferData(GL_ARRAY_BUFFER, 6 * sizeof(float), tri_vertex, GL_STATIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    glGenVertexArrays(1, &SharedObject::FullScreenQuadVAO);
    glBindVertexArray(SharedObject::FullScreenQuadVAO);
    glBindBuffer(GL_ARRAY_BUFFER, tri_vbo);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), 0);
    glBindVertexArray(0);
}

static void initQuadBuffer()
{
    const float quad_vertex[] = {
        -1., -1., -1., 1., // UpperLeft
        -1., 1., -1., -1., // LowerLeft
        1., -1., 1., 1., // UpperRight
        1., 1., 1., -1., // LowerRight
    };
    glGenBuffers(1, &SharedObject::quad_buffer);
    glBindBuffer(GL_ARRAY_BUFFER, SharedObject::quad_buffer);
    glBufferData(GL_ARRAY_BUFFER, 16 * sizeof(float), quad_vertex, GL_STATIC_DRAW);

    glGenVertexArrays(1, &SharedObject::UIVAO);
    glBindVertexArray(SharedObject::UIVAO);
    glEnableVertexAttribArray(0);
    glEnableVertexAttribArray(3);
    glBindBuffer(GL_ARRAY_BUFFER, SharedObject::quad_buffer);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), 0);
    glVertexAttribPointer(3, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (GLvoid *)(2 * sizeof(float)));
    glBindVertexArray(0);
}

static void initBillboardVBO()
{
    float quad[] = {
        -.5, -.5, 0., 1.,
        -.5, .5, 0., 0.,
        .5, -.5, 1., 1.,
        .5, .5, 1., 0.,
    };
    glGenBuffers(1, &(SharedObject::billboardvbo));
    glBindBuffer(GL_ARRAY_BUFFER, SharedObject::billboardvbo);
    glBufferData(GL_ARRAY_BUFFER, 16 * sizeof(float), quad, GL_STATIC_DRAW);
}



static void initSkyTriVBO()
{
    const float tri_vertex[] = {
        -1., -1., 1.,
        -1., 3., 1.,
        3., -1., 1.,
    };

    glGenBuffers(1, &SharedObject::skytrivbo);
    glBindBuffer(GL_ARRAY_BUFFER, SharedObject::skytrivbo);
    glBufferData(GL_ARRAY_BUFFER, 3 * 3 * sizeof(float), tri_vertex, GL_STATIC_DRAW);
}

static void initFrustrumVBO()
{
    glGenBuffers(1, &SharedObject::frustrumvbo);
    glBindBuffer(GL_ARRAY_BUFFER, SharedObject::frustrumvbo);
    glBufferData(GL_ARRAY_BUFFER, 8 * 3 * sizeof(float), 0, GL_DYNAMIC_DRAW);

    int indices[24] = {
        0, 1, 1, 3, 3, 2, 2, 0,
        4, 5, 5, 7, 7, 6, 6, 4,
        0, 4, 1, 5, 2, 6, 3, 7,
    };

    glGenBuffers(1, &SharedObject::frustrumindexes);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, SharedObject::frustrumindexes);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, 12 * 2 * sizeof(int), indices, GL_STATIC_DRAW);
}


static void initShadowVPMUBO()
{
    glGenBuffers(1, &SharedObject::ViewProjectionMatrixesUBO);
    glBindBuffer(GL_UNIFORM_BUFFER, SharedObject::ViewProjectionMatrixesUBO);
    glBufferData(GL_UNIFORM_BUFFER, (16 * 9 + 2) * sizeof(float), 0, GL_STREAM_DRAW);
    glBindBuffer(GL_UNIFORM_BUFFER, 0);
}

static void initLightingDataUBO()
{
    glGenBuffers(1, &SharedObject::LightingDataUBO);
    glBindBuffer(GL_UNIFORM_BUFFER, SharedObject::LightingDataUBO);
    glBufferData(GL_UNIFORM_BUFFER, 40 * sizeof(float), 0, GL_STREAM_DRAW);
    glBindBuffer(GL_UNIFORM_BUFFER, 0);
}


static void initParticleQuadVBO()
{
    static const GLfloat quad_vertex[] = {
        -.5, -.5, 0., 0.,
        .5, -.5, 1., 0.,
        -.5, .5, 0., 1.,
        .5, .5, 1., 1.,
    };
    glGenBuffers(1, &SharedObject::ParticleQuadVBO);
    glBindBuffer(GL_ARRAY_BUFFER, SharedObject::ParticleQuadVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(quad_vertex), quad_vertex, GL_STATIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void SharedObject::init()
{
    initQuadVBO();
    initQuadBuffer();
    initBillboardVBO();
    initSkyTriVBO();
    initFrustrumVBO();
    initShadowVPMUBO();
    initLightingDataUBO();
    initParticleQuadVBO();
}