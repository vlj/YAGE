#ifndef SHARED_OBJECT_HPP
#define SHARED_OBJECT_HPP

#include "gl_headers.hpp"

class SharedObject
{
public:
    static GLuint billboardvbo;
    static GLuint skytrivbo, frustrumvbo, frustrumindexes, ParticleQuadVBO;
    static GLuint quad_vbo, quad_buffer;
    static GLuint ViewProjectionMatrixesUBO, LightingDataUBO;
    static GLuint FullScreenQuadVAO;
    static GLuint UIVAO;
    static void init();
};

#endif