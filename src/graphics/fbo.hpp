#ifndef FBO_HPP
#define FBO_HPP

#include "gl_headers.hpp"
#include <utils/leak_check.hpp>
#include <vector>
#include <assert.h>

class FrameBuffer
{
private:
    GLuint fbo, fbolayer;
    std::vector<GLuint> RenderTargets;
    GLuint DepthTexture;
    size_t width, height;
public:
    FrameBuffer();
    FrameBuffer(const std::vector <GLuint> &RTTs, size_t w, size_t h, bool layered = false);
    FrameBuffer(const std::vector <GLuint> &RTTs, GLuint DS, size_t w, size_t h, bool layered = false);
    ~FrameBuffer();
    void Bind();
    void BindLayer(unsigned);
    const std::vector<GLuint> &getRTT() const { return RenderTargets; }
    GLuint &getDepthTexture() { assert(DepthTexture); return DepthTexture; }
    size_t getWidth() const { return width; }
    size_t getHeight() const { return height; }
    static void Blit(const FrameBuffer &Src, FrameBuffer &Dst, GLbitfield mask = GL_COLOR_BUFFER_BIT, GLenum filter = GL_NEAREST);
    void BlitToDefault(size_t, size_t, size_t, size_t);

    LEAK_CHECK();
};

#endif