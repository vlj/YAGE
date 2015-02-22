//  SuperTuxKart - a fun racing game with go-kart
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the GNU General Public License
//  as published by the Free Software Foundation; either version 3
//  of the License, or (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program; if not, write to the Free Software
//  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.

#ifndef HEADER_RTTS_HPP
#define HEADER_RTTS_HPP


#include "utils/ptr_vector.hpp"
#include "utils/leak_check.hpp"

class FrameBuffer;


enum TypeFBO
{
    FBO_SSAO,
    FBO_NORMAL_AND_DEPTHS,
    FBO_GBUFFERS,
    FBO_COLORS,
    FBO_MLAA_COLORS,
    FBO_MLAA_BLEND,
    FBO_MLAA_TMP,
    FBO_TMP1_WITH_DS,
    FBO_TMP2_WITH_DS,
    FBO_TMP4,
    FBO_LINEAR_DEPTH,
    FBO_HALF1,
    FBO_HALF1_R,
    FBO_HALF2,
    FBO_HALF2_R,
    FBO_QUARTER1,
    FBO_QUARTER2,
    FBO_EIGHTH1,
    FBO_EIGHTH2,
    FBO_DISPLACE,
    FBO_BLOOM_1024,
    FBO_SCALAR_1024,
    FBO_BLOOM_512,
    FBO_TMP_512,
    FBO_LENS_512,

    FBO_BLOOM_256,
    FBO_TMP_256,
    FBO_LENS_256,

    FBO_BLOOM_128,
    FBO_TMP_128,
    FBO_LENS_128,
    FBO_COUNT
};

enum TypeRTT
{
    RTT_TMP1 = 0,
    RTT_TMP2,
    RTT_TMP3,
    RTT_TMP4,
    RTT_LINEAR_DEPTH,
    RTT_NORMAL_AND_DEPTH,
    RTT_BASE_COLOR,
    RTT_EMIT_VALUE,
    RTT_COLOR,


    RTT_HALF1,
    RTT_HALF2,
    RTT_HALF1_R,
    RTT_HALF2_R,

    RTT_QUARTER1,
    RTT_QUARTER2,
    //    RTT_QUARTER3,
    //    RTT_QUARTER4,

    RTT_EIGHTH1,
    RTT_EIGHTH2,

    //    RTT_SIXTEENTH1,
    //    RTT_SIXTEENTH2,

    RTT_SSAO,

    //    RTT_COLLAPSE,
    //    RTT_COLLAPSEH,
    //    RTT_COLLAPSEV,
    //    RTT_COLLAPSEH2,
    //    RTT_COLLAPSEV2,
    //    RTT_WARPH,
    //    RTT_WARPV,

    //    RTT_HALF_SOFT,

    RTT_DISPLACE,
    RTT_MLAA_COLORS,
    RTT_MLAA_BLEND,
    RTT_MLAA_TMP,

    RTT_BLOOM_1024,
    RTT_SCALAR_1024,
    RTT_BLOOM_512,
    RTT_TMP_512,
    RTT_LENS_512,
    RTT_BLOOM_256,
    RTT_TMP_256,
    RTT_LENS_256,
    RTT_BLOOM_128,
    RTT_TMP_128,
    RTT_LENS_128,

    RTT_COUNT
};

namespace irr {
    namespace scene {
        class ICameraSceneNode;
    }
};



class RTT
{
public:
    RTT(unsigned, unsigned);
    ~RTT();

    FrameBuffer &getShadowFBO() { return *m_shadow_FBO; }
    FrameBuffer &getRH() { return *m_RH_FBO; }
    FrameBuffer &getRSM() { return *m_RSM; }

    unsigned getDepthStencilTexture() const { return DepthStencilTexture; }
    unsigned getRenderTarget(enum TypeRTT target) const { return RenderTargetTextures[target]; }
    FrameBuffer& getFBO(enum TypeFBO fbo) { return FrameBuffers[fbo]; }

    FrameBuffer* render(irr::scene::ICameraSceneNode* camera, float dt);

private:
    unsigned RenderTargetTextures[RTT_COUNT];
    PtrVector<FrameBuffer> FrameBuffers;
    unsigned DepthStencilTexture;

    int m_width;
    int m_height;

    unsigned shadowColorTex, shadowNormalTex, shadowDepthTex;
    unsigned RSM_Color, RSM_Normal, RSM_Depth;
    unsigned RH_Red, RH_Green, RH_Blue;
    FrameBuffer* m_shadow_FBO, *m_RSM, *m_RH_FBO;

    LEAK_CHECK();
};

#endif

