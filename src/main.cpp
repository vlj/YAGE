#include <IrrlichtDevice.h>
#include <irrlicht.h>

int main(int argc, char *argv[] )
{
    irr::IrrlichtDevice* device = irr::createDevice(irr::video::EDT_OPENGL,
        irr::core::dimension2d<irr::u32>(640, 480),
        /*bits*/16U, /**fullscreen*/ false,
        /*stencilBuffer*/ false,
        /*vsync*/false,
        /*event receiver*/ NULL, NULL);
//        file_manager->getFileSystem());
    return 0;
}   // main
