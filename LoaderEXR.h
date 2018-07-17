#pragma once

#include <OpenEXR/OpenEXRConfig.h>
#include <OpenEXR/ImfRgbaFile.h>
#include <OpenEXR/ImfInputFile.h>
#include <OpenEXR/ImfArray.h>

static inline void read_openexr(std::string filename, float *image, int width, int height, int n_channels) {
    namespace IMF = OPENEXR_IMF_NAMESPACE;
    IMF::InputFile file (filename.c_str());
    Imath::Box2i dw = file.header().dataWindow();
    int w = dw.max.x - dw.min.x + 1;
    int h = dw.max.y - dw.min.y + 1;
    assert(width == w);
    assert(height == h);
    IMF::FrameBuffer frameBuffer;
    // frameBuffer.insert("R", IMF::Slice(IMF::FLOAT, (char *) (image - dw.min.x - dw.min.y*width), sizeof(float)*3, sizeof(float)*3*width, 1, 1, 0.0f));
    // frameBuffer.insert("G", IMF::Slice(IMF::FLOAT, (char *) (image - dw.min.x - dw.min.y*width), sizeof(float)*3, sizeof(float)*3*width, 1, 1, 0.0f));
    // frameBuffer.insert("B", IMF::Slice(IMF::FLOAT, (char *) (image - dw.min.x - dw.min.y*width), sizeof(float)*3, sizeof(float)*3*width, 1, 1, 0.0f));
    frameBuffer.insert("Z", IMF::Slice(IMF::FLOAT, (char *) (image), sizeof(float)*1, sizeof(float)*1*width, 1, 1, 0.0f));
	file.setFrameBuffer(frameBuffer);
    file.readPixels(dw.min.y, dw.max.y);
 }


