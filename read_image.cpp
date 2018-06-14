#include "mview.h"
#include "FreeImage.h"

GrayImage ReadImageFromFile(std::string);

auto read_image(CameraParameter &cameraParameter) -> Image
{
  Image image;
  image.intrinsics = cameraParameter.intrinsics;
  image.extrinsics = cameraParameter.extrinsics;
  image.rgb_pixels = ReadImageFromFile(cameraParameter.filename);

  return image;
}

GrayImage ReadImageFromFile(std::string &filename)
{
  FreeImage_Initialise();

  //image format
  FREE_IMAGE_FORMAT fif = FIF_UNKNOWN;
  //pointer to the image, once loaded
  FIBITMAP *dib(0);

  //check the file signature and deduce its format
  fif = FreeImage_GetFileType(filename.c_str(), 0);
  if (fif == FIF_UNKNOWN)
    fif = FreeImage_GetFIFFromFilename(filename.c_str());
  if (fif == FIF_UNKNOWN)
    std::runtime_error("Data image format can not be determined");

  //check that the plugin has reading capabilities and load the file
  if (FreeImage_FIFSupportsReading(fif))
    dib = FreeImage_Load(fif, filename.c_str());
  if (!dib)
    std::runtime_error("Image could not be read");

  // Convert to RGBA float images
  FIBITMAP *hOldImage = dib;
  dib = FreeImage_ConvertToRGBAF(hOldImage); // ==> 4 channels
  FreeImage_Unload(hOldImage);

  //get the image width and height
  w = FreeImage_GetWidth(dib);
  h = FreeImage_GetHeight(dib);

  // rescale to fit width and height
  if (width != 0 && height != 0)
  {
    FIBITMAP *hOldImage = dib;
    dib = FreeImage_Rescale(hOldImage, width, height, FILTER_CATMULLROM);
    FreeImage_Unload(hOldImage);
    w = width;
    h = height;
  }

  //retrieve the image data
  BYTE *bits = FreeImage_GetBits(dib);

  //if this somehow one of these failed (they shouldn't), return failure
  if ((bits == 0) || (w == 0) || (h == 0))
    std::runtime_error("Invalid image");

  nChannels = 4;

  // copy image data
  float* data = new float[nChannels * w * h];

  // flip
  for (int y = 0; y < (int)h; ++y)
  {
    memcpy(&(data[y * nChannels * w]), &bits[sizeof(float) * (h - 1 - y) * nChannels * w], sizeof(float) * nChannels * w);
  }

  //Free FreeImage's copy of the data
  FreeImage_Unload(dib);

  return data;
}
