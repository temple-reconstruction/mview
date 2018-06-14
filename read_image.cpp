#include "mview.h"
#include "FreeImage.h"

auto ReadImageFromFile(std::string);

auto read_image(CameraParameter &cameraParameter) -> Image
{
  Image image;
  image.intrinsics = cameraParameter.intrinsics;
  image.extrinsics = cameraParameter.extrinsics;
  image.pixel_values = ReadImageFromFile(CameraParameter.filename);

  return image;
}

auto ReadImageFromFile(std::string &filename) -> Eigen::MatrixXf
{
  FreeImage_Initialise();
  if (data != nullptr)
    delete[] data;

  //image format
  FREE_IMAGE_FORMAT fif = FIF_UNKNOWN;
  //pointer to the image, once loaded
  FIBITMAP *dib(0);

  //check the file signature and deduce its format
  fif = FreeImage_GetFileType(filename.c_str(), 0);
  if (fif == FIF_UNKNOWN)
    fif = FreeImage_GetFIFFromFilename(filename.c_str());
  if (fif == FIF_UNKNOWN)
    return false;

  //check that the plugin has reading capabilities and load the file
  if (FreeImage_FIFSupportsReading(fif))
    dib = FreeImage_Load(fif, filename.c_str());
  if (!dib)
    return false;

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
    return false;

  nChannels = 4;

  // copy image data
  data = new float[nChannels * w * h];

  // flip
  for (int y = 0; y < (int)h; ++y)
  {
    memcpy(&(data[y * nChannels * w]), &bits[sizeof(float) * (h - 1 - y) * nChannels * w], sizeof(float) * nChannels * w);
  }
  //memcpy(data, bits, sizeof(float) * nChannels * w * h);

  //Free FreeImage's copy of the data
  FreeImage_Unload(dib);

  return data;
}