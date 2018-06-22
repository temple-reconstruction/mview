#include "mview.h"
#include "FreeImage.h"

static std::pair<GrayImage, RgbImage> ReadImageFromFile(std::string);

auto read_image(CameraParameter cameraParameter) -> Image
{
  Image image;
  image.intrinsics = cameraParameter.intrinsics;
  image.extrinsics = cameraParameter.extrinsics;
  std::tie(image.gray_pixels, image.rgb_pixels) = ReadImageFromFile(cameraParameter.filename);

  return image;
}

std::pair<GrayImage, RgbImage> ReadImageFromFile(std::string filename)
{
  FreeImage_Initialise();

  //image format
  FREE_IMAGE_FORMAT fif = FIF_UNKNOWN;
  //pointer to the image, once loaded
  FIBITMAP *dib = nullptr;

  //check the file signature and deduce its format
  fif = FreeImage_GetFileType(filename.c_str(), 0);
  if (fif == FIF_UNKNOWN)
    fif = FreeImage_GetFIFFromFilename(filename.c_str());
  if (fif == FIF_UNKNOWN)
    throw std::runtime_error("Data image format can not be determined");

  //check that the plugin has reading capabilities and load the file
  if (FreeImage_FIFSupportsReading(fif))
    dib = FreeImage_Load(fif, filename.c_str());
  if (!dib)
    throw std::runtime_error("Image could not be read");

  // Convert to RGBA float images
  FIBITMAP *hOldImage = dib;
  dib = FreeImage_ConvertToRGBAF(hOldImage); // ==> 4 channels
  FreeImage_Unload(hOldImage);

  //get the image width and height
  int w = FreeImage_GetWidth(dib);
  int h = FreeImage_GetHeight(dib);

  //retrieve the image data
  auto bits = FreeImage_GetBits(dib);

  //if this somehow one of these failed (they shouldn't), return failure
  if ((bits == 0) || (w == 0) || (h == 0))
    throw std::runtime_error("Invalid image");

  // Eigen stores column major by default, we prefer pixel samples in row major.
  using PixelSampleMatrix = Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor>;
  
  // Map the pixel values skipping the alpha value by a larger stride.
  using PixelSamples = Eigen::Map<PixelSampleMatrix, 0, Eigen::Stride<4, 1>>;
  PixelSamples data_vector { reinterpret_cast<float*>(bits), w*h, 3 };

  RgbImage rgb_target { w, h };
  for(int i = 0; i < w*h; i++)
    rgb_target(i%w, i/w) = data_vector.row(i);

  GrayImage gray_target;

  //Free FreeImage's copy of the data
  FreeImage_Unload(dib);

  return {gray_target, rgb_target};
}
