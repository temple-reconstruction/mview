#include "mview.h"

void triangulate(const Rectified &rectified, Correspondence &correspondence)
{
  
  const auto &extrinsics = rectified.extrinsics;
  const auto &baseline_distance = rectified.baseline_distance;

  const auto &pixel_left = correspondence.left;
  const auto &pixel_right = correspondence.right;
  const auto &cost = correspondence.cost;
  const auto &global = correspondence.global;

  const auto parallaxX = -(pixel_right.x - pixel_left.x);
  const auto parallaxY = -(pixel_right.y - pixel_left.y);

  global[2] = ((baseline_distance / parallaxX) * camera_constant) * extrinsics;
  global[0] = (pixel_left.x * (baseline_distance / parallaxX)) * extrinsics;
  global[1] = (((pixel_left.y + pixel_right.y) / 2) * (baseline_distance / parallaxX))) * extrinsics;
}