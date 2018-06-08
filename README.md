# mview
# Multi-View Stero Reconstruction

## 1 Introduction

The stereo reconstruction is a work which aims at building a 3D model from only the
RGB images without being provided the depth information. Our team project is going
to realize this reconstruction method on a multi-view stereo dataset of an Greek style
temple and build up its 3D model.

## 2 Dataset

We use the stereo dataset provided by http://vision.middlebury.edu/mview/data/.
The dataset provides both the intrinsic matrices and the extrinsic matrices for the cam-
era positions, together with the images. We will start to deal with the small dataset
”TempleSparseRing” which contains 16 stereo images on the temple and may be expand
to a larger dataset if our code works well and we have time left after we finished the
work on the small one.

## 3 Processing pipeline

Our stereo reconstruction task will consist of the following steps:

1. For each pair of images, match the key points. We will try different matching
methods (block matching,sift, etc..) and choose the one with the best effect

2. Use mathematical (triangle) relations to get the distances (depth) information

3. Build a point cloud and estimate a 3D model with reconstructed textures

4. Render and output from different views with the help of the visualization tools
 (Meshlab, Blender)

5. Compare the rendered model against existing evaluation standard.

## 4 References

• Benchmarking/Evaluation standard: http://vision.middlebury.edu/mview/eval/

• Slide reference: http://www.cs.middlebury.edu/ schar/papers/structlight/

• Related paper: "High-Accuracy Stereo Depth Maps Using Structured Light",
Daniel Scharstein and Richard Szeliski In IEEE Computer Society Conference
on Computer Vision and Pattern Recognition (CVPR 2003), volume 1, pages
195-202, Madison, WI, June 2003.

• Possible library: https://github.com/cdcseacave/openMVS/wiki/Building

