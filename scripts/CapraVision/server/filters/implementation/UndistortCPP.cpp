#include "opencv2/opencv.hpp"
#include "opencv2/gpu/gpu.hpp"

#define DOCSTRING "C++ Undistort"

cv::Mat cameraMatrix(3, 3, cv::DataType<double>::type);
cv::Mat distCoeffs(5, 1, cv::DataType<double>::type);

void init() {
}

void configure() {
}

cv::Mat execute(cv::Mat image) {
  cv::gpu::GpuMat gmat(image);

  gmat.download(image);
	
  return image;
}
