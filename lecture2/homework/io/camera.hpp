#ifndef CAMERA_HPP
#define CAMERA_HPP

#include <opencv2/opencv.hpp>

#include "hikrobot/include/MvCameraControl.h"

class Camera
{
public:
  Camera();
  ~Camera();
  cv::Mat read();

private:
  cv::Mat transfer(MV_FRAME_OUT & raw);

  void * handle_;
  bool grabbing_;
};

#endif