#ifndef CAMERA_CONTROL_H
#define CAMERA_CONTROL_H

#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"
#include "opencv2/core.hpp"

class CameraControl {
private:
	CameraControl();
public:
	static int configureCamera(Spinnaker::CameraPtr camera);
	static int postStreamConfigureCamera(Spinnaker::CameraPtr camera);
	static int closeCamera(Spinnaker::CameraPtr camera);
	static int acquireImage(Spinnaker::CameraPtr camera, Spinnaker::ImagePtr& image);
	static bool spinnaker2cv(Spinnaker::ImagePtr image, cv::Mat& mat);
};

#endif
