#include "vision/CameraControl.h"

int CameraControl::configureCamera(Spinnaker::CameraPtr camera) {
	Spinnaker::GenApi::CStringPtr vendor_name_ptr = camera->GetNodeMap().GetNode("DeviceModelName");
	std::cout << "Camera name: " << vendor_name_ptr->GetValue() << std::endl;
	
	Spinnaker::GenApi::CEnumerationPtr acquisition_mode_ptr = camera->GetNodeMap().GetNode("AcquisitionMode");
	Spinnaker::GenApi::CEnumEntryPtr acquisition_continuous_ptr = acquisition_mode_ptr->GetEntryByName("Continuous");
	int64_t acquisition_continuous = acquisition_continuous_ptr->GetValue();
	acquisition_mode_ptr->SetIntValue(acquisition_continuous);

	Spinnaker::GenApi::CBooleanPtr frame_rate_enable = camera->GetNodeMap().GetNode("AcquisitionFrameRateEnable");
	frame_rate_enable->SetValue(true);

	Spinnaker::GenApi::CFloatPtr frame_rate = camera->GetNodeMap().GetNode("AcquisitionFrameRate");
	frame_rate->SetValue(20.0);
	
	std::cout << "Camera mode set to continuous." << std::endl;
	return 0;
}


int CameraControl::postStreamConfigureCamera(Spinnaker::CameraPtr camera) {

	Spinnaker::GenApi::CEnumerationPtr exposure_auto_ptr = camera->GetNodeMap().GetNode("ExposureAuto");
	Spinnaker::GenApi::CEnumEntryPtr exposure_auto_off_ptr = exposure_auto_ptr->GetEntryByName("Off");
	int64_t exposure_auto_off = exposure_auto_off_ptr->GetValue();
	exposure_auto_ptr->SetIntValue(exposure_auto_off);

	Spinnaker::GenApi::CEnumerationPtr gain_auto_ptr = camera->GetNodeMap().GetNode("GainAuto");
	Spinnaker::GenApi::CEnumEntryPtr gain_auto_off_ptr = gain_auto_ptr->GetEntryByName("Off");
	int64_t gain_auto_off = gain_auto_off_ptr->GetValue();
	gain_auto_ptr->SetIntValue(gain_auto_off);

	Spinnaker::GenApi::CEnumerationPtr balance_white_auto_ptr = camera->GetNodeMap().GetNode("BalanceWhiteAuto");
	Spinnaker::GenApi::CEnumEntryPtr balance_white_auto_off_ptr = balance_white_auto_ptr->GetEntryByName("Off");
	int64_t balance_white_auto_off = balance_white_auto_off_ptr->GetValue();
	balance_white_auto_ptr->SetIntValue(balance_white_auto_off);

	Spinnaker::GenApi::CEnumerationPtr balance_ratio_selector_ptr = camera->GetNodeMap().GetNode("BalanceRatioSelector");
	Spinnaker::GenApi::CEnumEntryPtr balance_ratio_red_ptr = balance_ratio_selector_ptr->GetEntryByName("Red");
	Spinnaker::GenApi::CEnumEntryPtr balance_ratio_blue_ptr = balance_ratio_selector_ptr->GetEntryByName("Blue");
	int64_t balance_ratio_red = balance_ratio_red_ptr->GetValue();
	int64_t balance_ratio_blue = balance_ratio_blue_ptr->GetValue();
	balance_white_auto_ptr->SetIntValue(balance_white_auto_off);

	Spinnaker::GenApi::CFloatPtr exposure_time = camera->GetNodeMap().GetNode("ExposureTime");
	exposure_time->SetValue(15000);

	Spinnaker::GenApi::CFloatPtr gain = camera->GetNodeMap().GetNode("Gain");
	gain->SetValue(15);
	
	Spinnaker::GenApi::CFloatPtr balance_ratio = camera->GetNodeMap().GetNode("BalanceRatio");
	balance_ratio_selector_ptr->SetIntValue(balance_ratio_red);
	balance_ratio->SetValue(1.25);
	balance_ratio_selector_ptr->SetIntValue(balance_ratio_blue);
	balance_ratio->SetValue(3.6);

	Spinnaker::GenApi::CBooleanPtr gamma_enable = camera->GetNodeMap().GetNode("GammaEnable");
	gamma_enable->SetValue(true);

	Spinnaker::GenApi::CFloatPtr gamma = camera->GetNodeMap().GetNode("Gamma");
	gamma->SetValue(1.0);
	return 0;
}

int CameraControl::closeCamera(Spinnaker::CameraPtr camera) {
	Spinnaker::GenApi::CEnumerationPtr balance_white_auto_ptr = camera->GetNodeMap().GetNode("BalanceWhiteAuto");
	Spinnaker::GenApi::CEnumEntryPtr balance_white_auto_off_ptr = balance_white_auto_ptr->GetEntryByName("Continuous");
	int64_t balance_white_auto_off = balance_white_auto_off_ptr->GetValue();
	balance_white_auto_ptr->SetIntValue(balance_white_auto_off);

	camera->EndAcquisition();
	camera->DeInit();
	camera = NULL;
}

int CameraControl::acquireImage(Spinnaker::CameraPtr camera, Spinnaker::ImagePtr& image) {
	Spinnaker::ImagePtr raw_image = camera->GetNextImage();
	image = raw_image->Convert(Spinnaker::PixelFormat_BGR8, Spinnaker::NEAREST_NEIGHBOR);
}

bool CameraControl::spinnaker2cv(Spinnaker::ImagePtr image, cv::Mat& mat) {
	cv::Mat cvimg = cv::Mat(image->GetHeight() + image->GetYPadding(), image->GetWidth() + image->GetXPadding(), CV_8UC3, image->GetData(), image->GetStride());
	mat = cvimg;
	return true;
}
