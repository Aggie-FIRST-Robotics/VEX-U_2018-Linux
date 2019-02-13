#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>
#include <vector>

typedef std::vector<cv::Point> contour;
enum Color {
	RED,
	BLUE
};

int configureCamera(Spinnaker::CameraPtr camera);
int postStreamConfigureCamera(Spinnaker::CameraPtr camera);
int acquireImage(Spinnaker::CameraPtr camera, Spinnaker::ImagePtr& image);
bool spinnaker2cv(Spinnaker::ImagePtr image, cv::Mat& mat);
void thresholdMask(cv::Mat& in, cv::Mat& out, double min_val, double max_val);
void HSVFilter(cv::Mat& in, cv::Mat& out, double min_h, double max_h, double min_s, double max_s, double min_v, double max_v);
void removeIsolatedPixels(cv::Mat& in, cv::Mat& out, int acceptance);
void getROIs(std::vector<contour>& in_contours, std::vector<cv::Rect>& out_rects, int min_area, int max_area, int scaling, int edge_margin);
void processROIs(cv::Mat& image, std::vector<cv::Rect>& rois, std::vector<cv::Rect>& out_quads, Color color);

float kernel[25] = {0.02, 0.02, 0.02, 0.02, 0.02, 
		    0.02, 0.07, 0.07, 0.07, 0.02,
		    0.02, 0.07, 0.12, 0.07, 0.02,
		    0.02, 0.07, 0.07, 0.07, 0.02,
		    0.02, 0.02, 0.02, 0.02, 0.02};


int main(int argc, char** argv) {
	std::cout << "Searching for cameras...:" << std::endl;
	Spinnaker::SystemPtr system = Spinnaker::System::GetInstance();
	Spinnaker::CameraList camlist = system->GetCameras();
	unsigned int numcams = camlist.GetSize();
	std::cout << "Found " << numcams << (numcams == 1 ? " camera." : " cameras.") << std::endl;
	if(numcams == 0) {
		camlist.Clear();
		system->ReleaseInstance();
		return -1;
	}
	
	//Get the first camera found since there should only ever be one connected
	Spinnaker::CameraPtr camera = camlist.GetByIndex(0);
	camera->Init();
	configureCamera(camera);
	camera->BeginAcquisition();
	postStreamConfigureCamera(camera);
	do {
		Spinnaker::ImagePtr image;
		cv::Mat cvimage;
		acquireImage(camera, image);
		spinnaker2cv(image, cvimage);
		cv::Mat scaledimage;
		cv::resize(cvimage, scaledimage, cv::Size(), 0.25, 0.25);
		cv::Mat scaledhsvimage;
		cv::cvtColor(scaledimage, scaledhsvimage, CV_BGR2HSV);
		std::vector<cv::Mat> combinedfilter(3);
		HSVFilter(scaledhsvimage, combinedfilter[2], 250, 15, 130, 220, 60, 180);
		HSVFilter(scaledhsvimage, combinedfilter[0], 105, 120, 80, 180, 60, 130);
		HSVFilter(scaledhsvimage, combinedfilter[1], 35, 55, 40, 130, 100, 230);
		removeIsolatedPixels(combinedfilter[0], combinedfilter[0], 80);
		removeIsolatedPixels(combinedfilter[1], combinedfilter[1], 80);
		removeIsolatedPixels(combinedfilter[2], combinedfilter[2], 80);
		std::vector<std::vector<contour>> contours_list = std::vector<std::vector<contour>>(3);
		std::vector<std::vector<cv::Vec4i>> hierarchy_list = std::vector<std::vector<cv::Vec4i>>(3);
		cv::findContours(combinedfilter[0], contours_list[0], hierarchy_list[0], CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
		cv::findContours(combinedfilter[1], contours_list[1], hierarchy_list[1], CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
		cv::findContours(combinedfilter[2], contours_list[2], hierarchy_list[2], CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
		std::vector<cv::Rect> bounding_boxes;
		getROIs(contours_list[2], bounding_boxes, 10, 2000, 4, 3);
		std::vector<cv::Rect> out_rects;
		processROIs(cvimage, bounding_boxes, out_rects, RED);
		cv::Mat combinedfiltermat;
		cv::merge(combinedfilter, combinedfiltermat);
		for(int i = 0; i < bounding_boxes.size(); i++) {
			cv::rectangle(cvimage, bounding_boxes[i], cv::Scalar(255, 255, 255), 2);
		}
		for(int i = 0; i < out_rects.size(); i++) {
			cv::rectangle(cvimage, out_rects[i], cv::Scalar(255, 0, 255), 2);
		}
		cv::Mat displayimage;
		cv::resize(cvimage, displayimage, cv::Size(), 0.5, 0.5);
		cv::imshow("Image", displayimage);
	} while(cv::waitKey(30)!='q');
	
	
	Spinnaker::GenApi::CEnumerationPtr balance_white_auto_ptr = camera->GetNodeMap().GetNode("BalanceWhiteAuto");
	Spinnaker::GenApi::CEnumEntryPtr balance_white_auto_off_ptr = balance_white_auto_ptr->GetEntryByName("Continuous");
	int64_t balance_white_auto_off = balance_white_auto_off_ptr->GetValue();
	balance_white_auto_ptr->SetIntValue(balance_white_auto_off);

	camera->EndAcquisition();
	camera->DeInit();
	camera = NULL;
	camlist.Clear();
	system->ReleaseInstance();
	
}

int configureCamera(Spinnaker::CameraPtr camera) {
	Spinnaker::GenApi::CStringPtr vendor_name_ptr = camera->GetNodeMap().GetNode("DeviceModelName");
	std::cout << "Camera name: " << vendor_name_ptr->GetValue() << std::endl;
	
	Spinnaker::GenApi::CEnumerationPtr acquisition_mode_ptr = camera->GetNodeMap().GetNode("AcquisitionMode");
	Spinnaker::GenApi::CEnumEntryPtr acquisition_continuous_ptr = acquisition_mode_ptr->GetEntryByName("Continuous");
	int64_t acquisition_continuous = acquisition_continuous_ptr->GetValue();
	acquisition_mode_ptr->SetIntValue(acquisition_continuous);

	Spinnaker::GenApi::CBooleanPtr frame_rate_enable = camera->GetNodeMap().GetNode("AcquisitionFrameRateEnable");
	frame_rate_enable->SetValue(true);

	Spinnaker::GenApi::CFloatPtr frame_rate = camera->GetNodeMap().GetNode("AcquisitionFrameRate");
	frame_rate->SetValue(10.0);
	
	std::cout << "Camera mode set to continuous." << std::endl;
	return 0;
}


int postStreamConfigureCamera(Spinnaker::CameraPtr camera) {

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
	gamma->SetValue(0.9);
	return 0;
}

int acquireImage(Spinnaker::CameraPtr camera, Spinnaker::ImagePtr& image) {
	Spinnaker::ImagePtr raw_image = camera->GetNextImage();
	image = raw_image->Convert(Spinnaker::PixelFormat_BGR8, Spinnaker::NEAREST_NEIGHBOR);
}

bool spinnaker2cv(Spinnaker::ImagePtr image, cv::Mat& mat) {
	cv::Mat cvimg = cv::Mat(image->GetHeight() + image->GetYPadding(), image->GetWidth() + image->GetXPadding(), CV_8UC3, image->GetData(), image->GetStride());
	mat = cvimg;
	return true;
}

void thresholdMask(cv::Mat& in, cv::Mat& out, double min_val, double max_val) {
	if(min_val > max_val) {
		cv::Mat minthresh;
		cv::Mat maxthresh;
		cv::threshold(in, maxthresh, min_val, 255, CV_THRESH_BINARY);
		cv::threshold(in, minthresh, max_val, 255, CV_THRESH_BINARY_INV);
		cv::bitwise_or(minthresh, maxthresh, out);
	}
	else {
		cv::inRange(in, min_val, max_val, out);
	}
}

void HSVFilter(cv::Mat& in, cv::Mat& out, double min_h, double max_h, double min_s, double max_s, double min_v, double max_v) {
	std::vector<cv::Mat> splithsv(3);
	cv::split(in, splithsv);
	thresholdMask(splithsv[0], splithsv[0], min_h, max_h);
	thresholdMask(splithsv[1], splithsv[1], min_s, max_s);
	thresholdMask(splithsv[2], splithsv[2], min_v, max_v);
	cv::bitwise_and(splithsv[0], splithsv[1], out);
	cv::bitwise_and(splithsv[2], out, out);
}

void removeIsolatedPixels(cv::Mat& in, cv::Mat& out, int acceptance) {
	cv::Mat kernel_mat = cv::Mat(5, 5, CV_32F, &kernel);
	cv::filter2D(in, out, -1, kernel_mat);
	cv::threshold(out, out, acceptance, 255, CV_THRESH_BINARY);	
}

void getROIs(std::vector<contour>& in_contours, std::vector<cv::Rect>& out_rects, int min_area, int max_area, int scaling, int edge_margin) {
	out_rects = std::vector<cv::Rect>();
	for(int i = 0; i < in_contours.size(); i++) {
		cv::Rect bounding_box = cv::boundingRect(cv::Mat(in_contours[i]));
		if(bounding_box.area() >= min_area && bounding_box.area() <= max_area) {
			bounding_box.x = scaling*bounding_box.x-edge_margin*bounding_box.width;
			bounding_box.y = scaling*bounding_box.y-edge_margin*bounding_box.height;
			bounding_box.height = (2*edge_margin+scaling)*bounding_box.height;
			bounding_box.width = (2*edge_margin+scaling)*bounding_box.width;
			out_rects.push_back(bounding_box);
		}
	}
}

void processROIs(cv::Mat& image, std::vector<cv::Rect>& rois, std::vector<cv::Rect>& out_quads, Color color) {
	out_quads = std::vector<cv::Rect>();
	for(int i = 0; i < rois.size(); i++) {
		if(!(rois[i].x > 0 && rois[i].y > 0 && rois[i].x < image.cols && rois[i].y < image.rows &&
			rois[i].x+rois[i].width > 0 && rois[i].y+rois[i].height > 0 && 
			rois[i].x+rois[i].width < image.cols && rois[i].y+rois[i].height < image.rows &&
			rois[i].width > 0 && rois[i].height > 0)) {
			continue;
		}
		cv::Mat subimage = cv::Mat(image, rois[i]);	
		cv::Mat hsvimage;
		cv::cvtColor(subimage, hsvimage, CV_BGR2HSV);
		std::vector<cv::Mat> combinedfilter(2);
		if(color == RED) {
			HSVFilter(hsvimage, combinedfilter[0], 250, 20, 60, 240, 40, 200);
		}
		else {
			HSVFilter(hsvimage, combinedfilter[0], 105, 120, 80, 180, 60, 130);
		}
		HSVFilter(hsvimage, combinedfilter[1], 30, 60, 40, 160, 80, 240);
		removeIsolatedPixels(combinedfilter[0], combinedfilter[0], 80);
		removeIsolatedPixels(combinedfilter[1], combinedfilter[1], 80);
		std::vector<std::vector<contour>> contours_list = std::vector<std::vector<contour>>(2);
		std::vector<std::vector<cv::Vec4i>> hierarchy_list = std::vector<std::vector<cv::Vec4i>>(2);
		cv::findContours(combinedfilter[0], contours_list[0], hierarchy_list[0], CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
		cv::findContours(combinedfilter[1], contours_list[1], hierarchy_list[1], CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
		int j = 0;
		while(hierarchy_list[0][j][0] >= 0) {
			cv::Rect color_bounding_rect = cv::boundingRect(cv::Mat(contours_list[0][j]));
			bool green_contour_found = false;
			for(int k = 0; k < contours_list[1].size(); k++) {
				cv::Rect green_bounding_rect = cv::boundingRect(cv::Mat(contours_list[1][k]));
				if(color == RED) {
					if(color_bounding_rect.x+color_bounding_rect.width-green_bounding_rect.x > -10 && 
						color_bounding_rect.x+color_bounding_rect.width-green_bounding_rect.x < 10 &&
						color_bounding_rect.y-green_bounding_rect.y > -20 &&
						color_bounding_rect.y-green_bounding_rect.y < 20 && 
						//hierarchy_list[0][j][2] >= 0 &&
						color_bounding_rect.area() > 60 &&
						green_bounding_rect.area() > 20 &&
						color_bounding_rect.x+color_bounding_rect.width/2-rois[i].width/2 > -10 &&
						color_bounding_rect.x+color_bounding_rect.width/2-rois[i].width/2 < 10 &&
						color_bounding_rect.y+color_bounding_rect.height/2-rois[i].height/2 > -10 &&
						color_bounding_rect.y+color_bounding_rect.height/2-rois[i].height/2 < 10) {
							color_bounding_rect.x += rois[i].x;
							color_bounding_rect.y += rois[i].y;
							out_quads.push_back(color_bounding_rect);
					}
				}
				else {

				}
			}
			j = hierarchy_list[0][j][0];
		}
	}
}
