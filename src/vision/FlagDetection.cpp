#include "vision/FlagDetection.h"


const std::array<float, 25> FlagDetection::kernel =  {0.02, 0.02, 0.02, 0.02, 0.02, 
				    0.02, 0.07, 0.07, 0.07, 0.02,
				    0.02, 0.07, 0.12, 0.07, 0.02,
				    0.02, 0.07, 0.07, 0.07, 0.02,
				    0.02, 0.02, 0.02, 0.02, 0.02};

void FlagDetection::getROIs(cv::Mat& cvimage, std::vector<cv::Rect>& rois, Color color) {
	cv::Mat scaledimage;
	cv::resize(cvimage, scaledimage, cv::Size(), 0.25, 0.25);
	cv::Mat scaledhsvimage;
	cv::cvtColor(scaledimage, scaledhsvimage, CV_BGR2HSV);
	cv::Mat colorfilter;
	if(color == RED) {
		HSVFilter(scaledhsvimage, colorfilter, 250, 15, 130, 220, 60, 180);
	}
	else {
		HSVFilter(scaledhsvimage, colorfilter, 100, 130, 50, 190, 5, 160);
	}
	removeIsolatedPixels(colorfilter, colorfilter, 80);
	std::vector<contour> contour_list;
	std::vector<cv::Vec4i> hierarchy_list;
	cv::findContours(colorfilter, contour_list, hierarchy_list, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
	contourBoundingBoxes(contour_list, rois, 10, 2000, 4, 3);
}

void FlagDetection::thresholdMask(cv::Mat& in, cv::Mat& out, double min_val, double max_val) {
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

void FlagDetection::HSVFilter(cv::Mat& in, cv::Mat& out, double min_h, double max_h, double min_s, double max_s, double min_v, double max_v) {
	std::vector<cv::Mat> splithsv(3);
	cv::split(in, splithsv);
	thresholdMask(splithsv[0], splithsv[0], min_h, max_h);
	thresholdMask(splithsv[1], splithsv[1], min_s, max_s);
	thresholdMask(splithsv[2], splithsv[2], min_v, max_v);
	cv::bitwise_and(splithsv[0], splithsv[1], out);
	cv::bitwise_and(splithsv[2], out, out);
}

void FlagDetection::removeIsolatedPixels(cv::Mat& in, cv::Mat& out, int acceptance) {
	float* _kernel = const_cast<float*>(kernel.data());
	cv::Mat kernel_mat = cv::Mat(5, 5, CV_32F, _kernel);
	cv::filter2D(in, out, -1, kernel_mat);
	cv::threshold(out, out, acceptance, 255, CV_THRESH_BINARY);	
}

void FlagDetection::contourBoundingBoxes(std::vector<contour>& in_contours, std::vector<cv::Rect>& out_rects, int min_area, int max_area, int scaling, int edge_margin) {
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

void FlagDetection::processROIs(cv::Mat& image, std::vector<cv::Rect>& rois, std::vector<cv::Rect>& out_quads, Color color) {
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
			HSVFilter(hsvimage, combinedfilter[0], 90, 130, 50, 190, 5, 160);
		}
		HSVFilter(hsvimage, combinedfilter[1], 30, 60, 40, 160, 80, 240);
		removeIsolatedPixels(combinedfilter[0], combinedfilter[0], 130);
		removeIsolatedPixels(combinedfilter[1], combinedfilter[1], 80);
		std::vector<std::vector<contour>> contours_list = std::vector<std::vector<contour>>(2);
		std::vector<std::vector<cv::Vec4i>> hierarchy_list = std::vector<std::vector<cv::Vec4i>>(2);
		cv::findContours(combinedfilter[0], contours_list[0], hierarchy_list[0], CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
		cv::findContours(combinedfilter[1], contours_list[1], hierarchy_list[1], CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
		int j = 0;
		if(contours_list[0].size() > 0 && contours_list[1].size() > 0) {
			while(hierarchy_list[0][j][0] >= 0) {
				cv::Rect color_bounding_rect = cv::boundingRect(cv::Mat(contours_list[0][j]));
				bool green_contour_found = false;
				for(int k = 0; k < contours_list[1].size(); k++) {
					cv::Rect green_bounding_rect = cv::boundingRect(cv::Mat(contours_list[1][k]));
					if(color == RED) {
						if(
							color_bounding_rect.x+color_bounding_rect.width-green_bounding_rect.x > -10 && 
							color_bounding_rect.x+color_bounding_rect.width-green_bounding_rect.x < 10 &&
							color_bounding_rect.y-green_bounding_rect.y > -20 &&
							color_bounding_rect.y-green_bounding_rect.y < 20 && 
							color_bounding_rect.area() > 60 &&
							//green_bounding_rect.area() > 20 &&
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
						if(
							color_bounding_rect.x-green_bounding_rect.x-green_bounding_rect.width > -10 && 
							color_bounding_rect.x-green_bounding_rect.x-green_bounding_rect.width < 10 &&
							color_bounding_rect.y-green_bounding_rect.y > -20 &&
							color_bounding_rect.y-green_bounding_rect.y < 20 && 
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
				}
				j = hierarchy_list[0][j][0];
			}
		}
	}
}
