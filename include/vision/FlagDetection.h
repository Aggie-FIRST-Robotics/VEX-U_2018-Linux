#ifndef FLAG_DETECTION_H
#define FLAG_DETECTION_H

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "vision/CameraControl.h"
#include <iostream>
#include <vector>
#include <array>
#include <iostream>

typedef std::vector<cv::Point> contour;

class FlagDetection {
public:
	enum Color {
		RED,
		BLUE
	};
	
	static void getROIs(cv::Mat& cvimage, std::vector<cv::Rect>& rois, Color color);
	static void thresholdMask(cv::Mat& in, cv::Mat& out, double min_val, double max_val);
	static void HSVFilter(cv::Mat& in, cv::Mat& out, double min_h, double max_h, double min_s, double max_s, double min_v, double max_v);
	static void removeIsolatedPixels(cv::Mat& in, cv::Mat& out, int acceptance);
	static void contourBoundingBoxes(std::vector<contour>& in_contours, std::vector<cv::Rect>& out_rects, int min_area, int max_area, int scaling, int edge_margin);
	static void processROIs(cv::Mat& image, std::vector<cv::Rect>& rois, std::vector<cv::Rect>& out_quads, Color color);

private:
	static const std::array<float, 25> kernel;
};

#endif
