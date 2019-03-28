#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "vision/CameraControl.h"
#include "vision/FlagDetection.h"
#include <iostream>
#include <vector>
#include <signal.h>
#include <unistd.h>
#include "comms/SerialManager.h"

#define DISPLAY_TEST
#define SERIAL_ENABLED

using namespace std;

void exit_handler(int s);

const unsigned short STATUS_ADDR = 0;
const unsigned short NOT_CONNECTED = 0;
const unsigned short NO_CAMERA = 1;
const unsigned short FOUND_CAMERA = 2;
const unsigned short COMPETITION_RED = 1;
const unsigned short COMPETITION_BLUE = 2;

Spinnaker::CameraPtr camera;
Spinnaker::SystemPtr cam_system;
Spinnaker::CameraList camlist;

bool end_program;

int main(int argc, char** argv) {
	end_program = false;
	struct sigaction sig_exit_handler;
	sig_exit_handler.sa_handler = exit_handler;
	sigemptyset(&sig_exit_handler.sa_mask);
	sig_exit_handler.sa_flags = 0;
	sigaction(SIGINT, & sig_exit_handler, NULL);
	sigaction(SIGTERM, &sig_exit_handler, NULL);

	unsigned long lasttime = millis();
	int packet_index = 0;
	int count = 0;
	
	#ifdef SERIAL_ENABLED

	SerialManager::init();
	if(!SerialManager::serial_manager->is_connected()) {
		std::cerr << "Could not connect to V5" << std::endl;
		return 1;
	}
	SerialManager::serial_manager->handle_read();

	#endif

	std::cout << "Searching for cameras...:" << std::endl;
	cam_system = Spinnaker::System::GetInstance();
	camlist = cam_system->GetCameras();
	unsigned int numcams = camlist.GetSize();
	std::cout << "Found " << numcams << (numcams == 1 ? " camera." : " cameras.") << std::endl;
	if(numcams == 0) {
		camlist.Clear();
		cam_system->ReleaseInstance();

		#ifdef SERIAL_ENABLED

		SerialManager::serial_manager->enqueue_write(SerialManager::V5_ID,NO_CAMERA,STATUS_ADDR);
		SerialManager::serial_manager->send_buffer(SerialManager::V5_ID);

		#endif

		return -1;
	}
	
	#ifdef SERIAL_ENABLED

	SerialManager::serial_manager->enqueue_write(SerialManager::V5_ID,FOUND_CAMERA,STATUS_ADDR);
	SerialManager::serial_manager->send_buffer(SerialManager::V5_ID);

	#endif

	//Get the first camera found since there should only ever be one connected
	camera = camlist.GetByIndex(0);
	camera->Init();
	CameraControl::configureCamera(camera);
	camera->BeginAcquisition();
	CameraControl::postStreamConfigureCamera(camera);
	std::cout << "Beginning stream" << std::endl;
	while(!end_program) {

		#ifdef SERIAL_ENABLED

		SerialManager::serial_manager->handle_read();

		#endif

		Spinnaker::ImagePtr image;
		cv::Mat cvimage;
		CameraControl::acquireImage(camera, image);
		CameraControl::spinnaker2cv(image, cvimage);
	     
		std::vector<cv::Rect> bounding_boxes;
		std::vector<cv::Rect> out_rects;

		#ifdef SERIAL_ENABLED
		if(SerialManager::serial_manager->v5_table.read(STATUS_ADDR) == COMPETITION_RED) {
			FlagDetection::getROIs(cvimage, bounding_boxes, FlagDetection::RED);
			FlagDetection::processROIs(cvimage, bounding_boxes, out_rects, FlagDetection::RED);
		}
		else {
			FlagDetection::getROIs(cvimage, bounding_boxes, FlagDetection::RED);
			for(int i = 0; i < bounding_boxes.size(); i++) {
				std::cout << bounding_boxes[i].x << " " << bounding_boxes[i].y << " " 
					<< bounding_boxes[i].width << " " << bounding_boxes[i].height << std::endl;
			}
			FlagDetection::processROIs(cvimage, bounding_boxes, out_rects, FlagDetection::RED);
		}
		#else

		FlagDetection::getROIs(cvimage, bounding_boxes, FlagDetection::RED);
		std::cout << bounding_boxes.size() << std::endl;	
		FlagDetection::processROIs(cvimage, bounding_boxes, out_rects, FlagDetection::RED);

		#endif
	
		for(int i = 0; i < out_rects.size(); i++) {
			std::cout << "Detected rect: " << out_rects[i].x << " " << out_rects[i].y << " " <<
				out_rects[i].width << " " << out_rects[i].height << std::endl;			
		}

		#ifdef SERIAL_ENABLED
		
		SerialManager::serial_manager->enqueue_write(SerialManager::V5_ID,STATUS_ADDR + 1,out_rects.size());

		for(int i = 0; i < out_rects.size(); i++) {
			SerialManager::serial_manager->enqueue_write(SerialManager::V5_ID,STATUS_ADDR + 2 + i*4 + 0,out_rects.at(i).x);
			SerialManager::serial_manager->enqueue_write(SerialManager::V5_ID,STATUS_ADDR + 2 + i*4 + 1,out_rects.at(i).y);
			SerialManager::serial_manager->enqueue_write(SerialManager::V5_ID,STATUS_ADDR + 2 + i*4 + 2,out_rects.at(i).width);
			SerialManager::serial_manager->enqueue_write(SerialManager::V5_ID,STATUS_ADDR + 2 + i*4 + 3,out_rects.at(i).height);
		}

		SerialManager::serial_manager->send_buffer(SerialManager::V5_ID);

		#endif

		#ifdef DISPLAY_TEST
		
		for(int i = 0; i < bounding_boxes.size(); i++) {
			cv::rectangle(cvimage, bounding_boxes[i], cv::Scalar(255, 255, 255), 2);
		}

		for(int i = 0; i < out_rects.size(); i++) {
			cv::rectangle(cvimage, out_rects[i], cv::Scalar(255, 0, 255), 2);
		}
		
		cv::resize(cvimage, cvimage, cv::Size(180, 135));
		cv::imshow("Image", cvimage);
		cv::waitKey(30);

		#else
		usleep(30000);
		#endif
	 	
	}
	
	CameraControl::closeCamera(camera);	
	camlist.Clear();
	cam_system->ReleaseInstance();
}

void exit_handler(int s) {
	end_program = true;
}
