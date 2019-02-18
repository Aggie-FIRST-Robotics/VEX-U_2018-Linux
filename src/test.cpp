#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "vision/CameraControl.h"
#include "vision/FlagDetection.h"
#include <iostream>
#include <vector>
#include "comms/SerialManager.h"

void handle_serial (SerialManager*);

int main(int argc, char** argv) {
	//thread serial_task(handle_serial, &serial_manager);
	unsigned long timer = millis();
	unsigned long counter = 0;
	int disp;
	while(true) {
		//cout << timer << "\t" << millis() << endl;
		if( timer + UPDATE_PERIOD <= millis()) {

			timer = millis();
			serial_manager.v5_table.age_by(UPDATE_PERIOD);
			//serial_manager.arduino_table.age_by(UPDATE_PERIOD);
			serial_manager.handle_read();
			//erial_manager.receive_buffer(0);
			//serial_manager.receive_buffer(1);
			for (int i = 0; i < 64; i++) {
				serial_manager.enqueue_write (V5_ID, (uint8_t)i, (short)i);
			}
			//disp = counter % 1000;
			//serial_manager.enqueue_write (V5_ID, 0, disp);
			serial_manager.send_buffer(V5_ID);
			//cout << "Loop" << endl;
			//serial_manager.send_buffer(ARDUINO_ID);
			//cout << disp << endl;
			//cout << serial_manager.v5_table.read(0) << endl;
			//counter++;
			// for(int i = 0; i < 8; i++) {
			// 	printf("\n%d: %d\t%d: %d\t%d: %d\t%d: %d\t%d: %d\t%d: %d\t%d: %d\t%d: %d\n", 	0 + i*8 ,serial_manager.v5_table.read(0 + i*8),
			// 																					1 + i*8 ,serial_manager.v5_table.read(1 + i*8),
			// 																					2 + i*8 ,serial_manager.v5_table.read(2 + i*8),
			// 																					3 + i*8 ,serial_manager.v5_table.read(3 + i*8),
			// 																					4 + i*8 ,serial_manager.v5_table.read(4 + i*8),
			// 																					5 + i*8 ,serial_manager.v5_table.read(5 + i*8),
			// 																					6 + i*8 ,serial_manager.v5_table.read(6 + i*8),
			// 																					7 + i*8 ,serial_manager.v5_table.read(7 + i*8));

			// }
			// cout << "---------------------------------------------------------------" << endl;
		 }
	}
	// std::cout << "Searching for cameras...:" << std::endl;
	// Spinnaker::SystemPtr system = Spinnaker::System::GetInstance();
	// Spinnaker::CameraList camlist = system->GetCameras();
	// unsigned int numcams = camlist.GetSize();
	// std::cout << "Found " << numcams << (numcams == 1 ? " camera." : " cameras.") << std::endl;
	// if(numcams == 0) {
	// 	camlist.Clear();
	// 	system->ReleaseInstance();
	// 	return -1;
	// }
	
	// //Get the first camera found since there should only ever be one connected
	// Spinnaker::CameraPtr camera = camlist.GetByIndex(0);
	// camera->Init();
	// CameraControl::configureCamera(camera);
	// camera->BeginAcquisition();
	// CameraControl::postStreamConfigureCamera(camera);
	// do {
	// 	Spinnaker::ImagePtr image;
	// 	cv::Mat cvimage;
	// 	CameraControl::acquireImage(camera, image);
	// 	CameraControl::spinnaker2cv(image, cvimage);
		
	// 	std::vector<cv::Rect> bounding_boxes;
	// 	FlagDetection::getROIs(cvimage, bounding_boxes, FlagDetection::BLUE);
	// 	std::cout << bounding_boxes.size() << std::endl;	
	// 	std::vector<cv::Rect> out_rects;
	// 	FlagDetection::processROIs(cvimage, bounding_boxes, out_rects, FlagDetection::BLUE);
	
	// 	cv::Mat combinedfiltermat;
	// 	for(int i = 0; i < bounding_boxes.size(); i++) {
	// 		cv::rectangle(cvimage, bounding_boxes[i], cv::Scalar(255, 255, 255), 2);
	// 	}
	// 	for(int i = 0; i < out_rects.size(); i++) {
	// 		cv::rectangle(cvimage, out_rects[i], cv::Scalar(255, 0, 255), 2);
	// 	}
	// 	cv::Mat displayimage;
	// 	cv::resize(cvimage, displayimage, cv::Size(), 0.5, 0.5);
	// 	cv::imshow("Image", displayimage);
	// } while(cv::waitKey(30)!='q');
	
	// CameraControl::closeCamera(camera);	
	// camlist.Clear();
	// system->ReleaseInstance();
	
}

void handle_serial(SerialManager* params) {
	unsigned long timer = millis();
	while(true) {
		cout << "Loop" << endl;
		if( timer + UPDATE_PERIOD < millis()) {
			timer = millis();
			params->v5_table.age_by(UPDATE_PERIOD);
			params->arduino_table.age_by(UPDATE_PERIOD);
			params->receive_buffer(0);
			params->receive_buffer(1);
			params->send_buffer(V5_ID);
			params->send_buffer(ARDUINO_ID);
		}
	}
}

