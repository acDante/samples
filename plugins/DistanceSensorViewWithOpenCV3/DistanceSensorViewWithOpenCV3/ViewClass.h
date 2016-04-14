#pragma once

#include <opencv2\opencv.hpp>
#include <opencv2\imgproc.hpp>
#include <math.h>

#pragma comment (lib,"C:\\opencv3.0.0\\build\\x86\\vc12\\lib\\opencv_world300.lib")


using namespace System;
using namespace System::ComponentModel;
using namespace System::Collections;
using namespace System::Windows::Forms;
using namespace System::Data;
using namespace System::Drawing;
public ref class ViewClass{
public:
	ViewClass(){}
	~ViewClass(){}
	int initialize(sigverse::SIGService ^service, System::String ^getEntiryName, int getCameraID, int image_size){
		this->my = service;
		this->entity_name = getEntiryName;
		this->cameraID = getCameraID;
		this->img_size = image_size;

		return 0;
	}
	std::string sysString2stdStrng(System::String^ sys_str)
	{
		std::string tmp;
		System::IntPtr mptr;
		mptr = System::Runtime::InteropServices::Marshal::StringToHGlobalAnsi(sys_str);
		tmp = static_cast<const char*>(mptr.ToPointer());
		System::Runtime::InteropServices::Marshal::FreeHGlobal(mptr);
		return tmp;
	} 
	cv::Mat PinP_tr(const cv::Mat &srcImg, const cv::Mat &smallImg, const int tx, const int ty)
	{
		//背景画像の作成
		cv::Mat dstImg;
		srcImg.copyTo(dstImg);

		//前景画像の変形行列
		cv::Mat mat = (cv::Mat_<double>(2, 3) << 1.0, 0.0, tx, 0.0, 1.0, ty);

		//アフィン変換の実行
		cv::warpAffine(smallImg, dstImg, mat, dstImg.size(), CV_INTER_LINEAR, cv::BORDER_TRANSPARENT);
		return dstImg;
	}

	int main_process(){
		sigverse::ViewImage ^camera = my->captureView(entity_name, cameraID);
		sigverse::ViewImage ^depth = my->getDepthImage(entity_name, 0, 256 * 3 - 1, cameraID);
		sigverse::ViewImage ^lrf = my->distanceSensor1D(entity_name, 0, 256 * 3 - 1, cameraID);

		cv::Mat view(camera->getHeight(), camera->getWidth(), CV_8UC3);
		cv::Mat depth_view(camera->getHeight(), camera->getWidth(), CV_8UC1);
		this->img_size = camera->getWidth() * 2;
		cv::Mat distimg = cv::Mat::zeros(img_size, img_size, CV_8UC3);
		view.data = (unsigned char*)camera->getBuffer();
		depth_view.data = (unsigned char*)depth->getBuffer();
		cv::cvtColor(depth_view, depth_view, CV_GRAY2BGR);
		{
			int ar = 1.5;
			double fovy = camera->getFOVy()*180./CV_PI / 45. * 65 * CV_PI / 180.;
			double theta = 2 * atan(tan(fovy*0.5)*ar) * 180.0 / CV_PI;
			int length = lrf->getBufferLength();
			char *buf = lrf->getBuffer();
			for (int i = 0; i < length; i++, buf++){
				if (((int)(unsigned char)*buf) == 255) continue;
				int distance = ((int)(unsigned char)*buf) * 3;
				double angle = (-1 * theta / 2 + theta / length*i)*CV_PI / 180.;
				cv::line(distimg, cv::Point(img_size / 2, img_size), cv::Point(img_size / 2 + distance * sin(angle), img_size - distance * cos(angle)), cv::Scalar(255,255,255), 2);
			}
		}
		distimg = PinP_tr(distimg, depth_view, camera->getWidth()-1, 0);
		distimg = PinP_tr(distimg, view, 0, 0);
		//cv::imshow(sysString2stdStrng(this->entity_name + "-" + Convert::ToString(this->cameraID) + "-camera"), view);
		cv::imshow(sysString2stdStrng(this->entity_name + "-" + Convert::ToString(this->cameraID) + "-sensor"), distimg);
		cv::waitKey(1);
		delete camera;
		delete lrf;
		return 0;
	}
	sigverse::SIGService ^my;
private:
	System::String ^entity_name;
	int cameraID;
	int img_size;
};