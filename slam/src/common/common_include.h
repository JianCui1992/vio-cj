#pragma once

#include <opencv2/opencv.hpp>

namespace VIO
{
struct feature_points
{
	cv::Mat image;
	vector<cv::Point3f> v_p;
	vector<cv::Point2f> v_uv;
	vector<cv::Point2f> v_vel;
	vector<int> id;
};


}