#pragma once

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>


const double FOCAL_LENGTH = 460.0;
const int WINDOW_SIZE = 10;
const int NUM_OF_CAM = 1;
const int NUM_OF_F = 1000;
const double TIME_SHIFT = 1526010000.0;

namespace VIO
{
	
extern std::string FISHEYE_MASK;

extern std::vector<std::string> CAM_NAMES;
extern int MAX_CNT;
extern int MIN_DIST;
extern int START;
extern int END;
extern int FREQ;
extern double F_THRESHOLD;
extern int SHOW_TRACK;
extern int STEREO_TRACK;
extern int EQUALIZE;
extern int FISHEYE;
extern bool PUB_THIS_FRAME;
extern std::string IMAGE_PATH;
extern std::string IMU_PATH;




extern double INIT_DEPTH;
extern double MIN_PARALLAX;
extern int ESTIMATE_EXTRINSIC;
extern std::string EX_CALIB_RESULT_PATH;


extern double ACC_N, ACC_W;
extern double GYR_N, GYR_W;

extern std::vector<Eigen::Matrix3d> RIC;
extern std::vector<Eigen::Vector3d> TIC;
extern Eigen::Vector3d G;

extern double BIAS_ACC_THRESHOLD;
extern double BIAS_GYR_THRESHOLD;

extern double SOLVER_TIME;
extern int NUM_ITERATIONS;


extern double TD;
extern double TR;
extern int ESTIMATE_TD;
extern int ROLLING_SHUTTER;
extern double ROW, COL;

// extern double FOCAL_LENGTH;
// extern int WINDOW_SIZE;
// extern int NUM_OF_CAM;
// extern int NUM_OF_F;

void read_parameters(const std::string &filename);





struct image_with_timestamp
{
	cv::Mat image;
	double timestamp;
};

struct feature_points
{
	image_with_timestamp image_time;
	std::vector<cv::Point3f> v_p;
	std::vector<cv::Point2f> v_uv;
	std::vector<cv::Point2f> v_vel;
	std::vector<int> id;
};

struct imu_with_timestamp
{
	double timestamp;

	double linear_acceleration_x;
	double linear_acceleration_y;
	double linear_acceleration_z;

	double angular_velocity_x;
	double angular_velocity_y;
	double angular_velocity_z;
};

enum SIZE_PARAMETERIZATION
{
    SIZE_POSE = 7,
    SIZE_SPEEDBIAS = 9,
    SIZE_FEATURE = 1
};

enum StateOrder
{
    O_P = 0,
    O_R = 3,
    O_V = 6,
    O_BA = 9,
    O_BG = 12
};

enum NoiseOrder
{
    O_AN = 0,
    O_GN = 3,
    O_AW = 6,
    O_GW = 9
};

}