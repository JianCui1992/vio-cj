#include "common_include.h"

namespace VIO
{


std::string FISHEYE_MASK;

std::vector<std::string> CAM_NAMES;
int MAX_CNT;
int MIN_DIST;
int START;
int END;
int FREQ;
double F_THRESHOLD;
int SHOW_TRACK;
int STEREO_TRACK;
int EQUALIZE;
int FISHEYE;
bool PUB_THIS_FRAME;
std::string IMAGE_PATH;
std::string IMU_PATH;




double INIT_DEPTH;
double MIN_PARALLAX;
int ESTIMATE_EXTRINSIC;
std::string EX_CALIB_RESULT_PATH;


double ACC_N, ACC_W;
double GYR_N, GYR_W;

std::vector<Eigen::Matrix3d> RIC;
std::vector<Eigen::Vector3d> TIC;
Eigen::Vector3d G;

double BIAS_ACC_THRESHOLD;
double BIAS_GYR_THRESHOLD;

double SOLVER_TIME;
int NUM_ITERATIONS;


double TD;
double TR;
int ESTIMATE_TD;
int ROLLING_SHUTTER;
double ROW, COL;

// double FOCAL_LENGTH;
// int WINDOW_SIZE;
// int NUM_OF_CAM;
// int NUM_OF_F;


void read_parameters(const std::string &filename)
{
	cv::FileStorage fs(filename, cv::FileStorage::READ);
	if(!fs.isOpened())
	{
		std::cerr << "ERROR: Wrong path to setting" << std::endl;
	}

    // FOCAL_LENGTH = 460.0;
    // WINDOW_SIZE = 10;
    // NUM_OF_CAM = 1;
    // NUM_OF_F = 1000;

    
	FREQ = fs["freq"];

	IMAGE_PATH = (std::string)fs["image_path"];
	IMU_PATH = (std::string)fs["imu_path"];

    MAX_CNT = fs["max_cnt"];
    MIN_DIST = fs["min_dist"];

    ROW = fs["image_height"];
    COL = fs["image_width"];

    F_THRESHOLD = fs["F_threshold"];
    SHOW_TRACK = fs["show_track"];
    EQUALIZE = fs["equalize"];


    START = fs["start"];
    END = fs["end"];
    CAM_NAMES.push_back(filename);

    SOLVER_TIME = fs["max_solver_time"];
    NUM_ITERATIONS = fs["max_num_iterations"];
    MIN_PARALLAX = fs["keyframe_parallax"];
    MIN_PARALLAX = MIN_PARALLAX / FOCAL_LENGTH;

    ACC_N = fs["acc_n"];
    ACC_W = fs["acc_w"];
    GYR_N = fs["gyr_n"];
    GYR_W = fs["gyr_w"];
    G.z() = fs["g_norm"];
    ROW = fs["image_height"];
    COL = fs["image_width"];

    ESTIMATE_EXTRINSIC = fs["estimate_extrinsic"];
    std::string OUTPUT_PATH;
    fs["output_path"] >> OUTPUT_PATH;
    
    if (ESTIMATE_EXTRINSIC == 2)
    {
        printf("have no prior about extrinsic param, calibrate extrinsic param");
        RIC.push_back(Eigen::Matrix3d::Identity());
        TIC.push_back(Eigen::Vector3d::Zero());
        EX_CALIB_RESULT_PATH = OUTPUT_PATH + "/extrinsic_parameter.csv";

    }
    else 
    {
        if ( ESTIMATE_EXTRINSIC == 1)
        {
            printf(" Optimize extrinsic param around initial guess!");
            EX_CALIB_RESULT_PATH = OUTPUT_PATH + "/extrinsic_parameter.csv";
        }
        if (ESTIMATE_EXTRINSIC == 0)
            printf(" fix extrinsic param ");

        cv::Mat cv_R, cv_T;
        fs["extrinsicRotation"] >> cv_R;
        fs["extrinsicTranslation"] >> cv_T;
        Eigen::Matrix3d eigen_R;
        Eigen::Vector3d eigen_T;
        cv::cv2eigen(cv_R, eigen_R);
        cv::cv2eigen(cv_T, eigen_T);
        Eigen::Quaterniond Q(eigen_R);
        eigen_R = Q.normalized();
        RIC.push_back(eigen_R);
        TIC.push_back(eigen_T);
        // std::cout<<"Extrinsic_R : " << std::endl << RIC[0] << std::endl;
        // std::cout<<"Extrinsic_T : " << std::endl << TIC[0].transpose() << std::endl;
        
    } 

    INIT_DEPTH = 5.0;
    BIAS_ACC_THRESHOLD = 0.1;
    BIAS_GYR_THRESHOLD = 0.1;

    TD = fs["td"];
    ESTIMATE_TD = fs["estimate_td"];
    if (ESTIMATE_TD)
        // std::cout<<"Unsynchronized sensors, online estimate time offset, initial td: " << TD <<std::endl;
    else
        // std::cout<<"Synchronized sensors, fix time offset: " << TD <<std::endl;

    ROLLING_SHUTTER = fs["rolling_shutter"];
    if (ROLLING_SHUTTER)
    {
        TR = fs["rolling_shutter_tr"];
        // std::cout<<"rolling shutter camera, read out time per line: " << TR <<std::endl;
    }
    else
    {
        TR = 0;
    }


    STEREO_TRACK = false;
    PUB_THIS_FRAME = true;

    if (FREQ == 0)
        FREQ = 100;

    

    fs.release();

}


}
