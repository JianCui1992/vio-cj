#pragma once
#include <eigen3/Eigen/Dense>
#include <iostream>
#include "imu_factor.h"
#include "utility.h"
#include <map>
#include "feature_manager.h"
namespace VIO
{

class ImageFrame
{
    public:
        ImageFrame(){};
        ImageFrame(const std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 7, 1>>>>& _points, double _t):t{_t},is_key_frame{false}
        {
            points = _points;
        };
        std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 7, 1>> > > points;
        double t;
        Eigen::Matrix3d R;
        Eigen::Vector3d T;
        IntegrationBase *pre_integration;
        bool is_key_frame;
};

bool VisualIMUAlignment(std::map<double, ImageFrame> &all_image_frame, Eigen::Vector3d* Bgs, Eigen::Vector3d &g, Eigen::VectorXd &x);

}