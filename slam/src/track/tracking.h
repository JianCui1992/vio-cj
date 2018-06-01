#pragma once

#include "data.h"
#include "system.h"
#include "common_include.h"
#include "estimator.h"


namespace VIO
{
class System;
class data_base;
class Estimator;

class Tracking
{
public:
	Tracking(System *sys_,data_base *data_base_);
	void predict(const imu_with_timestamp &imu_msg);
	void update();
	void run();
	void init();

public:
	data_base *mdata_base;
	System* msystem;
	Estimator *mestimator;


	
	std::mutex MutexEstimator;


	int sum_of_wait;
	bool init_imu;

	double last_imu_t;
	double current_time;
	double latest_time;
	
	Eigen::Vector3d tmp_P;
	Eigen::Quaterniond tmp_Q;
	Eigen::Vector3d tmp_V;
	Eigen::Vector3d tmp_Ba;
	Eigen::Vector3d tmp_Bg;
	Eigen::Vector3d acc_0;
	Eigen::Vector3d gyr_0;
	
};

}