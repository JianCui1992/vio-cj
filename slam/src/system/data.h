#pragma once

#include "common_include.h"
#include "feature_tracker_node.h"
#include "system.h"
#include "tracking.h"

#include <stdio.h>
#include <mutex>
#include <queue>
#include <condition_variable>


namespace VIO
{
class Feature_Tracker_Node;
class System;

class data_base
{
public:
	data_base(System *sys_);
	void init();
	void set_tracker(Tracking *tracker_);
	//test version;
	void load_img_from_file();
	void load_imu_from_file();
	//real time version
	void image_callback();
	void imu_callback();

	bool getMeasurements();

	Feature_Tracker_Node *mfeature_tracker_node;
	System *msystem;
	Tracking *mtracker;

	std::mutex MutexMeasurements;
	std::mutex MutexState;

	std::condition_variable con;

	std::vector<std::string> v_image_name;
	std::vector<double> v_time_stamp;

	std::vector<std::pair<std::vector<imu_with_timestamp>, feature_points>> measurements;
	std::queue<feature_points> feature_buf;
	std::queue<imu_with_timestamp> imu_buf;
	std::queue<feature_points> relo_buf;


	double sum_of_wait;
	
	double last_img_time;
	double last_imu_time;

	bool is_first_img;
	bool is_first_imu;

};

}