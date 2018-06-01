#pragma once

#include <queue>
#include <mutex>

#include "feature_tracker.h"
#include "common_include.h"
#include "data.h"
#include "system.h"

namespace VIO
{
class System;
class data_base;

class Feature_Tracker_Node
{
public:
	Feature_Tracker_Node(data_base *data_base_);
	void init();
	
	//for real time 
	void process_once(const image_with_timestamp &image_msg);
	void add_image_feature(image_with_timestamp im_, vector<cv::Point3f> &point3f, vector<cv::Point2f> &point2f, vector<cv::Point2f> & velocity ,vector<int> &id);

public:
	std::vector<FeatureTracker*> track_data;
	data_base *mdata_base;

	std::vector<std::string> v_image_name;
	std::vector<double> v_time_stamp;


	bool is_first;
	bool is_init_publish;
	double first_time;
	double last_time;

	int publish_counter;
	bool is_restart;
	bool is_pub_frame;

};




}

