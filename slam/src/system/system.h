#pragma once

#include <thread>

#include "feature_tracker_node.h"
#include "common_include.h"
#include "data.h"
// #include "tracking.h"
namespace VIO
{
class Tracking;
class data_base;
class Feature_Tracker_Node;

class System
{
public:
	System();
	void init();

public:
	
	Feature_Tracker_Node *mfeature_tracker_node;
	Tracking *mtracker_node;
	data_base *mdata_base;


	std::thread *img_callback_thread;
	std::thread *imu_callback_thread;
	std::thread *track_thread;

	
};


}
