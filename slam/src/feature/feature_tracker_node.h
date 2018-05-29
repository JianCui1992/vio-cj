#pragma once

#include "feature_tracker.h"

namespace VIO
{

class Feature_Tracker_Node
{
public:
	Feature_Tracker_Node();

	void init();
	void load_img_from_file(const string &path);
	//for real time 
	void load_img_from_cam();

public:
	vector<string> v_image_name;
	vector<string> v_time_stamp;

}



}

