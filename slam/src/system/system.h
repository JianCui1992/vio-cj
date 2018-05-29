#pragma once


#include "feature_tracker_node.h"
#include "common_include.h"

namespace VIO
{

class System
{
public:
	System();
	void init();

public:
	Feature_Tracker_Node *mfeature_tracker_node;

	queue<feature_points> feature_buf;


}

}
