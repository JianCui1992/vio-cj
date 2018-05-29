#include "system.h"

namespace VIO
{
System::System()
{
	init();
}

void System::init()
{
	printf("System init\n");
	mfeature_tracker_node = new Feature_Tracker_Node();
	
}

}
