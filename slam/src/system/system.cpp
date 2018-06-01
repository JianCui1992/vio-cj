#include "system.h"

namespace VIO
{
System::System()
{
	mdata_base = new data_base(this);
	mtracker_node = new Tracking(this,mdata_base);
	mdata_base->set_tracker(mtracker_node);
	// mdata_base->load_img_from_file();
	init();

	img_callback_thread = new thread(&VIO::data_base::load_img_from_file, mdata_base);

	imu_callback_thread = new thread(&VIO::data_base::load_imu_from_file, mdata_base);

	mtracker_node->run();
	
	getchar();
}

void System::init()
{
	printf("System init\n");
}

}
