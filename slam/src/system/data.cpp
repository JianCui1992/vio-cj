#include "data.h"

namespace VIO
{
data_base::data_base(System *sys_)
{
	msystem = sys_;
	init();
}

void data_base::init()
{
	mfeature_tracker_node = new Feature_Tracker_Node(this);
	sum_of_wait = 0;
	last_img_time = 0.0;
	last_imu_time = 0.0;
	is_first_img = true;
	is_first_imu = true;
}

void data_base::set_tracker(Tracking *tracker_)
{
	mtracker = tracker_;
}
void data_base::load_img_from_file()
{

    std::ifstream fimage;
    std::string strPathImageFile = IMAGE_PATH + "image_name.txt"; 

    //std::cout<< strPathImageFile <<std::endl;

    fimage.open(strPathImageFile.c_str());
    std::string strPrefixLeft = IMAGE_PATH ;

    int i = 0;
    while(!fimage.eof())
    {
		std::string s;
		getline(fimage,s);
		if(!s.empty())
		{
			v_image_name.push_back(strPrefixLeft + s);

			std::stringstream stime(s.substr(0,17));
			double time;
			stime >> time;
			v_time_stamp.push_back(time - TIME_SHIFT);

		}
		++i;
    }
    printf("The image load over, total %ld .\n",v_image_name.size());

    for(unsigned int i = 0; i < v_image_name.size(); i++)
    {
    	TicToc t_img;
    	image_with_timestamp image_msg;
    	// //std::cout<< v_image_name[i] <<std::endl;
    	// getchar();
    	image_msg.image = cv::imread(v_image_name[i],CV_LOAD_IMAGE_UNCHANGED).clone();
    	image_msg.timestamp = v_time_stamp[i];
    	mfeature_tracker_node->process_once(image_msg);
    	double dt;
    	if(is_first_img)
    	{
    		dt = 0.01;
    		is_first_img = false;
    	}
    	else
    	{
	    	dt = v_time_stamp[i] - last_img_time - t_img.toc() / 1000;
    	}
    	// //std::cout<<"dt: "<<dt<<std::endl;
    	last_img_time = v_time_stamp[i];

    	if(dt > 0)
    	{
    		usleep(dt*1000000);
    	}
    	
    }
}
	
void data_base::load_imu_from_file()
{
	std::ifstream fimu(IMU_PATH);
	//std::cout << IMU_PATH << std::endl;
	if(fimu.is_open())
	{
		double  var0, var1, var2, var3, var4, var5, var6;
		imu_with_timestamp imu_;
		while(fimu >> var0 >> var1 >> var2 >>var3 >> var4 >> var5 >> var6)
		{

			TicToc t_imu;
			imu_.angular_velocity_x = var0;
			imu_.angular_velocity_y = var1;
			imu_.angular_velocity_z = var2;
			imu_.linear_acceleration_x = var3;
			imu_.linear_acceleration_y = var4;
			imu_.linear_acceleration_z = var5;
			imu_.timestamp = var6 - TIME_SHIFT;

			MutexMeasurements.lock();
			imu_buf.push(imu_);
			// printf("imu_\n");
			MutexMeasurements.unlock();
			con.notify_one();

			{
				std::lock_guard<std::mutex> lg(MutexState);
		        mtracker->predict(imu_);

		        //for vis and reloc
		        // std_msgs::Header header = imu_msg->header;
		        // header.frame_id = "world";
		        // if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
	         	//    pubLatestOdometry(tmp_P, tmp_Q, tmp_V, header);
    		}

    		double dt;
	    	if(is_first_imu)
	    	{
	    		dt = 0.01;
	    		is_first_imu = false;
	    	}
	    	else
	    	{
	    	
		    	dt = imu_.timestamp - last_imu_time - t_imu.toc() / 1000;
	    	}
	    	last_imu_time = imu_.timestamp;

	    	// //std::cout<<"dt: "<<dt<<std::endl;
	    	if(dt > 0)
	    	{
	    		usleep(dt*1000000);	
	    	}
	    	
		}
	}
}

bool data_base::getMeasurements()
{
	measurements.clear();
	while(true)
	{
		// //std::cout<<measurements.size()<<std::endl;
		if(imu_buf.empty() || feature_buf.empty())
			return true;
		if(measurements.size() >= 5)
		{
			//std::cout<<"I'm here e"<<std::endl;
			return true;
		}
		//wait for imu, only happened for real time.
		if(!(imu_buf.back().timestamp > feature_buf.front().image_time.timestamp + TD))
		{
			//std::cout<<"I'm here a"<< std::endl;
			sum_of_wait++;
			return false;
		}
		//imu should input first, otherwise throw img, only happend at the beginning.

		if(!(imu_buf.front().timestamp < feature_buf.front().image_time.timestamp + TD))
		{	
			//std::cout<<"imu timestamp: "<<imu_buf.front().timestamp<<" "<<"feature_buf timestamp: "<<feature_buf.front().image_time.timestamp << " TD: "<<TD<<std::endl;

			//std::cout<<"I'm here b"<< std::endl;
			feature_buf.pop();
			continue;
		}
		feature_points img_msg = feature_buf.front();
		feature_buf.pop();

		std::vector<imu_with_timestamp> imus;
		while(imu_buf.front().timestamp < img_msg.image_time.timestamp + TD)
		{
			//std::cout<<"imu timestamp: "<<imu_buf.front().timestamp<<" "<<"feature_buf timestamp: "<<img_msg.image_time.timestamp << " TD: "<<TD<<std::endl;

			//std::cout<<"I'm here f"<<std::endl;
			imus.emplace_back(imu_buf.front());
			imu_buf.pop();
		}
		//std::cout<<"I'm here z"<<std::endl;

		imus.emplace_back(imu_buf.front());
		if(imus.empty())
			printf("There is no imu message between two image");
		measurements.emplace_back(imus,img_msg);
	}

	if(measurements.empty())
	{
		//std::cout<<"I'm here c"<<std::endl;
		return false;
	}
	else
	{
		//std::cout<<"I'm here d"<<std::endl;
		return true;
	}
}

void image_callback()
{
	//pass
}
void imu_callback()
{
	//pass
}



}