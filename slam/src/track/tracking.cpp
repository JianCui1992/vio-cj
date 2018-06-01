#include "tracking.h"

namespace VIO
{
Tracking::Tracking(System *sys_, data_base *data_base_)
{
	msystem = sys_;
	mdata_base = data_base_;
    mestimator = new Estimator();
    init();
}
void Tracking::init()
{
	current_time = -1;
	init_imu = true;
    
    mestimator->setParameter();

}
void Tracking::predict(const imu_with_timestamp &imu_msg)
{	
	double t = imu_msg.timestamp;
	if(init_imu)
	{
		latest_time = t;
		init_imu = false;
		return;
	}
	double dt = t - latest_time;
	latest_time = t;
	double dx = imu_msg.linear_acceleration_x;
	double dy = imu_msg.linear_acceleration_y;
	double dz = imu_msg.linear_acceleration_z;

	Eigen::Vector3d linear_acceleration{dx,dy,dz};
    // std::cout<<"here?"<<std::endl;
    // getchar();
	double rx = imu_msg.angular_velocity_x;
	double ry = imu_msg.angular_velocity_y;
	double rz = imu_msg.angular_velocity_z;
	Eigen::Vector3d angular_velocity{rx, ry, rz};
    // std::cout<<"here?"<<std::endl;
    // getchar();
    // std::cout<<tmp_Q.x()<<std::endl;
    // std::cout<<acc_0.x()<<std::endl;
    // std::cout<<tmp_Ba.x()<<std::endl;
    // std::cout<<mestimator->g.x()<<std::endl;

    // getchar();
	Eigen::Vector3d un_acc_0 = tmp_Q * (acc_0 - tmp_Ba) - mestimator->g;
	Eigen::Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - tmp_Bg;
    // std::cout<<"here?"<<std::endl;
    // getchar();
    tmp_Q = tmp_Q * Utility::deltaQ(un_gyr * dt);

    Eigen::Vector3d un_acc_1 = tmp_Q * (linear_acceleration - tmp_Ba) - mestimator->g;
    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
    // std::cout<<"here?"<<std::endl;
    // getchar();
    tmp_P = tmp_P + dt * tmp_V + 0.5 * dt * dt * un_acc;
    tmp_V = tmp_V + dt * un_acc;

    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
}

void Tracking::update()
{
    TicToc t_predict;
    latest_time = current_time;
    tmp_P = mestimator->Ps[WINDOW_SIZE];
    tmp_Q = mestimator->Rs[WINDOW_SIZE];
    tmp_V = mestimator->Vs[WINDOW_SIZE];
    tmp_Ba = mestimator->Bas[WINDOW_SIZE];
    tmp_Bg = mestimator->Bgs[WINDOW_SIZE];
    acc_0 = mestimator->acc_0;
    gyr_0 = mestimator->gyr_0;

    std::queue<imu_with_timestamp> tmp_imu_buf = mdata_base->imu_buf;

    for (imu_with_timestamp tmp_imu_msg; !tmp_imu_buf.empty(); tmp_imu_buf.pop())
        predict(tmp_imu_buf.front());
}


void Tracking::run()
{
	while(true)
	{
        TicToc t_lock;
		std::vector<std::pair<std::vector<imu_with_timestamp>, feature_points>> measurements;
		std::unique_lock<std::mutex> lk(mdata_base->MutexMeasurements);
		mdata_base->con.wait(lk, [&]{ return mdata_base->getMeasurements(); });
		measurements = mdata_base->measurements;
        lk.unlock();
        // std::cout<<"a"<<std::endl;
        // printf("a\n");
        // int aaa = 1;


        MutexEstimator.lock();
        for(auto &measurement : measurements)
        {
        	auto img_msg = measurement.second;
        	double dx = 0, dy = 0, dz = 0, rx = 0, ry = 0, rz = 0;
        	for(auto &imu_msg : measurement.first)
        	{
        		double t = imu_msg.timestamp;
        		double img_t = img_msg.image_time.timestamp + TD;
        		if(t <= img_t)
        		{
        			if(current_time < 0)
        				current_time = t;
        			double dt = t - current_time;
        			if(dt < 0)
        			{
        				printf("Error!!! tracking.cpp line 47 !\n");
        				getchar();
        			}
        			current_time = t;
        			dx = imu_msg.linear_acceleration_x;
        			dy = imu_msg.linear_acceleration_y;
        			dz = imu_msg.linear_acceleration_z;

        			rx = imu_msg.angular_velocity_x;
        			ry = imu_msg.angular_velocity_y;
        			rz = imu_msg.angular_velocity_z;

        			mestimator->processIMU(dt,Eigen::Vector3d(dx,dy,dz),Eigen::Vector3d(rx,ry,rz));

        		}
        	}

        	//set relocalization frame, pose_graph 

        	// feature_points relo_msg = NULL;
        	// while(!mdata_base->relo_buf.empty())
        	// {
        	// 	relo_msg = mdata_base->relo_buf.front();
        	// 	relo_buf.pop();
        	// }
        	// if(relo_msg != NULL)
        	// {
        	// 	vector<Eigen::Vector3d> match_points;
        	// 	double frame_stamp = relo_msg.image_time.timestamp;
        	// 	for(unsigned int i = 0; i < relo_msg.v_p.size(); i++)
        	// 	{
        	// 		Eigen::Vector3d u_v_id;
        	// 		u_v_id.x() = relo_msg.v_p[i].x;
        	// 		u_v_id.y() = relo_msg.v_p[i].y;
        	// 		u_v_id.z() = relo_msg.v_p[i].z;
        	// 		match_points.push_back(u_v_id);
        	// 	}
        	// 	Eigen::Vector3d relo_t(relo_msg)

        	// }

        	TicToc t_s;
        	map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> image;
        	for(unsigned int i = 0; i < img_msg.v_p.size(); i++)
        	{
        		int v = img_msg.id[i] + 0.5;
        		int feature_id = v / NUM_OF_CAM;
        		int camera_id = v % NUM_OF_CAM;
        		double x = img_msg.v_p[i].x;
        		double y = img_msg.v_p[i].y;
        		double z = img_msg.v_p[i].z;

        		double p_u = img_msg.v_uv[i].x;
        		double p_v = img_msg.v_uv[i].y;

        		double velocity_x = img_msg.v_vel[i].x;
        		double velocity_y = img_msg.v_vel[i].y;

        		if(z != 1)
        		{
        			printf("Error ! tracking.cpp line 107 ! \n");
        			getchar();
        		}
        		Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
        		xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
        		image[feature_id].emplace_back(camera_id, xyz_uv_velocity);
        	}
        	mestimator->processImage(image,img_msg.image_time.timestamp);
        	double whole_t = t_s.toc();
        	
            //test
            // printf("%4.2f, %4.2f, %4.2f\n",mestimator->Ps[WINDOW_SIZE].x(),mestimator->Ps[WINDOW_SIZE].y(),mestimator->Ps[WINDOW_SIZE].z());

            // std::cout << mestimator->Ps[WINDOW_SIZE].x() << ","
            //   << mestimator->Ps[WINDOW_SIZE].y() << ","
            //   << mestimator->Ps[WINDOW_SIZE].z() << ","
              // << tmp_Q.w() << ","
              // << d.x() << ","
              // << tmp_Q.y() << ","
              // << tmp_Q.z() << ","
              // << mestimator->Vs[WINDOW_SIZE].x() << ","
              // << mestimator->Vs[WINDOW_SIZE].y() << ","
              // << mestimator->Vs[WINDOW_SIZE].z() << "," << std::endl;

        	//publish message for vis and reloc

            // printStatistics(mestimator, whole_t);
        	// pubOdometry(estimator, header);
         //    pubKeyPoses(estimator, header);
         //    pubCameraPose(estimator, header);
         //    pubPointCloud(estimator, header);
         //    pubTF(estimator, header);
         //    pubKeyframe(estimator);
         //    if (relo_msg != NULL)
         //        pubRelocalization(estimator);

        }

        MutexEstimator.unlock();
        

        mdata_base->MutexMeasurements.lock();
        mdata_base->MutexState.lock();
        if (mestimator->solver_flag == Estimator::SolverFlag::NON_LINEAR)
            update();
        mdata_base->MutexState.unlock();
        mdata_base->MutexMeasurements.unlock();
        
        std::cout<<t_lock.toc()<<std::endl;


	}
}



}