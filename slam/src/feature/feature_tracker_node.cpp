#include "feature_tracker_node.h"

namespace VIO
{

Feature_Tracker_Node::Feature_Tracker_Node(data_base *data_base_)
{
	mdata_base = data_base_;
	init();
}
void Feature_Tracker_Node::init()
{
	track_data.reserve(NUM_OF_CAM);
	for(int i = 0; i < NUM_OF_CAM; i++)
	{
		// std::cout<<CAM_NAMES[i]<<std::endl;
		track_data.push_back(new FeatureTracker());
	}
	for(unsigned int i = 0; i < track_data.size(); i++)
	{
		track_data[i]->readIntrinsicParameter(CAM_NAMES[i]);
	}
	is_first = true;
}
void Feature_Tracker_Node::process_once(const image_with_timestamp &image_msg)
{
	if(is_first)
	{
		is_first = false;
		first_time = image_msg.timestamp;
		last_time = image_msg.timestamp;
		return;
	}

	if(image_msg.timestamp - first_time > 1.0 || image_msg.timestamp < last_time)
	{
		// std::cout << image_msg.timestamp <<","<<first_time<<","<<last_time<<std::endl;
		std::cerr << "Discontinue image, Reset feature tracker !" << std::endl;
		is_first = true;
		last_time = 0.0;
		publish_counter = 1;
		is_restart = true;
		return;
	}

	last_time = image_msg.timestamp;

	// frequency control
	if (round(1.0 * publish_counter / (image_msg.timestamp - first_time)) <= FREQ)
	{
		is_pub_frame = true;

		//reset the frequency control
		if(abs(1.0 * publish_counter / (image_msg.timestamp - first_time)) - FREQ < 0.01 * FREQ)
		{
			first_time = image_msg.timestamp;
			publish_counter = 0;
		}
	}
	else
	{
		is_pub_frame = false;
	}

	cv::Mat show_image = image_msg.image;
	// cv::imshow("test",show_image);
	// cv::waitKey(4);
	// getchar();

	TicToc t_r;
	for(int i = 0; i < NUM_OF_CAM; i++)
	{
		// printf("processing camera %d\n",i);
		if(i != 1 || !STEREO_TRACK)
		{
			track_data[i]->readImage(image_msg.image.rowRange(ROW * i, ROW * (i+1)), image_msg.timestamp);
		}
		else
		{
			if (EQUALIZE)
            {
                cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
                clahe->apply(image_msg.image.rowRange(ROW * i, ROW * (i + 1)), track_data[i]->cur_img);
            }
            else
                track_data[i]->cur_img = image_msg.image.rowRange(ROW * i, ROW * (i + 1));
		}
#if SHOW_UNDISTORTION
        track_data[i]->showUndistortion("undistrotion_" + std::to_string(i));
#endif
	}
	for(unsigned int i = 0; ;i++)
	{
		bool completed = false;
		for(int j = 0; j < NUM_OF_CAM; j++)
		{
			if(j != 1 || !STEREO_TRACK){
				completed |= track_data[j]->updateID(i);
			}
		}
		if(!completed)
			break;
	}
	if(is_pub_frame)
	{
		publish_counter++;

		for(int i = 0; i< NUM_OF_CAM; i++)
		{
			vector<cv::Point3f> v_feature_points;
	        vector<cv::Point2f> v_uv_of_point;
	        vector<cv::Point2f> v_velocity;
	        vector<int> v_id_of_point;

	        auto &un_pts = track_data[i]->cur_un_pts;
	        auto &cur_pts = track_data[i]->cur_pts;
	        auto &ids = track_data[i]->ids;
	        auto &pts_velocity = track_data[i]->pts_velocity;

	        for(unsigned int j = 0; j < ids.size(); j++)
	        {
	        	if(track_data[i]->track_cnt[j] > 1)
	        	{

	        		int p_id = ids[j];

	                cv::Point3f p;
	                p.x = un_pts[j].x;
	                p.y = un_pts[j].y;
	                p.z = 1.0;

	                cv::Point2f uv;
					uv.x = cur_pts[j].x;                
					uv.y = cur_pts[j].y;

					cv::Point2f vel;
					vel.x = pts_velocity[j].x;
					vel.y = pts_velocity[j].y;

					v_feature_points.push_back(p);
					v_uv_of_point.push_back(uv);
					v_velocity.push_back(vel);
	                v_id_of_point.push_back(p_id);

	                // printf("%f,%f,%f\n",p.x,p.y,p.z);
	        	}
	        }
		
			// skip the first image; since no optical speed on first image;
			if(is_init_publish)
			{
				is_init_publish = false;
			}
			else
			{
				add_image_feature(image_msg,v_feature_points,v_uv_of_point,v_velocity,v_id_of_point);
			}
	        if (SHOW_TRACK)
	        {
	            
	            cv::Mat stereo_img = image_msg.image;
	            cv::Mat show_img = image_msg.image;

	            cv::Mat tmp_img = stereo_img;
	            cv::cvtColor(show_img, tmp_img, CV_GRAY2RGB);

	            for (unsigned int j = 0; j < track_data[i]->cur_pts.size(); j++)
	            {
	                    double len = std::min(1.0, 1.0 * track_data[i]->track_cnt[j] / WINDOW_SIZE);
	                    cv::circle(tmp_img, track_data[i]->cur_pts[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
	                    //draw speed line

	                    Vector2d tmp_cur_un_pts (track_data[i]->cur_un_pts[j].x, track_data[i]->cur_un_pts[j].y);
	                    Vector2d tmp_pts_velocity (track_data[i]->pts_velocity[j].x, track_data[i]->pts_velocity[j].y);

	                    Vector3d tmp_prev_un_pts;
	                    tmp_prev_un_pts.head(2) = tmp_cur_un_pts - 0.10 * tmp_pts_velocity;
	                    tmp_prev_un_pts.z() = 1;
	                    Vector2d tmp_prev_uv;
	                    track_data[i]->m_camera->spaceToPlane(tmp_prev_un_pts, tmp_prev_uv);

	                    cv::line(tmp_img, track_data[i]->cur_pts[j], cv::Point2f(tmp_prev_uv.x(), tmp_prev_uv.y()), cv::Scalar(255 , 0, 0), 1 , 8, 0);
	                    
	                    char name[10];
	                    sprintf(name, "%d", track_data[i]->ids[j]);
	                    cv::putText(tmp_img, name, track_data[i]->cur_pts[j], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
	            }

	            cv::imshow("vis", tmp_img);
	            cv::waitKey(5);
	        }

        }

	}

}

void Feature_Tracker_Node::add_image_feature(image_with_timestamp im_, vector<cv::Point3f> &point3f, vector<cv::Point2f> &point2f, vector<cv::Point2f> & velocity ,vector<int> &id){
	if(im_.image.empty())
	{
		printf("Error add_image_feature !\n");
		return;
	}

	feature_points fp;
	fp.image_time.image = im_.image.clone();
	fp.image_time.timestamp = im_.timestamp;
	fp.v_p.swap(point3f);
	fp.v_uv.swap(point2f);
	fp.v_vel.swap(velocity);
	fp.id.swap(id);

	mdata_base->MutexMeasurements.lock();
	mdata_base->feature_buf.push(fp);
	// printf("img\n");
	mdata_base->MutexMeasurements.unlock();
	mdata_base->con.notify_one();

	return;
}


}