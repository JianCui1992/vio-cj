#include "feature_tracker_node.h"

namespace VIO
{

Feature_Tracker_Node::Feature_Tracker_Node()
{
	init();
}

void Feature_Tracker_Node::load_img_from_file(const string &path)
{
	ifstream fTimes;

	cv::FileStorage fpath(path, cv::FileStorage::READ);

    if(!fpath.isOpened())
    {
       cerr << "Failed to open settings file at: " << path << endl;
       exit(-1);
    }

	string path_ = fpath["image_path"];

    string strPathTimeFile = path_ + "ini/times.txt";

    fTimes.open(strPathTimeFile.c_str());

    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;

            double t;
            ss  >> t;
            v_time_stamp.push_back(t);
        }
    }

    // cout<<v_time_stamp[0]<<endl;

    ifstream fimage;
    string strPathImageFile = path_ + "ini/image_name.txt";
 
    fimage.open(strPathImageFile.c_str());
    
    string strPrefixLeft = path_ + "image_capturer_0/";
    
    const int nTimes = v_time_stamp.size();

    v_image_name.resize(nTimes);

    int i = 0;
    while(!fimage.eof())
    {
      string s;
      getline(fimage,s);
      if(!s.empty())
      {
        v_image_name[i] = strPrefixLeft + s;
      }
      ++i;
    }
    printf("The image load over, total %ld .\n",v_image_name.size());
}


}