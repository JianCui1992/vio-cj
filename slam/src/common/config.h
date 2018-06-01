#pragma once

namespace VIO
{
class Config
{
public:
	Config();
	void read_parameters(const std::string &filename);
	int COL;
	int FOCAL_LENGTH;
	int NUM_OF_CAM = 1;

	int MAX_CNT;
	int MIN_DIST;
	int START;
	int END;

	int WINDOW_SIZE;
	int FREQ;

	double F_THRESHOLD;
	int SHOW_TRACK;
	int STEREO_TRACK;
	int EQUALIZE;

	int FISHEYE;
	bool PUB_THIS_FRAME;

};

}