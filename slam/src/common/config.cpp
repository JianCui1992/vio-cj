#include "config.h"

namespace VIO
{
Config::Config()
{

}
void Config::read_parameters(const std::string &filename)
{
	cv::FileStorage fs(filename, cv::FileStorage::READ);
	if(!fs.isOpened())
	{
		std::cerr << "ERROR: Wrong path to setting" << std::endl;
	}
	FREQ = fs["freq"];


}
}