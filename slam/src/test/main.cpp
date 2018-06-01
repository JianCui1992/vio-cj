#include "system.h"
#include "common_include.h"

int main(int argc, char** argv)
{
	VIO::read_parameters(argv[1]);
	
	VIO::System *SLAM = new VIO::System();


}