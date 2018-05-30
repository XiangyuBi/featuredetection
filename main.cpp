#include "src/FeatureDetection.hpp"
#include <string>



int main(int argc, const char* argv[])
{
	
	std::string file = argv[1];  // "/home/xib008/workspace/X-ray-project/dataset_ori/scan50.obj";
//	std::cout << "T" << std::endl;
	auto f = FeatureDetection(file.c_str());
//	std::cout << "Read fly" << std::endl;
	f.viewISSKeyPoints();

	return 0;
}
