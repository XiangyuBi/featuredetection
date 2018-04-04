#include "src/FeatureDetection.hpp"
#include <string>



int main()
{
	std::string file =  "/home/xib008/workspace/X-ray-project/globalnrregistrationbaseline/Dec_0004filtered.ply";
	std::cout << "T" << std::endl;
	auto f = FeatureDetection(file.c_str());
//	std::cout << "Read fly" << std::endl;
	f.viewNarfKeyPoints();

	return 0;
}
