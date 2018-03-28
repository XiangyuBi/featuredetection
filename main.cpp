#include "src/FeatureDetection.hpp"
#include <string>

using namespace std;

int main()
{
	string file =  "/home/xib008/workspace/X-ray-project/test.ply";
	auto cloud = FeatureDetection::readPLYFile(file.c_str());
		
	return 0;
}
