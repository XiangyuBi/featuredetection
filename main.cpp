#include "src/FeatureDetection.hpp"
#include <string>

using namespace std;

int main()
{
	string file =  "/home/xib008/workspace/X-ray-project/test.ply";
    auto f = FeatureDetection(file.c_str());
    f.viewRGB();

	return 0;
}
