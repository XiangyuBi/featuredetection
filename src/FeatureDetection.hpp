//
// Created by Birkee on 2018/3/26.
//

#ifndef X_RAY_TRIAL_FEATUREDETECTION_HPP
#define X_RAY_TRIAL_FEATUREDETECTION_HPP

#include "VisUtil.hpp"

class FeatureDetection {

public:
    FeatureDetection() = delete;
    explicit FeatureDetection(const char* plyname);
    FeatureDetection& viewRGB();


private:
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr origin;

};


FeatureDetection::FeatureDetection(const char *plyname) {
    try {
        origin = VisUtil::readPLYFile(plyname);
    }
    catch(const char*)
    {
        std::cout << "Cannot Open ply file: " << plyname << std::endl;}
}

FeatureDetection &FeatureDetection::viewRGB() {
    VisUtil::visualize(origin);
    return *this;
}


#endif //X_RAY_TRIAL_FEATUREDETECTION_HPP
