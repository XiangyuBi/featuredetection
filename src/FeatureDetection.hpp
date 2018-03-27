//
// Created by Birkee on 2018/3/26.
//

#ifndef X_RAY_TRIAL_FEATUREDETECTION_HPP
#define X_RAY_TRIAL_FEATUREDETECTION_HPP

#include<pcl/io/ply_io.h>
#include <pcl/point_types.h>



class FeatureDetection {
    FeatureDetection() = default;
public:
    static pcl::PointCloud<pcl::PointXYZRGB>::Ptr readPLYFile(const char* plyname);



};

pcl::PointCloud<pcl::PointXYZRGB>::Ptr FeatureDetection::readPLYFile(const char *plyname) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new )
}


#endif //X_RAY_TRIAL_FEATUREDETECTION_HPP
