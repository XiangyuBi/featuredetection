//
// Created by Birkee on 2018/3/26.
//

#ifndef X_RAY_TRIAL_FEATUREDETECTION_HPP
#define X_RAY_TRIAL_FEATUREDETECTION_HPP

#include<pcl/io/ply_io.h>
#include <pcl/point_types.h>



class FeatureDetection {
    FeatureDetection() = delete;
    explicit FeatureDetection(const char* plyname);

public:
    static pcl::PointCloud<pcl::PointXYZRGBA>::Ptr readPLYFile(const char* plyname);

private:
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr origin;


};

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr FeatureDetection::readPLYFile(const char *plyname) {

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    if(pcl::io::loadPLYFile<pcl::PointXYZRGBA>(plyname, *cloud) == -1)
        throw("Cannot open ply file...");

    std::cout << "Loaded "
              << cloud->width * cloud->height
              << " data points from test.ply with the following fields: "
              << std::endl;
/*
    for (size_t i = 0; i < cloud->points.size (); ++i)
        std::cout << "    " << cloud->points[i].x
                  << " "    << cloud->points[i].y
                  << " "    << cloud->points[i].z << std::endl;
*/
    return cloud;

}

FeatureDetection::FeatureDetection(const char *plyname) {
    origin = readPLYFile(plyname);
}


#endif //X_RAY_TRIAL_FEATUREDETECTION_HPP
