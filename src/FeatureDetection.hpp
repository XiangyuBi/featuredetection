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

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    if(pcl::io::loadPLYFile<pcl::PointXYZRGB>(plyname, *cloud) == -1)
        throw("Cannot open ply file...");

    std::cout << "Loaded "
              << cloud->width * cloud->height
              << " data points from test_pcd.pcd with the following fields: "
              << std::endl;

    for (size_t i = 0; i < cloud->points.size (); ++i)
        std::cout << "    " << cloud->points[i].x
                  << " "    << cloud->points[i].y
                  << " "    << cloud->points[i].z << std::endl;

    return cloud;

}


#endif //X_RAY_TRIAL_FEATUREDETECTION_HPP
