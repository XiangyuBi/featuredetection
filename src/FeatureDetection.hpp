//
// Created by Birkee on 2018/3/26.
//

#ifndef X_RAY_TRIAL_FEATUREDETECTION_HPP
#define X_RAY_TRIAL_FEATUREDETECTION_HPP

#include "VisUtil.hpp"
#include <pcl/features/normal_3d.h>
#include <pcl/range_image/range_image.h>
#include <pcl/range_image/range_image_planar.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/keypoints/iss_3d.h>
//#include <synchapi.h>

class NarfFeature{
public:
    
    NarfFeature() = delete;
    explicit NarfFeature(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    NarfFeature& keyPointExtractor();

private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;


};




NarfFeature &NarfFeature::keyPointExtractor() {
    pcl::PointCloud<int>::Ptr keypoints(new pcl::PointCloud<int>);
    int imageSizeX = 640, imageSizeY = 480;
    float centerX = (640.0f / 2.0f), centerY = (480.0f / 2.0f);
    float focalLengthX = 525.0f, focalLengthY = focalLengthX;
    Eigen::Affine3f sensorPose = Eigen::Affine3f(Eigen::Translation3f(cloud->sensor_origin_[0],
                                                                      cloud->sensor_origin_[1],
                                                                      cloud->sensor_origin_[2])) *
                                 Eigen::Affine3f (cloud->sensor_orientation_);
    float noiseLevel = 0.0f, minimumRange = 0.0f;
    pcl::RangeImagePlanar rangeImage;
    rangeImage.createFromPointCloudWithFixedSize(*cloud, imageSizeX, imageSizeY,
                                                 centerX, centerY, focalLengthX, focalLengthX,
                                                 sensorPose, pcl::RangeImage::CAMERA_FRAME,
                                                 noiseLevel, minimumRange);

    pcl::RangeImageBorderExtractor borderExtractor;
    // Keypoint detection object.
    pcl::NarfKeypoint detector(&borderExtractor);
    detector.setRangeImage(&rangeImage);
    // The support size influences how big the surface of interest will be,
    // when finding keypoints from the border information.
    detector.getParameters().support_size = 0.2f;

    detector.compute(*keypoints);

    // Visualize the keypoints.
    pcl::visualization::RangeImageVisualizer viewer("NARF keypoints");
    viewer.showRangeImage(rangeImage);
    for (size_t i = 0; i < keypoints->points.size(); ++i)
    {
        viewer.markPoint(keypoints->points[i] % rangeImage.width,
                         keypoints->points[i] / rangeImage.width,
                // Set the color of the pixel to red (the background
                // circle is already that color). All other parameters
                // are left untouched, check the API for more options.
                         pcl::visualization::Vector3ub(1.0f, 0.0f, 0.0f));
    }

    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
        // Sleep 100ms to go easy on the CPU.
        pcl_sleep(0.1);
    }
    return *this;
}

NarfFeature::NarfFeature(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
        : cloud(cloud)
{
}


class ISSFeature
{
public:
    ISSFeature() = delete;
    explicit ISSFeature(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    ISSFeature& viewISSFeatures();
private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    double computeCloudResolution(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud);

};

ISSFeature::ISSFeature(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
    : cloud(cloud)
{}

double ISSFeature::computeCloudResolution(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud) {
    double resolution = 0.0;
    int numberOfPoints = 0;
    int nres;
    std::vector<int> indices(2);
    std::vector<float> squaredDistances(2);
    pcl::search::KdTree<pcl::PointXYZ> tree;
    tree.setInputCloud(cloud);

    for (size_t i = 0; i < cloud->size(); ++i)
    {
        if (! pcl_isfinite((*cloud)[i].x))
            continue;

        // Considering the second neighbor since the first is the point itself.
        nres = tree.nearestKSearch(i, 2, indices, squaredDistances);
        if (nres == 2)
        {
            resolution += sqrt(squaredDistances[1]);
            ++numberOfPoints;
        }
    }
    if (numberOfPoints != 0)
        resolution /= numberOfPoints;

    return resolution;
}

ISSFeature &ISSFeature::viewISSFeatures() {
    pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZ> detector;
    detector.setInputCloud(cloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
    detector.setSearchMethod(kdtree);
    double resolution = computeCloudResolution(cloud);
    // Set the radius of the spherical neighborhood used to compute the scatter matrix.
    detector.setSalientRadius(6 * resolution);
    // Set the radius for the application of the non maxima supression algorithm.
    detector.setNonMaxRadius(4 * resolution);
    // Set the minimum number of neighbors that has to be found while applying the non maxima suppression algorithm.
    detector.setMinNeighbors(5);
    // Set the upper bound on the ratio between the second and the first eigenvalue.
    detector.setThreshold21(0.975);
    // Set the upper bound on the ratio between the third and the second eigenvalue.
    detector.setThreshold32(0.975);
    // Set the number of prpcessing threads to use. 0 sets it to automatic.
    detector.setNumberOfThreads(4);
    detector.compute(*keypoints);

    VisUtil::visualizeWithFeture(cloud, keypoints);

    return *this;
}



//<<==================================================================================================================>>

class FeatureDetection {

public:
    FeatureDetection() = delete;
    explicit FeatureDetection(const char* plyname);
    FeatureDetection& viewRGB();
    FeatureDetection& viewNarfKeyPoints();
    FeatureDetection& viewISSKeyPoints();



	FeatureDetection& viewPoints();

private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr origin;

};


FeatureDetection::FeatureDetection(const char *plyname) {
    try {
		std::cout << "Reading PLY:" << plyname << std::endl;
        origin = VisUtil::readPLY_XYZ(plyname);
    }
    catch(const char*)
    {
        std::cout << "Cannot Open ply file: " << plyname << std::endl;}
}

FeatureDetection &FeatureDetection::viewPoints()
{
	for(size_t i = 0; i < origin->points.size(); ++i)
	{
		std::cout << "Point coor: " 
				<< origin->points[i].x << " "
				<< origin->points[i].y << " "
				<< origin->points[i].z << std::endl;
	}
	return *this;
}


FeatureDetection &FeatureDetection::viewRGB() {
    VisUtil::visualize(origin);
    return *this;
}

FeatureDetection &FeatureDetection::viewNarfKeyPoints() {
    auto narf = std::shared_ptr<NarfFeature>(new NarfFeature(origin));
    narf->keyPointExtractor();
    return *this;

}

FeatureDetection &FeatureDetection::viewISSKeyPoints() {
    auto iss = std::shared_ptr<ISSFeature>(new ISSFeature(origin));
    iss->viewISSFeatures();
    return *this;
}


#endif //X_RAY_TRIAL_FEATUREDETECTION_HPP
