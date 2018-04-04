//
// Created by Birkee on 2018/3/28.
//

#ifndef X_RAY_TRIAL_VISUTIL_HPP
#define X_RAY_TRIAL_VISUTIL_HPP

#include<pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <memory>
#include <iostream>
#include <chrono>
#include <thread>

unsigned int text_id = 0;

struct VisUtil
{
   // unsigned int text_id = 0;
    static pcl::PointCloud<pcl::PointXYZRGBA>::Ptr readPLY_RGB(const char* plyname);
    static pcl::PointCloud<pcl::PointXYZ>::Ptr readPLY_XYZ(const char* plyname);
    static std::shared_ptr<pcl::visualization::PCLVisualizer> visualizeXYZCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);
    static std::shared_ptr<pcl::visualization::PCLVisualizer> visualizeRGBCloud(pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud);
    static void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void* viewer_void);
    static void mouseEventOccurred (const pcl::visualization::MouseEvent &event, void* viewer_void);
    static std::shared_ptr<pcl::visualization::PCLVisualizer> interactionCustomizationVis();
    static void visualize(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    static void visualizeWithFeture(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                    const pcl::PointCloud<pcl::PointXYZ>::Ptr &keypoints);


};

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr VisUtil::readPLY_RGB(const char *plyname){

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    if(pcl::io::loadPLYFile<pcl::PointXYZRGBA>(plyname, *cloud) == -1)
	 {       throw("Cannot open ply file...");}

    std::cout << "Loaded "
              << cloud->width * cloud->height
              << " data points from test.ply"
              << std::endl;

    return cloud;

}



std::shared_ptr<pcl::visualization::PCLVisualizer>
VisUtil::visualizeRGBCloud(pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud) {
    std::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(cloud);

    // Color handle object, it will get the RGB color fields from each point for the viewer to use

    viewer->addPointCloud<pcl::PointXYZRGBA>(cloud, rgb, "Point Cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3 , "Point Cloud");
 //   viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    return (viewer);
}

void VisUtil::keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void *viewer_void) {
    pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
    if (event.getKeySym () == "r" && event.keyDown ())
    {
        std::cout << "r was pressed => removing all text" << std::endl;

        char str[512];
        for (unsigned int i = 0; i < text_id; ++i)
        {
            sprintf (str, "text#%03d", i);
            viewer->removeShape (str);
        }
        text_id = 0;
    }
}

void VisUtil::mouseEventOccurred(const pcl::visualization::MouseEvent &event, void *viewer_void) {
    pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
    if (event.getButton () == pcl::visualization::MouseEvent::LeftButton &&
        event.getType () == pcl::visualization::MouseEvent::MouseButtonRelease)
    {
        std::cout << "Left mouse button released at position (" << event.getX () << ", " << event.getY () << ")" << std::endl;
        char str[512];
        sprintf (str, "text#%03d", text_id ++);
        viewer->addText ("PointClicked", event.getX (), event.getY (), str);
    }
}

std::shared_ptr<pcl::visualization::PCLVisualizer> VisUtil::interactionCustomizationVis() {
    std::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addCoordinateSystem (1.0);
    viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)viewer.get ());
    viewer->registerMouseCallback (mouseEventOccurred, (void*)viewer.get ());

    return (viewer);}

void VisUtil::visualize(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    
	using namespace std::chrono_literals;
    auto viewer = visualizeXYZCloud(cloud);
    while(!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(10s);
    }
}

std::shared_ptr<pcl::visualization::PCLVisualizer>
VisUtil::visualizeXYZCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) {
    std::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    return (viewer);

}

void VisUtil::visualizeWithFeture(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                  const pcl::PointCloud<pcl::PointXYZ>::Ptr &keypoints) {
    using namespace std::chrono_literals;

    if (keypoints->points.size() == 0)
    {
        std::cerr << "No features to show" << std::endl;
        return;
    }
    auto viewer = visualizeXYZCloud(cloud);
    int idx = 0;
    for(size_t i = 0; i < keypoints->points.size(); ++i) {
     //   idx = keypoints->points[i];
        viewer->addSphere(keypoints->points[i], 0.2, 0.5, 0.5, 0.0, "sphere");
    }

    while(!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(10s);
    }

}

pcl::PointCloud<pcl::PointXYZ>::Ptr VisUtil::readPLY_XYZ(const char *plyname) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if(pcl::io::loadPLYFile<pcl::PointXYZ>(plyname, *cloud) == -1)
    {       throw("Cannot open ply file...");}

    std::cout << "Loaded "
              << cloud->width * cloud->height
              << " data points from test.ply"
              << std::endl;

    return cloud;
}


#endif //X_RAY_TRIAL_VISUTIL_HPP
