#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h> 
#include <pcl/filters/project_inliers.h> 
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/time.h>

int main(int argc, char** argv)
{

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZRGB>);

    if (pcl::io::loadPCDFile("data//*.pcd", *cloud) == -1)
        return (-1);

   
//std::cerr << "Cloud before projection: " << std::endl;
//for (std::size_t i = 0; i < cloud->points.size(); ++i)
// std::cerr << " " << cloud->points[i].x << " "
// << cloud->points[i].y << " "
// << cloud->points[i].z << std::endl;


    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    coefficients->values.resize(4);
    coefficients->values[0] = 0;
    coefficients->values[1] = 1;
    coefficients->values[2] = 0;
    coefficients->values[3] = 0;


    // Create the filtering object
    pcl::ProjectInliers<pcl::PointXYZRGB> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);   
    proj.setInputCloud(cloud);
    proj.setModelCoefficients(coefficients);
    proj.filter(*cloud_projected);

    
    //std::cerr << "Cloud after projection: " << std::endl;
    //for (std::size_t i = 0; i < cloud_projected->points.size(); ++i)
    // std::cerr << " " << cloud_projected->points[i].x << " "
    // << cloud_projected->points[i].y << " "
    // << cloud_projected->points[i].z << std::endl;

    
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("test Viewer"));
    viewer->initCameraParameters();
    int v1(0), v2(0);
    
    viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer->setBackgroundColor(0, 0, 0, v1);
    viewer->addText("original", 10, 10, "v1 text", v1);
    viewer->addPointCloud<pcl::PointXYZRGB>(cloud, "sample cloud1", v1);
    viewer->addCoordinateSystem(1.0);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud1");
    
    viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    viewer->setBackgroundColor(0, 0, 0, v2);
    viewer->addText("cloud_projected", 10, 10, "v2 text", v2);
    viewer->addPointCloud<pcl::PointXYZRGB>(cloud_projected, "sample cloud2", v2);
    viewer->addCoordinateSystem(1.0);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud2");
    while (!viewer->wasStopped())
    {

        viewer->spinOnce(); 
        //boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
    system("pause");
    return 0;
}

