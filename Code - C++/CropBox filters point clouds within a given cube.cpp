#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;
int main()
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PCDReader reader;
    reader.read("data//*.pcd", *cloud);

    cout << "The points data:  " << cloud->points.size() << endl;
  
    pcl::CropBox<pcl::PointXYZRGB> crop;
    Eigen::Vector4f min_pt = { 0,0,0,0 };
    Eigen::Vector4f max_pt = { 40,40,40,0 };
    //Eigen::Vector3f direction_vector = {};   //Eigen::Vector3f  getRotation() const
    crop.setMin(min_pt);          
    crop.setMax(max_pt);           
    crop.setInputCloud(cloud);
    crop.setKeepOrganized(false);  
    crop.setUserFilterValue(0.1f);  
    //crop.setRotation(1,0,0);    
    /*
    void  setRotation(const Eigen::Vector3f & rotation)
        Eigen::Vector3f  getRotation() const
    */
    pcl::IndicesPtr indexes(new pcl::Indices());
    crop.filter(*indexes);        
   
    pcl::ExtractIndices<pcl::PointXYZRGB> extr;
    extr.setInputCloud(cloud);  
    extr.setIndices(indexes); 
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in_box(new pcl::PointCloud<pcl::PointXYZRGB>());
    extr.filter(*cloud_in_box); 
    cout << "The number of points in the CropBox is:" << cloud_in_box->points.size() << endl;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out_box(new pcl::PointCloud<pcl::PointXYZRGB>);
    extr.setNegative(true);   
    extr.filter(*cloud_out_box);
    cout << "The number of CropBox outer points is:" << cloud_out_box->points.size() << endl;
    
   // pcl::io::savePCDFileBinary("cloud_in_box.pcd", *cloud_in_box);
   // pcl::io::savePCDFileBinary("cloud_out_box.pcd", *cloud_out_box);
   
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setWindowName("CropBox filter");
    int v1 = 0, v2 = 1;
    viewer->createViewPort(0, 0, 0.5, 1, v1);
    viewer->createViewPort(0.5, 0, 1, 1, v2);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB>(cloud, "sample cloud", v1);
    
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> in(cloud_in_box, 255, 0, 0);
    
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> out(cloud_out_box, 0, 255, 0);
    viewer->addCube(min_pt[0], max_pt[0], min_pt[1], max_pt[1], min_pt[2], max_pt[2], 0.5, 0, 0.5, "cube", v2);
    viewer->setRepresentationToWireframeForAllActors(); 
    viewer->addPointCloud(cloud_in_box, in, "cloud_in_box", v2);
    viewer->addPointCloud(cloud_out_box, out, "offGround", v2);
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }

    return 0;
}

