#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace std;
int main() {
   
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PCDReader reader;
    reader.read<pcl::PointXYZRGB>("StemTest//*.pcd", *cloud);
   
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);  
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    seg.setOptimizeCoefficients(true);     
    seg.setModelType(pcl::SACMODEL_LINE);   
    seg.setMethodType(pcl::SAC_RANSAC);     
    seg.setDistanceThreshold(0.02);         
    seg.setMaxIterations(500);              
    seg.setInputCloud(cloud);               
    seg.segment(*inliers, *coefficients);  
    
    cout << "The model coefficients of the fitted line are:" << endl;
    cout << "a£º" << coefficients->values[0] << endl;
    cout << "b£º" << coefficients->values[1] << endl;
    cout << "c£º" << coefficients->values[2] << endl;
    cout << "d£º" << coefficients->values[3] << endl;
    cout << "e£º" << coefficients->values[4] << endl;
    cout << "f£º" << coefficients->values[5] << endl;

   
    /*method_1
    pcl::PointCloud<pcl::PointXYZ>::Ptr c_plane(new pcl::PointCloud<pcl::PointXYZ>);
    for (int i = 0; i < inliers->indices.size(); ++i) {
        c_plane->points.push_back(cloud->points.at(inliers->indices[i]));
    }
    */
   
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr line(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::ExtractIndices<pcl::PointXYZRGB> extract; 
    extract.setInputCloud(cloud);   
    extract.setIndices(inliers);    
    extract.setNegative(false);     
    extract.filter(*line);       
    pcl::io::savePCDFile("StemTest//*.pcd", *line);
    
    pcl::visualization::PCLVisualizer viewer;
    viewer.addPointCloud(cloud, "cloud");  
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> line_color(line, 255, 255, 0);
    viewer.addPointCloud(line, line_color, "line");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "line");
    viewer.setBackgroundColor(255, 255, 255);
   
    viewer.addLine(*coefficients, "line");
    viewer.spin();

    return 0;
}


