#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/file_io.h> 
using namespace std;
int main() {
    
    string folder_path = "source";      
    string save_folder_path = "target"; 
    vector<string>PCD;
    pcl::getAllPcdFilesInDirectory(folder_path, PCD); 
    
    
    for (int i = 0; i < PCD.size(); i++)
    {
        cout << "======================================================" << endl;
        
        cout << i + 1 << "/" << PCD.size() << endl;

        cout << "load file from:" << PCD[i].c_str() << endl;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::io::loadPCDFile(folder_path + "//" + PCD[i], *cloud);


        /*
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PCDReader reader;
        reader.read<pcl::PointXYZ>("L.pcd", *cloud);
        */
        
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);  //inliers
        pcl::SACSegmentation<pcl::PointXYZRGB> seg;
        seg.setOptimizeCoefficients(true);      
        seg.setModelType(pcl::SACMODEL_LINE);   
        seg.setMethodType(pcl::SAC_RANSAC);     
        seg.setDistanceThreshold(0.03);        
        seg.setMaxIterations(500);             
        seg.setInputCloud(cloud);              
        seg.segment(*inliers, *coefficients);  
        
        cout << "Plane equation coefficients:" << endl;
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
        //pcl::io::savePCDFile("l1.pcd", *line);  

        
        string file_type = "_stem.pcd";
        string save_path = save_folder_path + "//" + PCD[i].substr(0, PCD[i].rfind(".")) + file_type;
        cout << "Save file in: " << save_path << endl;

        pcl::io::savePCDFileASCII(save_path, *line);


        /*
        
        pcl::visualization::PCLVisualizer viewer;
        viewer.addPointCloud(cloud, "cloud");  
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> line_color(line, 255, 0, 0);
        viewer.addPointCloud(line, line_color, "line");
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "line");
        
        //viewer.addLine(*coefficients, "line");

        viewer.spin();
        */
    }
    return 0;
}


