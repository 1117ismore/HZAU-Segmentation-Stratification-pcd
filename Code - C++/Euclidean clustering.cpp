#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree.h>                 
#include <pcl/segmentation/extract_clusters.h> 
#include <pcl/visualization/pcl_visualizer.h> 
#include <boost/thread/thread.hpp>

using namespace std;
using PointT = pcl::PointXYZRGB;

int
main(int argc, char** argv)
{
    
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    if (pcl::io::loadPCDFile <PointT>("*.pcd", *cloud) == -1)
    {
        PCL_ERROR("Cloud reading failed.");
        return (-1);
    }

    
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);
    vector<pcl::PointIndices> cluster_indices; 
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(0.002);              
    ec.setMinClusterSize(100);              
    ec.setMaxClusterSize(100000);            
    ec.setSearchMethod(tree);                
    ec.setInputCloud(cloud);                  
    ec.extract(cluster_indices);            

   
    boost::shared_ptr<pcl::visualization::PCLVisualizer>viewer(new pcl::visualization::PCLVisualizer("planar segment")); ;
    viewer->setBackgroundColor(0, 0, 0);
    viewer->setWindowName("Euclidean clustering");

    int begin = 0;
    for (auto it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);
        for (auto pit = it->indices.begin(); pit != it->indices.end(); pit++)
        {
            cloud_cluster->push_back(cloud->points[*pit]);
        }

        std::stringstream ss;
        ss << "Euclidean_cluster_" << begin + 1 << ".pcd";
        pcl::io::savePCDFileBinary(ss.str(), *cloud_cluster);
        cout << ss.str() << "save finish£¡£¡£¡" << endl;
        begin++;
        
        uint8_t R = rand() % (256) + 0;
        uint8_t G = rand() % (256) + 0;
        uint8_t B = rand() % (256) + 0;
        string str;
        ss >> str;
        pcl::visualization::PointCloudColorHandlerCustom<PointT> color(cloud_cluster, R, G, B);
        viewer->addPointCloud<PointT>(cloud_cluster, color, str);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, str);
    }

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
    return (0);
}


