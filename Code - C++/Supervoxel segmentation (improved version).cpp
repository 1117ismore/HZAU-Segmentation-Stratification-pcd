#include<iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/segmentation/supervoxel_clustering.h>

using namespace std;

int main() {

	string outpath("data");       
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::io::loadPCDFile("bunny.pcd", *cloud);

	
	float voxel_resultion = 10.0f;  
	float seed_resultion = 1.0f;     

	pcl::SupervoxelClustering<pcl::PointXYZRGB> super(voxel_resultion, seed_resultion);
	super.setInputCloud(cloud);      
	super.setNormalImportance(5);    
	super.setColorImportance(0);    
	super.setSpatialImportance(0.4); 
	std::map<uint32_t, pcl::Supervoxel<pcl::PointXYZRGB>::Ptr >supervoxl_clustering;
	super.extract(supervoxl_clustering);
	cout << "SuperVoxel Numbers£º" << supervoxl_clustering.size() << endl;

	
	pcl::PointCloud<pcl::PointXYZL>::Ptr supervoxel_cloud = super.getLabeledCloud();

	pcl::Indices pointIdxVec;      //Voxel_index  
	int begin = 0;
	pcl::PointCloud<pcl::PointXYZ>::Ptr seg_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	
	for (auto iter = supervoxl_clustering.cbegin(); iter != supervoxl_clustering.cend(); ++iter)
	{
		for (size_t i = 0; i < supervoxel_cloud->points.size(); ++i)
		{
			if (supervoxel_cloud->points[i].label == iter->first)
				pointIdxVec.push_back(i);
		}

		std::stringstream ss;
		ss << outpath << "//" << "super_block_" << begin + 1 << ".pcd";
		pcl::copyPointCloud(*cloud, pointIdxVec, *seg_cloud);
		pcl::io::savePCDFileBinary(ss.str(), *seg_cloud);
		cout << "No.[" << begin + 1 << "]over£¡" << endl;
		begin++;

		pointIdxVec.clear();
		seg_cloud->clear(); 
	}
	
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("VCCS"));
	viewer->setWindowName("SuperVoxel");
	viewer->addPointCloud(supervoxel_cloud, "supervoxel_cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "supervoxel_cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "supervoxel_cloud");
	
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(1000));
	}

	return 0;
}

