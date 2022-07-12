#include <iostream>
#include <algorithm>
#include <vector>
#include <pcl/io/pcd_io.h>  
#include <pcl/point_types.h>  
#include <pcl/kdtree/kdtree_flann.h>  
#include <pcl/filters/extract_indices.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;

int main()
{
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	if (pcl::io::loadPCDFile<pcl::PointXYZRGB>("*.pcd", *cloud) == -1)
	{
		PCL_ERROR("Cloudn't read file!");
		return -1;
	}
	
	pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
	kdtree.setInputCloud(cloud);
	vector<int> pointIdxRadiusSearch;          
	vector<float> pointRadiusSquaredDistance;  
	vector<int> total_index;
	float radius = 0.000001;
	/*If a point has more than one point within the 0.000001 field, it is considered to have duplicate points. Record the index of the duplicate point, because when this duplicate point is used as the query point to search in the future, this point will also be defined as a duplicate point at this time, but all points in pointIdxRadiusSearch are arranged in ascending order, so from the second point in pointIdxRadiusSearch The index to start recording, this ensures that only duplicate points are removed, keeping the first point*/
	for (size_t i = 0; i < cloud->size(); ++i)
	{
		pcl::PointXYZRGB searchPoint = cloud->points[i];

		if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
		{
			if (pointIdxRadiusSearch.size() != 1)
			{
				for (size_t j = 1; j < pointIdxRadiusSearch.size(); j++)
				{
					total_index.push_back(pointIdxRadiusSearch[j]);
				}
			}
		}
	}
	
	sort(total_index.begin(), total_index.end());
	total_index.erase(unique(total_index.begin(), total_index.end()), total_index.end());

	
	pcl::PointIndices::Ptr outliners(new pcl::PointIndices());
	outliners->indices.resize(total_index.size());
	for (size_t i = 0; i < total_index.size(); i++)
	{
		outliners->indices[i] = total_index[i];
	}
	cout << "Duplicate point cloud deleted£¡£¡£¡" << endl;
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::ExtractIndices<pcl::PointXYZRGB> extract;
	extract.setInputCloud(cloud);
	extract.setIndices(outliners);
	extract.setNegative(true);
	extract.filter(*cloud_filtered);
	cout << "original point cloud numbers£º" << cloud->points.size() << endl;
	cout << "deleted point cloud numbers:" << total_index.size() << endl;
	cout << "final point cloud numbers:" << cloud_filtered->points.size() << endl;
	
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color(cloud_filtered, 0, 255, 0); // green

	viewer->addPointCloud<pcl::PointXYZRGB>(cloud_filtered, single_color, "sample cloud");
	pcl::io::savePCDFileASCII<pcl::PointXYZRGB>("*.pcd", *cloud_filtered);
	
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

	return 0;
}
