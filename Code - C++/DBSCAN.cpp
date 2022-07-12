#include <iostream>
#include<string>
#include"DBSCAN.h"
#include<pcl/io/pcd_io.h>
#include<pcl/point_cloud.h>
#include<pcl/common/time.h>
#include<pcl/visualization/cloud_viewer.h>

using namespace std;
 
int main()
{
	// --------------------------------read data-----------------------------------
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	if (pcl::io::loadPCDFile<pcl::PointXYZ>("Z1A.pcd", *cloud) < 0)
	{
		PCL_ERROR("Failed to read point cloud£¡£¡£¡ \n");
		return -1;
	}
	cout << "Read from point cloud data£º" << cloud->points.size() << "points" << endl;
	// -------------------------------density clustering------------------------------------
	pcl::StopWatch time;
	vector<pcl::Indices> cluster_indices;
	dbscan(*cloud, cluster_indices, 2, 50); //2 means that the domain distance of the cluster is 2 meters, and 50 means the minimum number of points for the cluster.

	cout << "The number of density clusters is£º" << cluster_indices.size() << endl;
	cout << "code runtime:" << time.getTimeSeconds() << "second" << endl;
	// ---------------------------Clustering results are classified and saved--------------------------------
	int begin = 1;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr dbscan_all_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	for (vector<pcl::Indices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	{
		// Get the points of each clustered point cloud
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_dbscan(new pcl::PointCloud<pcl::PointXYZRGB>);
		// The same point cloud is assigned the same color
		uint8_t R = rand() % (256) + 0;
		uint8_t G = rand() % (256) + 0;
		uint8_t B = rand() % (256) + 0;

		for (auto pit = it->begin(); pit != it->end(); ++pit)
		{
			pcl::PointXYZRGB point_db;
			point_db.x = cloud->points[*pit].x;
			point_db.y = cloud->points[*pit].y;
			point_db.z = cloud->points[*pit].z;
			point_db.r = R;
			point_db.g = G;
			point_db.b = B;
			cloud_dbscan->points.push_back(point_db);
		}
		// Clustering results are classified and saved
		stringstream ss;
		ss << "dbscan_cluster_" << begin << ".pcd";
		pcl::PCDWriter writer;
		writer.write<pcl::PointXYZRGB>(ss.str(), *cloud_dbscan, true);
		begin++;

		*dbscan_all_cloud += *cloud_dbscan;
	}
	// -------------------------------visualization----------------------------------
	pcl::visualization::CloudViewer viewer("DBSCAN cloud viewer.");
	viewer.showCloud(dbscan_all_cloud);
	while (!viewer.wasStopped())
	{

	}
	return 0;
}

