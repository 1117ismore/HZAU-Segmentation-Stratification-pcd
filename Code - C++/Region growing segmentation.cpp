#include <iostream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <windows.h>
#include <stdio.h>
#include <psapi.h>
void PrintMemoryInfo()
{
	HANDLE hProcess;
	PROCESS_MEMORY_COUNTERS pmc;

	hProcess = GetCurrentProcess();
	printf("\nProcess ID: %u\n", hProcess);

	

	if (NULL == hProcess)
		return;

	if (GetProcessMemoryInfo(hProcess, &pmc, sizeof(pmc)))
	{
		printf("\tPageFaultCount: 0x%08X\n", pmc.PageFaultCount);
		printf("\tPeakWorkingSetSize: 0x%08X\n",
			pmc.PeakWorkingSetSize);
		printf("\tWorkingSetSize: 0x%08X\n", pmc.WorkingSetSize);
		printf("\tQuotaPeakPagedPoolUsage: 0x%08X\n",
			pmc.QuotaPeakPagedPoolUsage);
		printf("\tQuotaPagedPoolUsage: 0x%08X\n",
			pmc.QuotaPagedPoolUsage);
		printf("\tQuotaPeakNonPagedPoolUsage: 0x%08X\n",
			pmc.QuotaPeakNonPagedPoolUsage);
		printf("\tQuotaNonPagedPoolUsage: 0x%08X\n",
			pmc.QuotaNonPagedPoolUsage);
		printf("\tPagefileUsage: 0x%08X\n", pmc.PagefileUsage);
		printf("\tPeakPagefileUsage: 0x%08X\n",
			pmc.PeakPagefileUsage);
	}

	CloseHandle(hProcess);
}

using namespace pcl::console;
int
main(int argc, char** argv)
{

	if (argc < 2)
	{
		std::cout << ".exe xx.pcd -kn 50 -bc 0 -fc 10.0 -nc 0 -st 30 -ct 0.05" << endl;

		return 0;
	}
	time_t start, end, diff[5], option;
	start = time(0);
	int K = 50;             
	bool Bool_Cuting = false;
	float far_cuting = 10, near_cuting = 0, SmoothnessThreshold = 30.0, CurvatureThreshold = 0.05;
	parse_argument(argc, argv, "-kn", K);
	parse_argument(argc, argv, "-bc", Bool_Cuting);
	parse_argument(argc, argv, "-fc", far_cuting);
	parse_argument(argc, argv, "-nc", near_cuting);
	parse_argument(argc, argv, "-st", SmoothnessThreshold);
	parse_argument(argc, argv, "-ct", CurvatureThreshold);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile <pcl::PointXYZ>(argv[1], *cloud) == -1)
	{
		std::cout << "Cloud reading failed." << std::endl;
		return (-1);
	}
	//  order£ºout_stem_1.pcd 5 10 0.3 10 10
	end = time(0);
	diff[0] = difftime(end, start);
	PCL_INFO("\Loading pcd file takes(seconds): %d\n", diff[0]);
	
	pcl::search::Search<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::PointCloud <pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	n.setSearchMethod(tree);  
	n.setInputCloud(cloud);   
	n.setKSearch(K);          
	n.compute(*normals);     
	end = time(0);
	diff[1] = difftime(end, start) - diff[0];
	PCL_INFO("\Estimating normal takes(seconds): %d\n", diff[1]);
	
	pcl::IndicesPtr indices(new std::vector <int>);
	if (Bool_Cuting)
	{
		pcl::PassThrough<pcl::PointXYZ> pass;
		pass.setInputCloud(cloud);           
		pass.setFilterFieldName("z");        
		pass.setFilterLimits(near_cuting, far_cuting);
		pass.filter(*indices);               
	}

	
	pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
	reg.setMinClusterSize(50);                         
	reg.setMaxClusterSize(1000000);                   
	reg.setSearchMethod(tree);                         
	reg.setNumberOfNeighbours(30);                     
	reg.setInputCloud(cloud);                         
	if (Bool_Cuting)reg.setIndices(indices);          
	reg.setInputNormals(normals);                      
	reg.setSmoothnessThreshold(SmoothnessThreshold / 180.0 * M_PI);
	reg.setCurvatureThreshold(CurvatureThreshold);     

	std::vector <pcl::PointIndices> clusters;
	reg.extract(clusters);                             
	end = time(0);
	diff[2] = difftime(end, start) - diff[0] - diff[1];
	PCL_INFO("\Region growing takes(seconds): %d\n", diff[2]);

	std::cout << "Number of clusters is equal to " << clusters.size() << std::endl;        
	std::cout << "First cluster has " << clusters[0].indices.size() << " points." << endl; 
	std::cout << "These are the indices of the points of the initial" <<
		std::endl << "cloud that belong to the first cluster:" << std::endl;
	
	/*int j = 0;
	for (std::vector<pcl::PointIndices>::const_iterator it = clusters.begin(); it != clusters.end(); ++it)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);

		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
			cloud_cluster->points.push_back(cloud->points[*pit]);
		cloud_cluster->width = cloud_cluster->points.size();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points."
			<< std::endl;
		std::stringstream ss;
		ss << "cloud_cluster_" << j << ".pcd";
		pcl::io::savePCDFileASCII(ss.str(), *cloud_cluster);
		cout << ss.str() << "Saved" << endl;
		j++;
	}

	*/
	PrintMemoryInfo();
	
	pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();
	
	pcl::io::savePCDFileASCII("colored_pointCloud.pcd",*colored_cloud);
	pcl::visualization::CloudViewer viewer("Region growing segmentation method");
	viewer.showCloud(colored_cloud);
	while (!viewer.wasStopped())
	{
	}

	return (0);
}
