#include <iostream>
#include <Eigen/Core>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/file_io.h> 

using namespace std;
int main()
{
	string folder_path = "data";     
	string save_folder_path = "output";
	vector<string>PCD; 
	pcl::getAllPcdFilesInDirectory(folder_path, PCD); 
	
	for (int i = 0; i < PCD.size(); i++)
	{
		cout << "==============================================" << endl;
		
		cout << i + 1 << "/" << PCD.size() << endl;

		cout << "load file from:" << PCD[i].c_str() << endl;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::io::loadPCDFile(folder_path + "//" + PCD[i], *cloud);
		
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
		Eigen::Vector4f centroid; 
		pcl::compute3DCentroid(*cloud, centroid);	       
		pcl::demeanPointCloud(*cloud, centroid, *cloud_out); 
	
		string file_type = "_centre.pcd";
		string save_path = save_folder_path + "//" + PCD[i].substr(0, PCD[i].rfind(".")) + file_type;
		cout << "Save file in: " << save_path << endl;

		pcl::io::savePCDFileASCII(save_path, *cloud_out);
	}
	return 0;
}
