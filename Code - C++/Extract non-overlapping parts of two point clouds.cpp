#include<pcl/io/pcd_io.h>
#include<pcl/kdtree/io.h>
#include <pcl/filters/extract_indices.h>
#include<pcl/registration/correspondence_estimation.h>

using namespace std;

int main()
{

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr source(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr target(new pcl::PointCloud<pcl::PointXYZRGB>);

	
	pcl::io::loadPCDFile<pcl::PointXYZRGB>("A.pcd", *source);
	pcl::io::loadPCDFile<pcl::PointXYZRGB>("B.pcd", *target);
	
	pcl::registration::CorrespondenceEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>core;
	core.setInputSource(source);
	core.setInputTarget(target);
	boost::shared_ptr<pcl::Correspondences> cor(new pcl::Correspondences);
	core.determineReciprocalCorrespondences(*cor, 0.1);   

	vector<int>pointIdxVec_A;
	vector<int>pointIdxVec_B;

	pcl::registration::getQueryIndices(*cor, pointIdxVec_A); 
	pcl::registration::getMatchIndices(*cor, pointIdxVec_B);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr non_overlapA(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr non_overlapB(new pcl::PointCloud<pcl::PointXYZRGB>);
	int non_A = target->points.size() - pointIdxVec_B.size();
	int non_B = source->points.size() - pointIdxVec_A.size();
	if (non_A < non_B) {

		pcl::ExtractIndices<pcl::PointXYZRGB> extrA;
		extrA.setInputCloud(source);
		extrA.setIndices(std::make_shared<vector<int>>(pointIdxVec_A));//index
		extrA.setNegative(true);   
		extrA.filter(*non_overlapA);
		pcl::io::savePCDFileASCII("C.pcd", *non_overlapA);
	}
	else {
		pcl::ExtractIndices<pcl::PointXYZRGB> extrB;
		extrB.setInputCloud(target);
		extrB.setIndices(std::make_shared<vector<int>>(pointIdxVec_B));
		extrB.setNegative(true);
		extrB.filter(*non_overlapB);

		pcl::io::savePCDFileASCII("error.pcd", *non_overlapB);

	};

	return 0;
}
