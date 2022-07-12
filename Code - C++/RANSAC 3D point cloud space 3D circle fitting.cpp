#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_circle3D.h> 
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

using namespace std;

int main()
{
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile("data//*.pcd", *cloud) < 0)
	{
		PCL_ERROR("Couldn't read file \n");
		return -1;
	}

	  
	pcl::SampleConsensusModelCircle3D<pcl::PointXYZ>::Ptr model_circle3D(new pcl::SampleConsensusModelCircle3D<pcl::PointXYZ>(cloud));
	pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_circle3D);
	ransac.setDistanceThreshold(0.005);	       
	ransac.setMaxIterations(100);		       
	ransac.computeModel();				        
	pcl::IndicesPtr inliers(new vector <int>());
	ransac.getInliers(*inliers);			   

	
	pcl::PointCloud<pcl::PointXYZ>::Ptr circle_3D(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud<pcl::PointXYZ>(*cloud, *inliers, *circle_3D);

	Eigen::VectorXf coeff;
	ransac.getModelCoefficients(coeff);

	cout << "Circle_X£º" << coeff[0] << "\n"
		<< "Circle_Y£º" << coeff[1] << "\n"
		<< "Circle_Z£º" << coeff[2] << "\n"
		<< "Circle_Point£º" << coeff[3] << "\n"
		<< "Circle_Normal£º" << coeff[4] << ","
		<< coeff[5] << ","
		<< coeff[6] << endl;
	
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Viewer 3D Cloud"));
	
	viewer->addPointCloud<pcl::PointXYZ>(cloud, "cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 1, "cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
	
	viewer->addPointCloud<pcl::PointXYZ>(circle_3D, "circle3D");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "circle3D");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "circle3D");

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}


	return 0;
}
