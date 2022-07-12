#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

using namespace std;

int
main(int argc, char** argv)
{
	
	pcl::PointCloud<pcl::PointXYZ >::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ >);
	pcl::PCDReader reader;
	reader.read("*.pcd", *cloud);
	cout << "PointCloud has: " << cloud->points.size() << " data points." << endl;

	
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ >::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ >);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	n.setSearchMethod(tree);
	n.setInputCloud(cloud);
	n.setKSearch(10);
	n.compute(*normals);

	
	pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
	pcl::ModelCoefficients::Ptr coefficients_cylinder(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);

	seg.setOptimizeCoefficients(true);      
	seg.setModelType(pcl::SACMODEL_CYLINDER); 
	seg.setMethodType(pcl::SAC_RANSAC);       
	seg.setNormalDistanceWeight(0.1);       
	seg.setMaxIterations(500);              
	seg.setDistanceThreshold(0.025);     
	seg.setRadiusLimits(0, 0.03);            
	seg.setInputCloud(cloud);
	seg.setInputNormals(normals);
	
	seg.segment(*inliers_cylinder, *coefficients_cylinder);
	cout << "Cylinder coefficients: " << *coefficients_cylinder << endl;
	
	pcl::ExtractIndices<pcl::PointXYZ > extract;
	extract.setInputCloud(cloud);
	extract.setIndices(inliers_cylinder);
	extract.setNegative(false);
	pcl::PointCloud<pcl::PointXYZ >::Ptr cloud_cylinder(new pcl::PointCloud<pcl::PointXYZ >);
	extract.filter(*cloud_cylinder);
	if (cloud_cylinder->points.empty())
		cout << "Can't find the cylindrical component." << endl;
	else
	{
		pcl::PCDWriter writer;
		cout << "PointCloud representing the cylindrical component: " << cloud_cylinder->points.size() << " data points." << endl;
		writer.write("table_scene_mug_stereo_textured_cylinder.pcd", *cloud_cylinder, false);
	}

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

	viewer->addPointCloud<pcl::PointXYZ>(cloud_cylinder, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

	return 0;
}

