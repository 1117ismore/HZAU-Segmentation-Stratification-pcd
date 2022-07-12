#include <iostream>
#include<pcl/io/pcd_io.h>
#include<pcl/point_types.h>
#include<pcl/common/pca.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace std;

int main()
{
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	if (pcl::io::loadPCDFile<pcl::PointXYZRGB>("*.pcd", *cloud) < 0)
	{
		PCL_ERROR("ERROR");
		return -1;
	}
	//---------------------------------PCA-------------------------------------
	pcl::PCA<pcl::PointXYZRGB>pca;
	pca.setInputCloud(cloud);
	//pca.getEigenValues(); 
	// pca.getEigenVectors()

	Eigen::RowVector3f V1 = pca.getEigenVectors().col(0);
	Eigen::RowVector3f V2 = pca.getEigenVectors().col(1);
	Eigen::RowVector3f V3 = pca.getEigenVectors().col(2);
	
	float m = V1[0], n = V1[1], p = V1[2];
	float x0 = pca.getMean()[0], y0 = pca.getMean()[1], z0 = pca.getMean()[2];
	cout << "The direction vector of the line is£º" << V1 << "\n"
		<< "\nThe coordinates of a point on the line are£º" << pca.getMean().head<3>().transpose() << endl;
	
	Eigen::Matrix<float, 2, 3>A;
	A.row(0) = V2;
	A.row(1) = V3;
	Eigen::Vector3f sigma = pca.getMean().head<3>();
	Eigen::Vector2f b = A * sigma;

	cout << "\nThe general formula for a straight line is£º\n"
		<< V2[0] << "x+(" << V2[1] << "y)+(" << V2[2] << "z)=" << b[0] << "\n"
		<< V3[0] << "x+(" << V3[1] << "y)+(" << V3[2] << "z)=" << b[1] << endl;
	
	pcl::visualization::PCLVisualizer viewer;
	viewer.addPointCloud(cloud, "cloud");  
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> line_color(cloud, 255, 255, 0);
	viewer.addPointCloud(cloud, line_color, "line");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "line");
	
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	
	coefficients->values.push_back(x0);
	coefficients->values.push_back(y0);
	coefficients->values.push_back(z0);
	coefficients->values.push_back(m);
	coefficients->values.push_back(n);
	coefficients->values.push_back(p);

	
	viewer.setWindowName("Least squares fit straight line in space");
	viewer.addLine(*coefficients, "line");
	viewer.spin();

	return 0;
}

