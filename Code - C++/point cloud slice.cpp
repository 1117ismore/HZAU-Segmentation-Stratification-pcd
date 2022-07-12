#include <iostream>
#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

using namespace std;


inline Eigen::Vector4d cal_plane(const pcl::PointXYZ& a, const pcl::PointXYZ& b, const pcl::PointXYZ& c)
{
	Eigen::Vector4d param;
	Eigen::Vector3d p1, p2, p3, p1p2, p1p3, N, N1;
	p1 << a.x, a.y, a.z;
	p2 << b.x, b.y, b.z;
	p3 << c.x, c.y, c.z;
	p1p2 = p2 - p1;
	p1p3 = p3 - p1;

	N = p1p2.cross(p1p3); 
	N1 = N / N.norm();    

	param[0] = N1[0];
	param[1] = N1[1];
	param[2] = N1[2];
	param[3] = -N1.dot(p1);
	return param;
}

int
main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("data//*.pcd", *cloud);
	cout << "point cloud" << cloud->points.size() << endl;
	
	pcl::PointXYZ pa = { -5.5266,-0.939036,2.313929 };
	pcl::PointXYZ pb = { 6.23236,-0.629336,3.029729 };
	pcl::PointXYZ pc = { 2.39399,-0.977036,-4.596331 };
	Eigen::Vector4d n;
	n = cal_plane(pa, pb, pc);

	cout << "Plane equation coefficients:\n" << "a=" << n[0] << "\tb=" << n[1] <<
		"\tc=" << n[2] << "\td=" << n[3] << endl;

	
	double Delta = 0.02;
	vector<int>point_idx;
	for (int i = 0; i < cloud->size(); i++)
	{
		double Wr = n[0] * (*cloud)[i].x + n[1] * (*cloud)[i].y + n[2] * (*cloud)[i].z + n[3] - Delta;
		double Wl = n[0] * (*cloud)[i].x + n[1] * (*cloud)[i].y + n[2] * (*cloud)[i].z + n[3] + Delta;
		if (Wr * Wl <= 0)
		{
			point_idx.push_back(i);
		}
	}
	pcl::PointCloud<pcl::PointXYZ>::Ptr slicing_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(*cloud, point_idx, *slicing_cloud);
	
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setWindowName("Clip");
	viewer->addPointCloud<pcl::PointXYZ>(slicing_cloud, "slicing cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 0, "slicing cloud");
	viewer->setBackgroundColor(1, 1, 1);
	pcl::io::savePCDFileASCII("slice.pcd", *slicing_cloud);
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

	return 0;
}

