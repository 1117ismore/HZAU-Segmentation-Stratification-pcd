#include <vector>
#include <iostream>
#include <algorithm>
#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include<pcl/visualization/pcl_plotter.h>

#pragma region 
//y=nx^n+(n-1)x^n-1+...+x+1
Eigen::VectorXd polyfit(const pcl::PointCloud<pcl::PointXY>::Ptr& cloud, int N = 1)
{
	int Row = cloud->points.size();
	
	Eigen::MatrixXd A = Eigen::MatrixXd::Ones(Row, N + 1);
	
	Eigen::VectorXd B(Row);
	
	for (size_t i = 0; i < cloud->points.size(); ++i)
	{
		B(i) = cloud->points[i].y;
		for (int n = N, dex = 0; n >= 1; --n, ++dex) 
		{
			A(i, dex) = pow(cloud->points[i].x, n);
		}
	}
	
	Eigen::VectorXd W;
	//W = (A.transpose() * A).inverse() * A.transpose() * B; 
	W = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(B); //SVD
	cout << "Fitting error: " << (A * W - B).norm() / B.norm() << endl;

	return W;
}
#pragma endregion
using namespace std;

int main() {

	pcl::PointCloud<pcl::PointXY>::Ptr cloud(new pcl::PointCloud<pcl::PointXY>);
	pcl::io::loadPCDFile<pcl::PointXY>("*.pcd", *cloud);

	Eigen::VectorXd L;
	L = polyfit(cloud, 7); 
	cout << "Fitting£º\n" << L << endl;
#pragma region
	
	pcl::visualization::PCLPlotter* plot = new pcl::visualization::PCLPlotter("My Plotter");
	
	std::vector<std::pair<double, double> >points;
	float Xmin = FLT_MAX;
	float Xmax = -FLT_MAX;

	for (auto point_i : *cloud)
	{
		points.push_back(std::make_pair(point_i.x, point_i.y));
		Xmin = min(Xmin, point_i.x); 
		Xmax = max(Xmax, point_i.x); 
	}
	// PCL£ºy=1+x+...(n-1)x^n-1+nx^n
	vector<double> func1;
	for (size_t i = 0; i < L.size(); ++i)
	{
		func1.push_back(L.reverse()[i]); // Eigen::reverse()

	}

	
	plot->addPlotData(points, "OriginPoints", vtkChart::POINTS);
	plot->addPlotData(func1, Xmin, Xmax, "polyfit");
	
	plot->setWindowName("Polynomial Curve Fitting");
	plot->setShowLegend(true);
	plot->setTitle("Polynomial curve fitting");
	plot->setXTitle("x");
	plot->setYTitle("y");
	
	plot->plot();
#pragma endregion
	return 0;
}

