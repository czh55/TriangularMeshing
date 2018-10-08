//https://blog.csdn.net/wkxxuanzijie920129/article/details/51404396
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <pcl/io/PLY_io.h>

int main(int argc, char** argv)
{
	// Load input file into a PointCloud<T> with an appropriate type
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("bunny.pcd", *cloud) == -1)//打开点云文件
	{
		PCL_ERROR("Couldn't read file mypointcloud.pcd\n"); //文件名要写对
		return -1;
	}
	//sensor_msgs::PointCloud2 cloud_blob;
	//pcl::io::loadPCDFile ("bun0.pcd", cloud_blob);
	//pcl::fromROSMsg (cloud_blob, *cloud);
	//* the data should be available in cloud

	// Normal estimation（法向量估计）
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud);
	n.setInputCloud(cloud);
	n.setSearchMethod(tree);
	n.setKSearch(20);
	n.compute(*normals); //法线
	//* normals should not contain the point normals + surface curvatures（不能同时包含点的法向量和表面的曲率）
	// Concatenate the XYZ and normal fields （将点云和法线放在一起）
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
	// Create search tree*
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud(cloud_with_normals);
	// Initialize objects （初始化对象）
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	//创建多边形网格，用于存储结果
	pcl::PolygonMesh triangles; 
	//设置参数
	gp3.setSearchRadius(1.5); // 设置连接点之间的最大距离（最大边长）用于确定k近邻的球半径（默认为0）
	gp3.setMu(2.5); // 设置最近邻距离的乘子，已得到每个点的最终搜索半径（默认为0）
	gp3.setMaximumNearestNeighbors(100); //设置搜索的最近邻点的最大数量
	gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees 最大平面角
	gp3.setMinimumAngle(M_PI / 18); // 10 degrees 每个三角的最大角度
	gp3.setMaximumAngle(2 * M_PI / 3); // 120 degrees
	gp3.setNormalConsistency(false); //若法向量一致，设为true
									  // 设置搜索方法和输入点云
	gp3.setInputCloud(cloud_with_normals);
	gp3.setSearchMethod(tree2);
	//执行重构，结果保存在triangles中
	gp3.reconstruct(triangles);
	//保存网格图
	//pcl::io::saveVTKFile("mymesh.vtk", triangles); //这句可以没有
	pcl::io::savePLYFile("mesh_result.ply", triangles);
	// Additional vertex information
	std::vector<int> parts = gp3.getPartIDs();
	std::vector<int> states = gp3.getPointStates();
	fstream fs;
	fs.open("partsID.txt", ios::out);
	if (!fs)
	{
		return -2;
	}
	fs << "点云数量为：" << parts.size() << "\n";
	for (int i = 0; i < parts.size(); i++)
	{
		if (parts[i] != 0)
		{
			fs << parts[i] << "\n"; //这的fs对吗？
		}
	}
	//显示结果图
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0); //设置背景
	viewer->addPolygonMesh(triangles, "my"); //设置显示的网格
	viewer->addCoordinateSystem(1.0); //设置坐标系
	viewer->initCameraParameters();
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	// Finish
	return 0;
}
