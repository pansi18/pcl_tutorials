#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>

// 可视化统计滤波后的点云
boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
    //创建视窗对象并给标题栏定义一个名称“3D Viewer”,将它定义为boost:shared_ptr智能共享指针
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    //将点云添加到视窗对象中并定义一个唯一的字符串作为ID号
    viewer->addPointCloud<pcl::PointXYZ> (cloud, "filtered cloud");
    //改变显示点云的尺寸
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "filtered cloud");
    //显示坐标系统方向，用X(红)、Y(绿)、Z(蓝)圆柱体代表坐标轴，圆柱体的大小设置为1.0
    viewer->addCoordinateSystem (1.0);
    //设置相机参数，使用户从默认的角度和方向观察点云
    viewer->initCameraParameters ();
    return (viewer);
}

int main (int argc, char** argv)
{ 
    float z0 = 0.1, z1 = 0.2;
    std::cout << "Please input the height intervals for sampling:[##.##]\n";
    std::cin >> z0 >> z1;
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ> cloud_filtered;
  
    // 读取点云文件
    if(pcl::io::loadPCDFile<pcl::PointXYZ>("robot_residential.pcd",*cloud) == -1)
    {
		PCL_ERROR("Couldn't read file robot_residential.pcd\n");
		return(-1);
    }
    std::cout<< "Loading the pcd file is done." << std::endl;

    float min_z = 0.0;
    float max_z = 0.0;

    for(size_t i = 0; i < cloud->points.size(); ++i)
    {
        if(cloud->points[i].z < min_z)
		    min_z = cloud->points[i].z;
	    if(cloud->points[i].z > max_z)
		    max_z = cloud->points[i].z;
    }
	std::cout << "Before filtering: " << cloud->points.size() << " points \n"
        << "The minimum z: " << min_z << std::endl
		<< "The maximum z: " << max_z << std::endl;

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);        //设置待滤波的点云
    sor.setMeanK(50);                //邻近点个数设为50
    sor.setStddevMulThresh(1.0);     //标准差倍数设为1，即如果一个点的距离超出平均距离的一个标准差以上，则该点被标记为离群点，被移除
    sor.filter(cloud_filtered);

    pcl::io::savePCDFileASCII ("filtered_robot_resdential.pcd", cloud_filtered);
    std::cout<< "Saving the filtered_robot_resdential.pcd is done." << std::endl;

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    viewer = simpleVis(cloud_filtered.makeShared());

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }

    min_z = 0;
    max_z = 0;

    for(size_t i = 0; i < cloud_filtered.points.size(); ++i)
    {
        if(cloud_filtered.points[i].z < min_z)
		    min_z = cloud_filtered.points[i].z;
	    if(cloud_filtered.points[i].z > max_z)
		    max_z = cloud_filtered.points[i].z;
    }
	std::cout << "After filtering: " << cloud_filtered.points.size() << " points \n"
        << "The minimum z: " << min_z << std::endl
		<< "The maximum z: " << max_z << std::endl;

    pcl::PassThrough<pcl::PointXYZ> pass;   //创建直通滤波器对象
	pass.setInputCloud(cloud_filtered.makeShared());             //设置输入点云
	pass.setFilterFieldName("z");          //设置过滤时所需点云类型的z字段
	pass.setFilterLimits(min_z + z0, min_z + z1);        //设置在过滤字段上的范围
	//pass.setFilterLimitsNegative (true);  //设置保留范围外的点，默认为false,即保留范围内的点
	pass.filter(cloud_filtered);          //执行滤波，结果保存在cloud_filtered中
    
    for(size_t i = 0; i < cloud_filtered.points.size(); ++i)
    {
        cloud_filtered.points[i].z = 0;
    }

    pcl::io::savePCDFileASCII ("profile.pcd", cloud_filtered);
    std::cout<< "Writting the profile.pcd is done." << std::endl;

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }    

    return (0);
}