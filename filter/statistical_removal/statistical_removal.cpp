#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  // 定义读取对象
  pcl::PCDReader reader;
  // 读取点云文件
  reader.read<pcl::PointXYZ> ("table_scene_lms400.pcd", *cloud);
  std::cerr << "Cloud before filtering: " << std::endl;
  std::cerr << *cloud << std::endl;
  // 创建滤波器
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud);        //设置待滤波的点云
  sor.setMeanK (50);                //邻近点个数设为50
  sor.setStddevMulThresh (1.0);     //标准差倍数设为1，即如果一个点的距离超出平均距离的一个标准差以上，则该点被标记为离群点，被移除
  sor.filter (*cloud_filtered);
  std::cerr << "Cloud after filtering: " << std::endl;
  std::cerr << *cloud_filtered << std::endl;
  //定义写对象
  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZ> ("table_scene_lms400_inliers.pcd", *cloud_filtered, false);

  sor.setNegative (true);         //设置为获取离群点数据
  sor.filter (*cloud_filtered);
  writer.write<pcl::PointXYZ> ("table_scene_lms400_outliers.pcd", *cloud_filtered, false);
  return (0);
}
