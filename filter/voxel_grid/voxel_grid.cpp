#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

int
main (int argc, char** argv)
{
  pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
  pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());
  // 点云读取对象
  pcl::PCDReader reader;
  // 将路径改为自己存放文件的路径或将该文件与产生的可执行文件放在同一目录下
  reader.read ("table_scene_lms400.pcd", *cloud);   // 读取点云文件中的数据到cloud对象
  std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height 
       << " data points (" << pcl::getFieldsList (*cloud) << ").";
  
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;     //创建滤波对象
  sor.setInputCloud (cloud);                        //设置需要过滤的点云
  sor.setLeafSize (0.01f, 0.01f, 0.01f);            //设置滤波时创建的体素体积为1cm3的立方体
  sor.filter (*cloud_filtered);           //执行滤波处理，存储结果到cloud_filtered中
  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 
       << " data points (" << pcl::getFieldsList (*cloud_filtered) << ").";
  //将数据写入磁盘以供其他使用
  pcl::PCDWriter writer;
  writer.write ("table_scene_lms400_downsampled.pcd", *cloud_filtered, 
         Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);
  return (0);
}
