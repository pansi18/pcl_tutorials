#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>    //采样一致性的方法
#include <pcl/sample_consensus/model_types.h>     //分割模型的头文件
#include <pcl/segmentation/sac_segmentation.h>    //ransac分割法
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
int
main (int argc, char** argv)
{
  pcl::PCLPointCloud2::Ptr cloud_blob (new pcl::PCLPointCloud2), cloud_filtered_blob (new pcl::PCLPointCloud2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>), cloud_p (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
  // 创建点云读取对象
  pcl::PCDReader reader;
  reader.read ("table_scene_lms400.pcd", *cloud_blob);
  std::cerr << "PointCloud before filtering: " << cloud_blob->width * cloud_blob->height << " data points." << std::endl;
  
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;     //创建体素栅格下采样对象
  sor.setInputCloud (cloud_blob);              //设置下采样原始点云数据
  sor.setLeafSize (0.01f, 0.01f, 0.01f);       //设置采样的体素大小
  sor.filter (*cloud_filtered_blob);           //执行采样并保存数据

  //将点云由 pcl::PCLPointCloud2 格式转换为 pcl::PointCloud 格式
  pcl::fromPCLPointCloud2 (*cloud_filtered_blob, *cloud_filtered);
  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;
  
  // 将采样后的点云写入磁盘
  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZ> ("table_scene_lms400_downsampled.pcd", *cloud_filtered, false);

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  // 创建分割对象
  pcl::SACSegmentation<pcl::PointXYZ> seg;    
  seg.setOptimizeCoefficients (true);      // 设置对估计的模型参数进行优化处理
  seg.setModelType (pcl::SACMODEL_PLANE);  // 设置分割模型类别
  seg.setMethodType (pcl::SAC_RANSAC);     // 设置用哪个随机参数估计方法
  seg.setMaxIterations (1000);             // 设置最大迭代次数
  seg.setDistanceThreshold (0.01);         // 设置判断是否为模型内点的距离阈值

  // 设置Extraction filter的实际参数
  pcl::ExtractIndices<pcl::PointXYZ> extract;     //创建点云提取对象
  int i = 0, nr_points = (int) cloud_filtered->points.size ();
  // µ±»¹ÓÐ30%Ô­ÊŒµãÔÆÊýŸÝÊ±
  while (cloud_filtered->points.size () > 0.3 * nr_points)
  {
    // ŽÓÓàÏÂµÄµãÔÆÖÐ·Öžî×îŽóÆœÃæ×é³É²¿·Ö
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }
    
    extract.setInputCloud (cloud_filtered);      //设置输入点云
    extract.setIndices (inliers);                //设置分割后的内点为需要提取的点集
    extract.setNegative (false);                 //设置提取内点而非外点
    extract.filter (*cloud_p);                   //执行提取，并存储到cloud_p

    std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;
    std::stringstream ss;
    ss << "table_scene_lms400_plane_" << i << ".pcd";
    writer.write<pcl::PointXYZ> (ss.str (), *cloud_p, false);
    // 设置提取外点
    extract.setNegative (true);
    extract.filter (*cloud_f);
    cloud_filtered.swap (cloud_f);
    i++;
  }
  return (0);
}
