#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>     //随机参数估计方法头文件
#include <pcl/sample_consensus/model_types.h>      //模型定义头文件
#include <pcl/segmentation/sac_segmentation.h>     //基于采样一致性分割的类的头文件
int
main (int argc, char** argv)
{
  //创建点云并填充数据
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.width  = 15;
  cloud.height = 1;
  cloud.points.resize (cloud.width * cloud.height);
  //采样随机数据填充点云的x,y坐标，但都处在z为1的平面上
  for (size_t i = 0; i < cloud.points.size (); ++i)
  {
    cloud.points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud.points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud.points[i].z = 1.0;
  }

  //设置几个局外点，即重新设置几个点的z值，使其偏离z为1的平面
  cloud.points[0].z = 2.0;
  cloud.points[3].z = -2.0;
  cloud.points[6].z = 4.0;
  //在标准输出上打印出点云中各点的坐标值，方便分割后的参考
  std::cerr << "Point cloud data: " << cloud.points.size () <<" points" <<  std::endl;
  for (size_t i = 0; i < cloud.points.size (); ++i)
    std::cerr << "    " << cloud.points[i].x << " " 
  << cloud.points[i].y << " " 
  << cloud.points[i].z << std::endl;

  //创建分割时所需要的模型系数对象coefficients，存储内点的点索引集合对象inliers
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  //创建分割对象
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  //可选配置，设置模型系数需要优化
  seg.setOptimizeCoefficients (true);
  //必须配置，设置分割的模型类别、所用的随机参数估计方法、距离阈值、输入点云
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);
  seg.setInputCloud (cloud.makeShared ());
  //执行分割，存储分割结果到点集合inliers，存储平面模型的系数coefficients
  seg.segment (*inliers, *coefficients);

  
  if (inliers->indices.size () == 0)
  {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
return (-1);
  }
  //打印出估计的平面模型参数
  std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
  <<coefficients->values[1] << " "
  <<coefficients->values[2] << " " 
  <<coefficients->values[3] <<std::endl;
  std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
  for (size_t i = 0; i < inliers->indices.size (); ++i)
    std::cerr << inliers->indices[i] << "    " <<cloud.points[inliers->indices [i]].x << " "
  <<cloud.points[inliers->indices[i]].y << " "
  <<cloud.points[inliers->indices[i]].z << std::endl;
  return (0);
}
