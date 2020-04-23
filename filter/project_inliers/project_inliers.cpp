#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);
  // 填充点云并打印其内容
  cloud->width  = 5;
  cloud->height = 1;
  cloud->points.resize (cloud->width * cloud->height);
  for (size_t i = 0; i < cloud->points.size (); ++i)
  {
    cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
  }
  std::cerr << "Cloud before projection: " << std::endl;
  for (size_t i = 0; i < cloud->points.size (); ++i)
    std::cerr << "    " << cloud->points[i].x << " " 
                        << cloud->points[i].y << " " 
                        << cloud->points[i].z << std::endl;

  // 填充ModelCoefficients的值，使用ax+by+cz+d=0的平面模型，令a=b=d=0,c=1，即X-Y平面
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  coefficients->values.resize (4);
  coefficients->values[0] = coefficients->values[1] = 0;
  coefficients->values[2] = 1.0;
  coefficients->values[3] = 0;
  
  pcl::ProjectInliers<pcl::PointXYZ> proj;      //创建投影滤波对象
  proj.setModelType (pcl::SACMODEL_PLANE);      //设置对象对应的投影模型
  proj.setInputCloud (cloud);                   //设置输入点云
  proj.setModelCoefficients (coefficients);     //设置模型对应的系数
  proj.filter (*cloud_projected);            //执行投影滤波并存储结果到cloud_projected

  //打印出投影后的点云
  std::cerr << "Cloud after projection: " << std::endl;
  for (size_t i = 0; i < cloud_projected->points.size (); ++i)
    std::cerr << "    " << cloud_projected->points[i].x << " " 
                        << cloud_projected->points[i].y << " " 
                        << cloud_projected->points[i].z << std::endl;

  return (0);
}
