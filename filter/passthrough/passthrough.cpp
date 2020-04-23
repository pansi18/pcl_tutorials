#include <iostream>
#include <ctime>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
int
main (int argc, char** argv)
{ 
  srand(time(0));
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  //生成并填充点云数据
  cloud->width  = 5;
  cloud->height = 1;
  cloud->points.resize (cloud->width * cloud->height);
  for (size_t i = 0; i < cloud->points.size (); ++i)
  {
    cloud->points[i].x = rand () / (RAND_MAX + 1.0f)-0.5;
    cloud->points[i].y = rand () / (RAND_MAX + 1.0f)-0.5;
    cloud->points[i].z = rand () / (RAND_MAX + 1.0f)-0.5;
  }
  std::cerr << "Cloud before filtering: " << std::endl;
  for (size_t i = 0; i < cloud->points.size (); ++i)
    std::cerr << "    " << cloud->points[i].x << " " 
                        << cloud->points[i].y << " " 
                        << cloud->points[i].z << std::endl;
  
  pcl::PassThrough<pcl::PointXYZ> pass;   //创建直通滤波器对象
  pass.setInputCloud (cloud);             //设置输入点云
  pass.setFilterFieldName ("z");          //设置过滤时所需点云类型的z字段
  pass.setFilterLimits (0.0, 1.0);        //设置在过滤字段上的范围
  //pass.setFilterLimitsNegative (true);  //设置保留范围外的点，默认为false,即保留范围内的点
  pass.filter (*cloud_filtered);          //执行滤波，结果保存在cloud_filtered中
    
  std::cerr << "Cloud after filtering: " << std::endl;
  for (size_t i = 0; i < cloud_filtered->points.size (); ++i)
    std::cerr << "    " << cloud_filtered->points[i].x << " " 
                        << cloud_filtered->points[i].y << " " 
                        << cloud_filtered->points[i].z << std::endl;
  return (0);
}
