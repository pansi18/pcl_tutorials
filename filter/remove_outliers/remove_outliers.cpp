#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
int
main (int argc, char** argv)
{
  if (argc != 2)
  {
    std::cerr << "please specify command line arg '-r' or '-c'" << std::endl;
    exit(0);
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  // 填充点云数据
  cloud->width  = 10;
  cloud->height = 1;
  cloud->points.resize (cloud->width * cloud->height);
  for (size_t i = 0; i < cloud->points.size (); ++i)
  {
    cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
  }

  //判断输入的命令行参数
  if (strcmp(argv[1], "-r") == 0)
  {
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;     //创建滤波器
    outrem.setInputCloud(cloud);             //设置输入点云
    outrem.setRadiusSearch(0.8);             //设置在0.8半径的范围内找邻近点
    outrem.setMinNeighborsInRadius (2);      //设置查询点的邻近点集数小于2的删除
    outrem.filter (*cloud_filtered);         //执行条件滤波，存储结果到cloud_filtered
  }
  else if (strcmp(argv[1], "-c") == 0)
  {
    // 创建条件限定下的滤波器
    pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond (new
      pcl::ConditionAnd<pcl::PointXYZ> ());     //创建条件定义对象
    //为条件定义对象添加比较算子
    //添加在 z 字段上大于0的比较算子（GT,即 greater than)
    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
      pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, 0.0)));
    //添加在 z 字段上小于0.8的比较算子（LT,即 less than)
    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
      pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::LT, 0.8)));
    // 创建滤波器并用条件定义对象初始化
    pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
    condrem.setCondition(range_cond);
    condrem.setInputCloud (cloud);          //设置输入点云
    condrem.setKeepOrganized(true);         //设置保持点云的结构
    condrem.filter (*cloud_filtered);       //执行条件滤波，存储结果到cloud_filtered
  }
  else
  {
    std::cerr << "please specify command line arg '-r' or '-c'" << std::endl;
    exit(0);
  }

  //输出滤波前的点云
  std::cerr << "Cloud before filtering: " << std::endl;
  for (size_t i = 0; i < cloud->points.size (); ++i)
    std::cerr << "    " << cloud->points[i].x << " "
                        << cloud->points[i].y << " "
                        << cloud->points[i].z << std::endl;
  // 输出滤波后点云
  std::cerr << "Cloud after filtering: " << std::endl;
  for (size_t i = 0; i < cloud_filtered->points.size (); ++i)
    std::cerr << "    " << cloud_filtered->points[i].x << " "
                        << cloud_filtered->points[i].y << " "
                        << cloud_filtered->points[i].z << std::endl;
  return (0);
}
