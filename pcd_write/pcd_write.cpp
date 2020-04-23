#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int
main (int argc, char** argv)
{
  //创建点云
  pcl::PointCloud<pcl::PointXYZ> cloud;

  // Fill in the cloud data
  cloud.width    = 5;          //5个点
  cloud.height   = 1;          //无组织数据集
  cloud.is_dense = false;      //可能包含无效点（X、Y、Z值包含Inf/NaN值)
  cloud.points.resize (cloud.width * cloud.height);

  for (size_t i = 0; i < cloud.points.size (); ++i)
  {
    cloud.points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud.points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud.points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
  }

  //将点云存储为test_pcd.pcd文件，类似的函数如savePCDFileBinary、savePCDFile、savePCDFileBinaryCompressed
  pcl::io::savePCDFileASCII ("test_pcd.pcd", cloud);

  //输出点云信息到屏幕
  std::cerr << "Saved " << cloud.points.size () << " data points to test_pcd.pcd." << std::endl;

  for (size_t i = 0; i < cloud.points.size (); ++i)
    std::cerr << "    " << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << std::endl;

  return (0);
}
