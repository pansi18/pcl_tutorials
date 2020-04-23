#include <iostream>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

boost::shared_ptr<pcl::visualization::PCLVisualizer>
simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  //viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}

int
main(int argc, char** argv)
{
	srand(time(NULL));
  // initialize PointClouds
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr final (new pcl::PointCloud<pcl::PointXYZ>);

  // populate our PointCloud with points
  cloud->width    = 5000;
  cloud->height   = 1;
  cloud->is_dense = false;
  cloud->points.resize (cloud->width * cloud->height);
  for (size_t i = 0; i < cloud->points.size (); ++i)
  {
    //根据命令行参数，用x2+y2+z2=1设置一部分点云数据，此时点云组成1/4个球体作为内点
    if (pcl::console::find_argument (argc, argv, "-s") >= 0 || pcl::console::find_argument (argc, argv, "-sf") >= 0)
    {
      cloud->points[i].x =  rand () / (RAND_MAX + 1.0);
      cloud->points[i].y =  rand () / (RAND_MAX + 1.0);
      if (i % 5 == 0)
        cloud->points[i].z =  rand () / (RAND_MAX + 1.0);    //此处对应的点为局外点
      else if(i % 2 == 0)
        cloud->points[i].z =  sqrt( 1 - (cloud->points[i].x * cloud->points[i].x)
                                      - (cloud->points[i].y * cloud->points[i].y));
      else
        cloud->points[i].z =  - sqrt( 1 - (cloud->points[i].x * cloud->points[i].x)
                                        - (cloud->points[i].y * cloud->points[i].y));
    }
    else
    {
      //根据命令行参数，用x+y+z=1设置一部分点云数据，此时点云组成菱形平面作为内点
      cloud->points[i].x =  rand () / (RAND_MAX + 1.0);
      cloud->points[i].y =  rand () / (RAND_MAX + 1.0);
      if( i % 5 == 0)
        cloud->points[i].z = rand () / (RAND_MAX + 1.0);
      else
        cloud->points[i].z = -1 * (cloud->points[i].x + cloud->points[i].y);
    }
  }

  std::vector<int> inliers;    //存储局内点集合的点的索引向量

  // created RandomSampleConsensus object and compute the appropriated model
  //针对球模型的对象
  pcl::SampleConsensusModelSphere<pcl::PointXYZ>::Ptr
    model_s(new pcl::SampleConsensusModelSphere<pcl::PointXYZ> (cloud));
  //针对平面模型的对象
  pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr
    model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (cloud));
  //根据命令行参数，来随机估算对应的平面模型，并存储估计的局内点
  if(pcl::console::find_argument (argc, argv, "-f") >= 0)
  {
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_p);
    ransac.setDistanceThreshold (.01);   //与平面距离小于0.01的点作为局内点
    ransac.computeModel();               //执行随机参数估计
    ransac.getInliers(inliers);          //存储估计所得的局内点
  }
  //根据命令行参数，来随机估算对应的圆球模型，并存储估计的局内点
  else if (pcl::console::find_argument (argc, argv, "-sf") >= 0 )
  {
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_s);
    ransac.setDistanceThreshold (.01);
    ransac.computeModel();
    ransac.getInliers(inliers);
  }

  // 复制估算模型的所有局内点到final中
  pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *final);

  // 创建可视化对象，并加入原始点云或者所有的局内点
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  if (pcl::console::find_argument (argc, argv, "-f") >= 0 || pcl::console::find_argument (argc, argv, "-sf") >= 0)
    viewer = simpleVis(final);
  else
    viewer = simpleVis(cloud);
  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
  return 0;
 }
