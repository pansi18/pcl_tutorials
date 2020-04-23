#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

typedef pcl::PointXYZ PointT;

int user_data;
// void 
// viewerOneOff (pcl::visualization::PCLVisualizer& viewer)
// {
// 	viewer.setBackgroundColor (1.0, 0.5, 0.5);//ÉèÖÃ±³Ÿ°ÑÕÉ«
// 	pcl::PointXYZ o;//ŽæŽ¢ÇòµÄÔ²ÐÄÎ»ÖÃ
// 	o.x = 1.0;
// 	o.y = 0;
// 	o.z = 0;
// 	viewer.addSphere (o, 0.25, "sphere", 0);//ÌíŒÓÔ²ÇòŒ¯ºÏ¶ÔÏó
// 	std::cout << "i only run once" << std::endl;
// 
// }

// void 
// viewerPsycho (pcl::visualization::PCLVisualizer& viewer)
// {
// 	static unsigned count = 0;
// 	std::stringstream ss;
// 	ss << "Once per viewer loop: " << count++;
// 	viewer.removeShape ("text", 0);
// 	viewer.addText (ss.str(), 200, 300, "text", 0);
// 	//FIXME: possible race condition here:
// 	user_data++;
// }

void 
cloudview(std::string file_name)
{
	// pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile (file_name, *cloud);//ŒÓÔØµãÔÆÎÄŒþ
	pcl::visualization::CloudViewer viewer("Cloud Viewer");    
	//showCloudº¯ÊýÊÇÍ¬²œµÄ£¬ÔÚŽËŽŠµÈŽýÖ±µœäÖÈŸÏÔÊŸÎªÖ¹
	viewer.showCloud(cloud);
	//žÃ×¢²áº¯ÊýÔÚ¿ÉÊÓ»¯Ê±Ö»µ÷ÓÃÒ»ŽÎ
	//viewer.runOnVisualizationThreadOnce (viewerOneOff);
	//žÃ×¢²áº¯ÊýÔÚäÖÈŸÊä³öÊ±Ã¿ŽÎ¶Œµ÷ÓÃ
	//viewer.runOnVisualizationThread (viewerPsycho);
	while (!viewer.wasStopped ())
	{
		//ÔÚŽËŽŠ¿ÉÒÔÌíŒÓÆäËûŽŠÀí
		user_data++;
	}
}

int
main (int argc, char** argv)
{
  // All the objects needed
  pcl::PCDReader reader;                                      //PCD文件读取对象
  pcl::PassThrough<PointT> pass;                              //直通滤波对象
  pcl::NormalEstimation<PointT, pcl::Normal> ne;              //法线估计对象
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;   //分割对象 
  pcl::PCDWriter writer;                                      //PCD文件写入对象
  pcl::ExtractIndices<PointT> extract;                        //点提取对象
  pcl::ExtractIndices<pcl::Normal> extract_normals;           //点提取对象
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());

  // Datasets
  pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<PointT>::Ptr cloud_filtered2 (new pcl::PointCloud<PointT>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
  pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients), coefficients_cylinder (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_cylinder (new pcl::PointIndices);

  // 从文件中读取点云数据
  reader.read ("table_scene_mug_stereo_textured.pcd", *cloud);
  std::cerr << "PointCloud has: " << cloud->points.size () << " data points." << std::endl;
  cloudview("table_scene_mug_stereo_textured.pcd");

  // 进行直通滤波，将z轴不在(0,1.5)范围内的点过滤掉
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0, 1.5);
  pass.filter (*cloud_filtered);
  std::cerr << "PointCloud after filtering has: " << cloud_filtered->points.size () << " data points." << std::endl;
  writer.write ("passthrough_filtered.pcd", *cloud_filtered, false);
  cloudview("passthrough_filtered.pcd");

  // 对过滤后点云进行法线估计
  ne.setSearchMethod (tree);
  ne.setInputCloud (cloud_filtered);
  ne.setKSearch (50);
  ne.compute (*cloud_normals);

  // 设置分割所用的模型类型、方法及相关参数
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
  seg.setNormalDistanceWeight (0.1);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.03);
  seg.setInputCloud (cloud_filtered);
  seg.setInputNormals (cloud_normals);
  // 执行分割，获取平面模型系数和处在平面内上的内点
  seg.segment (*inliers_plane, *coefficients_plane);
  std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

  // 从点云中抽取分割的处在平面上的点集
  extract.setInputCloud (cloud_filtered);
  extract.setIndices (inliers_plane);
  extract.setNegative (false);
  pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT> ());
  extract.filter (*cloud_plane);

  //存储分割得到的平面上的点云文件
  std::cerr << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;
  writer.write ("one_frame_std_plane.pcd", *cloud_plane, false);
  cloudview("one_frame_std_plane.pcd");

  // Remove the planar inliers, extract the rest
  extract.setNegative (true);
  extract.filter (*cloud_filtered2);
  extract_normals.setNegative (true);
  extract_normals.setInputCloud (cloud_normals);
  extract_normals.setIndices (inliers_plane);
  extract_normals.filter (*cloud_normals2);

  // 为圆柱体分割创建分割对象，设置所有参数
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_CYLINDER);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setNormalDistanceWeight (0.1);
  seg.setMaxIterations (10000);
  seg.setDistanceThreshold (0.05);            //设置内点到模型的距离允许最大值
  seg.setRadiusLimits (0, 0.1);               //设置估计出的圆柱模型的半径范围
  seg.setInputCloud (cloud_filtered2);
  seg.setInputNormals (cloud_normals2);

  // Obtain the cylinder inliers and coefficients
  seg.segment (*inliers_cylinder, *coefficients_cylinder);
  std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

  // Write the cylinder inliers to disk
  extract.setInputCloud (cloud_filtered2);
  extract.setIndices (inliers_cylinder);
  extract.setNegative (false);
  pcl::PointCloud<PointT>::Ptr cloud_cylinder (new pcl::PointCloud<PointT> ());
  extract.filter (*cloud_cylinder);
  if (cloud_cylinder->points.empty ()) 
    std::cerr << "Can't find the cylindrical component." << std::endl;
  else
  {
    std::cerr << "PointCloud representing the cylindrical component: " << cloud_cylinder->points.size () << " data points." << std::endl;
    writer.write ("one_frame_std_cylindrical.pcd", *cloud_cylinder, false);
    cloudview("one_frame_std_cylindrical.pcd");
  }
  return (0);
}
