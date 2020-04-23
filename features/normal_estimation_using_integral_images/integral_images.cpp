#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/visualization/cloud_viewer.h>

int main ()
{
    // load point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile ("table_scene_mug_stereo_textured.pcd", *cloud);

    // estimate normals
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);

    pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    /* The following normal estimation methods are available:
     * enum NormalEstimationMethod
     * {
     *     COVARIANCE_MATRIX,
     *     AVERAGE_3D_GRADIENT,
     *     AVERAGE_DEPTH_CHANGE
     *  };
     */
    ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);    // 设置估计方法
    ne.setMaxDepthChangeFactor(0.02f);                        // 最大深度变化系数
    ne.setNormalSmoothingSize(10.0f);                         // 优化法线方向时考虑邻域大小
    ne.setInputCloud(cloud);                                  // 输入点云（必须为有序点云）
    ne.compute(*normals);                                     // 执行法线估计，存储结果到normals

    // visualize normals
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.setBackgroundColor (0.0, 0.0, 0.5);
    viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud, normals);

    while (!viewer.wasStopped ())
    {
        viewer.spinOnce ();
    }
    return 0;
}