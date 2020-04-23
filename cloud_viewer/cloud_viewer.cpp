#include <pcl/visualization/cloud_viewer.h>            //类CloudViewer头文件声明
#include <iostream>                                    //标准输入输出头文件声明
#include <pcl/io/io.h>                                 //I/O相关头文件声明
#include <pcl/io/pcd_io.h>                             //PCD文件读取头文件声明
    
int user_data;
//回调函数，在主函数中注册后只执行一次
void 
viewerOneOff (pcl::visualization::PCLVisualizer& viewer)
{
    viewer.setBackgroundColor (1.0, 0.5, 1.0);         //设置背景颜色
     pcl::PointXYZ o;                                  //存储球的圆心位置
     o.x = 1.0;
     o.y = 0;
     o.z = 0;
     viewer.addSphere (o, 0.25, "sphere", 0);          //添加圆球几何对象
     std::cout << "i only run once" << std::endl;   
}

//回调函数，在主函数中注册后每帧显示都执行一次   
void 
viewerPsycho (pcl::visualization::PCLVisualizer& viewer)
{
     static unsigned count = 0;
     std::stringstream ss;
     ss << "Once per viewer loop: " << count++;
     viewer.removeShape ("text", 0);
     viewer.addText (ss.str(), 200, 300, "text", 0);
     //FIXME: possible race condition here:
     user_data++;
}
    
int 
main (int argc,char** argv)
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::io::loadPCDFile ("maize.pcd", *cloud);                 //加载点云文件
    pcl::visualization::CloudViewer viewer("Cloud Viewer");     //创建viewer对象 
    //showCloud函数是同步的，在此处等待直到渲染显示为止
    viewer.showCloud(cloud);
    //该注册函数在可视化时只调用一次
    viewer.runOnVisualizationThreadOnce (viewerOneOff);
    //该注册函数在渲染输出时每次都调用
    viewer.runOnVisualizationThread (viewerPsycho);
    while (!viewer.wasStopped ())
    {
    	//在此处可以添加其他处理
    	user_data++;
    }
    return 0;
}
