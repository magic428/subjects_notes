# PCL 可视化操作  

PCL 的可视化类有好几号，最强大的是 PCLvisualizer。如果只是简单显示点云，可以不需要PCLvisualizer 类，调用 CloudViewer 就可以了。CloudViewer 的调用过程更加简单直接，PCLvisualizer 更加强大，功能更加丰富。  

PCLvisualizer 还可以设置鼠标键盘操作回调函数，功能之多，并且开源的 PCL 也在不断丰富和完善，所以在调用 PCLvisualizer 时，大部分编程是需要参考文档手册的。  

createViewPort() 函数还可以创建多个视口，比如把视窗分为左右两个部分.  

```cpp
p->createViewPort(0.0, 0, 0.5, 1.0, vp_1); //创建左视区
p->createViewPort(0.5, 0, 1.0, 1.0, vp_2); //创建右视区
```

```cpp
#include <iostream>
#include <vector>
#include <boost/shared_ptr.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
 
using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;
 
int main(int argc, char *argv[]){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("../pcd/a1.pcd", *cloud1) == -1){
        std::cerr << "open failed!" << std::endl;
        return -1;
    }
 
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    viewer->addCoordinateSystem(1.0, "global");   // 设置坐标轴系统和显示的点的大小
    viewer->initCameraParameters();     
 
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);          // 每次要刷新点云
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
 
    return 0;
}
```

## viewer->spinOnce() 和 viewer->spin() 的区别  

viewer->spinOnce() 的用法相对来说很灵活，但往往需要考虑调用消息的时机，调用频率，以及消息池的大小，这些都要根据现实情况协调好，不然会造成数据丢包或者延迟的错误。  

其实看函数名也能理解个差不多，viewer->spin() 是一直调用；viewer->spinOnce() 是只调用一次，viewer->spinOnce() 在在调用后还可以继续执行之后的程序. 如果还想再调用，就需要加上循环了:  

```cpp
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);          // 每次要刷新点云
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
```

viewer->spin() 函数一般不会出现在循环中, 函数在 Viewer 关闭的时候会自动退出.  


## Issues  

1) 如果 viewer 显示界面没有刷新, 而是多次视角重叠.  

可以通过设置 viewer 的背景颜色.  

```cpp
viewer->setBackgroundColor(0.9, 0.9, 0.9);
viewer->setBackgroundColor(0.1, 0.1, 0.1);
```

2) 向 viewer 中添加了多个点云数据, 但是并没有全部显示, 而是只显示了第一个加入的数据  

如果是在 CMD 窗口中运行, 会打印以下消息:  

```bash
[addPointCloud] The id <file> already exists! Please choose a different id and retry.
```

说明多次调用 addPointCloud() 函数时指定的 id 参数是相同的. 这也是导致只显示一个数据的原因.  

解决办法: 调用 addPointCloud() 函数添加不同点云时指定不同的 id 参数即可.  

