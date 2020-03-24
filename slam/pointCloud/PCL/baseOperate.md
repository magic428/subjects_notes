# PCL 基本操作  

- PCL 不同类型的点云之间进行类型转换  
- PCL 使用 Index 从给定的的点云簇中提取子点云簇  

> XYZ 颜色: X 红色, Y 绿色, Z 蓝色.  

## PCL 不同类型的点云之间进行类型转换  

可以使用 PCL 里面现成的函数 pcl::copyPointCloud() 或者手动转换.   

```cpp
#include <pcl/common/impl/io.h>

int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudxyz (new pcl::PointCloud<pcl::PointXYZ> ());  
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudxyzrgba (new pcl::PointCloud<pcl::PointXYZRGBA> ());
    pcl::copyPointCloud(*cloudxyz, *cloudxyzrgba);

    /// 或者手动转换：

    cloudxyzrgba->points.resize(cloudxyz->size());
    for (size_t i = 0; i < cloudxyz->points.size(); i++) {

        cloudxyzrgb->points[i].x = cloudxyz->points[i].x;
        cloudxyzrgb->points[i].y = cloudxyz->points[i].y;
        cloudxyzrgb->points[i].z = cloudxyz->points[i].z;
    }
}
```

## PCL 使用 Index 从给定的的点云簇中提取子点云簇  

点云操作过程中经常会需要提取点云子集，包括一些点云滤波算法也会经常得到点云的索引，然后根据这些点云索引来提取点云子集.  

pcl::ExtractIndices 类可以提供多种功能的点云子集提取，如果只是提取指定索引的点云组成一个新的点云，还可以用 pcl::copyPointCloud 函数:  

- 使用 pcl::ExtractIndices 对象;  
- 使用 pcl::copyPointCloud() 函數;  

下面代码示例了如何利用索引向量来构建点云索引并提取点云子集。  

```cpp
#include <iostream>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>

using std::cout; 
using std::endl;

const uint32_t pointCloudSize = 6;

int main()
{
    std::vector<pcl::PointIndices> clusterIndices;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudp(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->width = pointCloudSize;
    cloud->height = 1;
    cloud->points.resize (cloud->width * cloud->height);

    for (size_t i = 0; i < cloud->points.size (); ++i)
    {
        cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
        cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
        cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
    }
    std::cerr << "Cloud before extract: " << std::endl;
    for (size_t i = 0; i < cloud->points.size (); ++i)
        std::cerr << "    " << cloud->points[i].x << " "
                << cloud->points[i].y << " "
                << cloud->points[i].z << std::endl;
    
    // 构建 indexs 
    for (size_t i = 0; i < 4; i++) {
        
        pcl::PointIndices idxs;
        for (size_t j = 0; j < 4; j++) {

            int idx = rand() % (pointCloudSize);
            idxs.indices.push_back(idx);
        }
        clusterIndices.push_back(idxs);
    }

    /// 提取点云
    for (size_t i = 0; i < clusterIndices.size(); i++)
    {
        pcl::PointIndicesPtr index_ptr = boost::make_shared<pcl::PointIndices>(clusterIndices[i]);
        // Create the filtering object
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        // Extract the inliers
        extract.setInputCloud (cloud);
        extract.setIndices (index_ptr);
        extract.setNegative (true); // 如果设为 true,可以提取指定 index 之外的点云
        extract.filter (*cloudp);

        cout<<"-------- pcl::ExtractIndices --------"<<endl;
        for (size_t i = 0; i < cloudp->points.size (); ++i)
            std::cerr << "    " << cloudp->points[i].x << " "
                        << cloudp->points[i].y << " "
                        << cloudp->points[i].z << std::endl;
    }


    std::vector<int > indexs = { 1, 2, 5 };//声明索引值
    pcl::copyPointCloud(*cloud, indexs, *cloudp);//将对应索引的点存储
    cout<<"--------- pcl::copyPointCloud -------"<<endl;
    for (size_t i = 0; i < cloudp->points.size (); ++i)
        std::cerr << "    " << cloudp->points[i].x << " "
                    << cloudp->points[i].y << " "
                    << cloudp->points[i].z << std::endl;

}
```

## PCL 保存模型（平面，圆柱等等）分割中的内点  

```cpp
pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices);//声明指向PointIndices的共享指针
pcl::ModelCoefficients::Ptr coefficients_plane;
pcl::SACSegmentationFromNormalsseg;
seg.setInputCloud (cloudfiltered);
seg.setInputNormals (cloudnormals);
seg.segment (*inliers_plane, *coefficients_plane);//计算模型内点索引

pcl::ExtractIndices<pcl::PointXYZ> extract;
extract.setInputCloud (cloudfiltered);//输入点云
extract.setIndices (inliers_plane);//输入模型内点索引
pcl::PointCloud::Ptr cloudplane (new pcl::PointCloud());
extract.filter (*cloudplane);//根据输入点云和模型内点索引分割出内点对应点云

//存储模型内点对应点云
pcl::PCDWriter writer;
writer.write ("table_scene_mug_stereo_textured_plane.pcd", *cloudplane, false);
```

 
## Realsense 点云原始数据点云太多

共 640*480 左右个点，其中有一些噪点，严重影响了精度，使用StatisticalOutlierRemoval 进行移除。

但是速度太慢，基本 1Hz 的速度（还算快的），所以我之前先使用 VoxelGrid 对其进行降采样，降到了1700+个点，最后的速率达到15hz以上，满足了要求。效果见图！

## 如何获取 PCD 文件点云里点的格式  

比如是 pcl::PointXYZ 还是 pcl::PointXYZRGB 等类型?  

```cpp
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>

int main()
{
    pcl::PCDReader reader;
    pcl::PCLPointCloud2 cloud;
    reader.readHeader("D:/data/points/PCLTutorial/table_scene_mug_stereo_textured.pcd", cloud);
    for (size_t i = 0; i < cloud.fields.size(); i++)
    {
        std::cout << "fields_name: " << cloud.fields[i].name << std::endl;
    }
}
```

输出信息如下:  

```bash
fields_name: x
fields_name: y
fields_name: z
fields_name: rgb
```

## 如何实现类似 pcl::PointCloud::Ptr 和 pcl::PointCloud 的两个类相互转换?  

其实质就是 boost::shared_ptr 和内置指针之间的转换. 可以使用 pcl::PointCloud 对象的 makeShared() 函数将变量转换为 boost::shared_ptr 指针.  

```cpp
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPointer(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud = *cloudPointer;
    cloudPointer = cloud.makeShared();
}
```

## 如何加快 ASCII 格式存储，也就是记事本打开可以看到坐标数据的 pcd 文件读取速度?    

建议将 pcd 文件换成以 Binary 格式存储。  

```cpp
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>

int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    reader.readHeader("D:/data/points/PCLTutorial/table_scene_mug_stereo_textured.pcd", cloud);

    pcl::io::loadPCDFile<pcl::PointXYZ>("D:/data/points/PCLTutorial/table_scene_mug_stereo_textured.pcd", *cloud);
    pcl::io::savePCDFileBinary("D:/data/points/PCLTutorial/table_scene_mug_stereo_textured_bin.pcd", *cloud);
}
```

## 如何给 pcl::PointXYZ 类型的点云在显示时指定颜色?   

```cpp
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>

int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ>("C:/office3-after21111.pcd", *cloud);
    
    pcl::visualization::PCLVisualizer viewer("pointcloud viewer");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> sig(cloud, , , );
    viewer.addPointCloud(cloud, sig, "cloud");
    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }
}
```

## 如何查找点云的 x，y，z 的极值?   

```cpp
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>

int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ> ("your_pcd_file.pcd", *cloud);
    pcl::PointXYZ minPt, maxPt;
    pcl::getMinMax3D (*cloud, minPt, maxPt);
}
```

## 如何判断点云中的点为无效点，即坐标值为 nan，以及如何将点设置为无效点?  

使用 pcl::isFinite() 函数.  

```cpp
#include <pcl/point_types.h>
 
int main()
{
    pcl::PointXYZ p_valid;
    p_valid.x = ;
    p_valid.y = ;
    p_valid.z = ;
    std::cout << "Is p_valid valid? " << pcl::isFinite(p_valid) << std::endl;
    
    pcl::PointXYZ p_invalid;
    p_invalid.x = std::numeric_limits<float>::quiet_NaN();
    p_invalid.y = ;
    p_invalid.z = ;
    std::cout << "Is p_invalid valid? " << pcl::isFinite(p_invalid) << std::endl;
}
```

## 如何将无效点从点云中移除?  

使用 pcl::removeNaNFromPointCloud() 函数.  

```cpp
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
 
typedef pcl::PointCloud<pcl::PointXYZ> CloudType;

int main()
{
    CloudType::Ptr cloud(new CloudType);
    cloud->is_dense = false;
    CloudType::Ptr output_cloud(new CloudType);
    
    CloudType::PointType p_nan;
    p_nan.x = std::numeric_limits<float>::quiet_NaN();
    p_nan.y = std::numeric_limits<float>::quiet_NaN();
    p_nan.z = std::numeric_limits<float>::quiet_NaN();
    cloud->push_back(p_nan);
    
    CloudType::PointType p_valid;
    p_valid.x = 1.0f;
    cloud->push_back(p_valid);
    
    std::cout << "size: " << cloud->points.size() << std::endl;
    
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud, *output_cloud, indices);
    std::cout << "size: " << output_cloud->points.size() << std::endl;
}
```

## 如果知道需要保存点的序号，如何从原点云中拷贝点到新点云?  

```cpp
#include <pcl/io/pcd_io.h>
#include <pcl/common/impl/io.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ>("C:/office3-after21111.pcd", *cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<int> indexs = {1, 3, 4};
    pcl::copyPointCloud(*cloud, indexs, *cloudOut);
}
```

## 如何从点云里删除和添加点?  

```cpp
#include "stdafx.h"
#include <pcl/io/pcd_io.h>
#include <pcl/common/impl/io.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ>("C:/office3-after21111.pcd", *cloud);
    pcl::PointCloud<pcl::PointXYZ>::iterator index = cloud->begin();
    cloud->erase(index);//删除第一个
    index = cloud->begin() + 5;
    cloud->erase(index);//删除第5个
    pcl::PointXYZ point = {1, 2, 3};
    
    // 在索引号为 5 的位置上插入一点，原来的点后移一位
    cloud->insert(cloud->begin() + 5, point);
    cloud->push_back(point); // 从点云最后面插入一点
    std::cout << cloud->points[5].x; // 输出第一个点
}
```

如果删除的点太多建议用上面的方法拷贝到新点云，再赋值给原点云，如果要添加很多点，建议先 resize() 函数，然后用循环向点云里的添加。  

## pcl::PointCloud 和 pcl::PCLPointCloud2 类型如何相互转换?  

```cpp
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
 
int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCLPointCloud2 cloud2;
    pcl::io::loadPCDFile("C:/office3-after21111.pcd", cloud2);
    pcl::fromPCLPointCloud2(cloud2, *cloud);
    pcl::toPCLPointCloud2(*cloud, cloud2);
}
```

## 如何显示自定义的各类图形，

```cpp
void viewerOneOff(boost::shared_ptr<pcl::visualization::PCLVisualizer>& viewer)
{
    //viewer->setBackgroundColor(0.9, 0.9, 0.9);       //设置背景颜色

    //add a line
    pcl::PointXYZ p1, p2;
    p1.x = 0; p1.y = 0; p1.z = 0;
    p2.x = 1; p2.y = 1; p2.z = 1;
    viewer->addLine(p1, p2, 0, 1, 0, "line", 0);
    //add a circle
    pcl::ModelCoefficients circle_coeff;
    circle_coeff.values.resize(3);
    circle_coeff.values[0] = 1;
    circle_coeff.values[1] = 1;
    circle_coeff.values[2] = 2;
    viewer->addCircle(circle_coeff, "circle", 0);

    //添加球体
    pcl::PointXYZ o;
    o.x = 1.0;
    o.y = 0;
    o.z = 0;
    viewer->addSphere(o, 0.25, "sphere", 0);
    //addSphere (const PointT &center, double radius, const std::string &id, int viewport)

    //添加立方体
    viewer->addCube(0, 0.1, 0, 0.1, 0, 0.1, 1, 0, 0, "cube", 0);
    //addCube(float x_min, float x_max, float y_min, float y_max, float z_min, float z_max,
    //double r = 1.0, double g = 1.0, double b = 1.0, const std::string &id = "cube", int viewport = 0);

    //添加圆锥
    pcl::ModelCoefficients cone;
    cone.values.push_back(0);
    cone.values.push_back(0);
    cone.values.push_back(0);
    cone.values.push_back(0.0);
    cone.values.push_back(1.0);
    cone.values.push_back(0.0);
    cone.values.push_back(5.0);
    viewer->addCone(cone, "cone");

    //viewer->removeAllPointClouds();//删除所有点	

    //添加箭头
    pcl::PointXYZ A, B;
    A.x = 0; A.y = 0; A.z = 0;
    B.x = -1; B.y = 0; B.z = 0;
    viewer->addArrow<pcl::PointXYZ>(A, B, 255, 0, 0, false, "arrow", 0);

    //viewer->removeAllShapes();移除形状
}
```

## 如何在某个指定点的固定半径长度邻域内搜索点  

核心方法是: kdtree.radiusSearch(). 通过指定基准点和需要搜索的半径, 就可以得到搜索结果.  

```cpp
int main()
{
    // Search the neighbors in the range with radius
    pcl::KdTreeFLANN<PointT> kdtree;
    kdtree.setInputCloud (cloudIn);

    // Neighbors within radius search
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    float radius = 0.2;

    // set the searchPoint with the picked point
    PointT searchPoint;
    searchPoint.x = picked_point_coord[0];
    searchPoint.y = picked_point_coord[1];
    searchPoint.z = picked_point_coord[2];

    LOG(DEBUG) << "Neighbors within radius search at (" << searchPoint.x
                    << " " << searchPoint.y
                    << " " << searchPoint.z
                    << ") with radius=" << radius;

    if(kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {

        // color each cluster
        int r = rand() % 156;
        int g = rand() % 186;
        int b = rand() % 256;

        for (int j = 0; j < pointIdxRadiusSearch.size(); j++)
        {
            cloudIn->points[pointIdxRadiusSearch[j]].r = r;
            cloudIn->points[pointIdxRadiusSearch[j]].g = g;
            cloudIn->points[pointIdxRadiusSearch[j]].b = b;
        }
    } else {
        return 0;
    }

    LOG(DEBUG) << "Search Result: " << pointIdxRadiusSearch.size() << " neighbor points.";
}
```

## 参考资料 

[1] PCL点云处理BUG记录及分析: https://blog.csdn.net/u010141368/article/details/52664332  

[2] http://www.zhangzscn.com/2015/11/27/pcl%E5%BC%80%E6%BA%90%E5%BA%93%E5%B8%B8%E8%A7%81%E7%BC%96%E7%A8%8B%E9%97%AE%E9%A2%98/

