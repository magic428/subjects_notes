# 圆柱模型分割 - Cylinder model segmentation

This tutorial exemplifies how to run a Sample Consensus segmentation for cylindrical models. To make the example a bit more practical, the following operations are applied to the input dataset (in order):   

- data points further away than 1.5 meters are filtered
- surface normals at each point are estimated
- a plane model (describing the table in our demo dataset) is segmented and saved to disk
- a cylindrical model (describing the mug in our demo dataset) is segmented and saved to disk

- 1.5 米以外的数据被过滤掉;  
- 预先估计出了每个点的表面法向量;  
- 提前将实例中的平面模型分割出来并保存在本地磁盘文件中;  
- 最后检测圆柱体模型;  



**Note**  

由于噪声的存在, 圆柱体模型不是很理想.  


**The code**  

首先下载数据集 [table_scene_mug_stereo_textured.pcd](https://raw.github.com/PointCloudLibrary/data/master/tutorials/table_scene_mug_stereo_textured.pcd).  

cylinder_segmentation.cpp 文件内容如下:  

```cpp
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

int main (int argc, char** argv)
{
    // All the objects needed
    pcl::PCDReader reader;
    pcl::PassThrough<PointT> pass;
    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg; 
    pcl::PCDWriter writer;
    pcl::ExtractIndices<PointT> extract;
    pcl::ExtractIndices<pcl::Normal> extract_normals;
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());

    // Datasets
    pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<PointT>::Ptr cloud_filtered2 (new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
    pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients), coefficients_cylinder (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_cylinder (new pcl::PointIndices);

    // Read in the cloud data
    reader.read ("table_scene_mug_stereo_textured.pcd", *cloud);
    std::cerr << "PointCloud has: " << cloud->points.size () << " data points." << std::endl;

    // Build a passthrough filter to remove spurious NaNs
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0, 1.5);    // 移除 1.5 米 以外的点, 不做处理
    pass.filter (*cloud_filtered);
    std::cerr << "PointCloud after filtering has: " << cloud_filtered->points.size () << " data points." << std::endl;

    // Estimate point normals
    ne.setSearchMethod (tree);
    ne.setInputCloud (cloud_filtered);
    ne.setKSearch (50);
    ne.compute (*cloud_normals);

    // Create the segmentation object for the planar model and set all the parameters
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
    seg.setNormalDistanceWeight (0.1);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.03);
    seg.setInputCloud (cloud_filtered);
    seg.setInputNormals (cloud_normals);
    // Obtain the plane inliers and coefficients
    seg.segment (*inliers_plane, *coefficients_plane);
    std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

    // Extract the planar inliers from the input cloud
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers_plane);
    extract.setNegative (false);

    // Write the planar inliers to disk
    pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT> ());
    extract.filter (*cloud_plane);
    std::cerr << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;
    writer.write ("table_scene_mug_stereo_textured_plane.pcd", *cloud_plane, false);

    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_filtered2);
    extract_normals.setNegative (true);
    extract_normals.setInputCloud (cloud_normals);
    extract_normals.setIndices (inliers_plane);
    extract_normals.filter (*cloud_normals2);

    // Create the segmentation object for cylinder segmentation and set all the parameters
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_CYLINDER);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight (0.1);
    seg.setMaxIterations (10000);
    seg.setDistanceThreshold (0.05);
    seg.setRadiusLimits (0, 0.1);
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
        writer.write ("table_scene_mug_stereo_textured_cylinder.pcd", *cloud_cylinder, false);
    }
  return (0);
}
```

**The explanation**  

与本例最相关的代码就只有下面这几行:   

```cpp
  // Create the segmentation object for cylinder segmentation and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_CYLINDER);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setNormalDistanceWeight (0.1);    // 表面法向量的影响权重: surface normals influence
  seg.setMaxIterations (10000);         // 
  seg.setDistanceThreshold (0.05);      // 每个内点和模型之间的最大距离, 5cm
  seg.setRadiusLimits (0, 0.1);         // 圆柱体的半径为 10 cm.  
```

可以看到, 使用 SAC_RANSAC 来获取圆柱体模型参数, 具体含义在代码中用注释标出.  


**Compiling and running the program**  

CMakeLists.txt 文件内容大概如下:  

```cmake
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(cylinder_segmentation)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (cylinder_segmentation cylinder_segmentation.cpp)
target_link_libraries (cylinder_segmentation ${PCL_LIBRARIES})
```

After you have made the executable, you can run it. Simply do:  

```bash
$ ./cylinder_segmentation
```

You will see something similar to:  

```bash
PointCloud has: 307200 data points.
PointCloud after filtering has: 139897 data points.
[pcl::SACSegmentationFromNormals::initSACModel] Using a model of type: SACMODEL_NORMAL_PLANE
[pcl::SACSegmentationFromNormals::initSACModel] Setting normal distance weight to 0.100000
[pcl::SACSegmentationFromNormals::initSAC] Using a method of type: SAC_RANSAC with a model threshold of 0.030000
[pcl::SACSegmentationFromNormals::initSAC] Setting the maximum number of iterations to 100
Plane coefficients: header:
  seq: 0
  stamp: 0.000000000
  frame_id:
values[]
  values[0]: -0.0161854
  values[1]: 0.837724
  values[2]: 0.545855
  values[3]: -0.528787

PointCloud representing the planar component: 117410 data points.
[pcl::SACSegmentationFromNormals::initSACModel] Using a model of type: SACMODEL_CYLINDER
[pcl::SACSegmentationFromNormals::initSACModel] Setting radius limits to 0.000000/0.100000
[pcl::SACSegmentationFromNormals::initSACModel] Setting normal distance weight to 0.100000
[pcl::SACSegmentationFromNormals::initSAC] Using a method of type: SAC_RANSAC with a model threshold of 0.050000
[pcl::SampleConsensusModelCylinder::optimizeModelCoefficients] LM solver finished with exit code 2, having a residual norm of 0.322616.
Initial solution: 0.0452105 0.0924601 0.790215 0.20495 -0.721649 -0.661225 0.0422902
Final solution: 0.0452105 0.0924601 0.790215 0.20495 -0.721649 -0.661225 0.0396354
Cylinder coefficients: header:
  seq: 0
  stamp: 0.000000000
  frame_id:
values[]
  values[0]: 0.0452105
  values[1]: 0.0924601
  values[2]: 0.790215
  values[3]: 0.20495
  values[4]: -0.721649
  values[5]: -0.661225
  values[6]: 0.0396354

PointCloud representing the cylindrical component: 8625 data points.
```