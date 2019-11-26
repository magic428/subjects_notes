# 基于欧氏距离度量的点云簇聚类提取 - Euclidean Cluster Extraction  

In this tutorial we will learn how to extract Euclidean clusters with the pcl::EuclideanClusterExtraction class. In order to not complicate the tutorial, certain elements of it such as the plane segmentation algorithm, will not be explained here. Please check the Plane model segmentation tutorial for more information.

Theoretical Primer
A clustering method needs to divide an unorganized point cloud model P into smaller parts so that the overall processing time for P is significantly reduced. A simple data clustering approach in an Euclidean sense can be implemented by making use of a 3D grid subdivision of the space using fixed width boxes, or more generally, an octree data structure. This particular representation is very fast to build and is useful for situations where either a volumetric representation of the occupied space is needed, or the data in each resultant 3D box (or octree leaf) can be approximated with a different structure. In a more general sense however, we can make use of nearest neighbors and implement a clustering technique that is essentially similar to a flood fill algorithm.

Let’s assume we have given a point cloud with a table and objects on top of it. We want to find and segment the individual object point clusters lying on the plane. Assuming that we use a Kd-tree structure for finding the nearest neighbors, the algorithmic steps for that would be (from [RusuDissertation]):

create a Kd-tree representation for the input point cloud dataset P;

set up an empty list of clusters C, and a queue of the points that need to be checked Q;

then for every point \boldsymbol{p}_i \in P, perform the following steps:

add \boldsymbol{p}_i to the current queue Q;

for every point \boldsymbol{p}_i \in Q do:

search for the set P^i_k of point neighbors of \boldsymbol{p}_i in a sphere with radius r < d_{th};

for every neighbor \boldsymbol{p}^k_i \in P^k_i, check if the point has already been processed, and if not add it to Q;

when the list of all points in Q has been processed, add Q to the list of clusters C, and reset Q to an empty list

the algorithm terminates when all points \boldsymbol{p}_i \in P have been processed and are now part of the list of point clusters C

The Code
First, download the dataset table_scene_lms400.pcd and save it somewhere to disk.

Then, create a file, let’s say, cluster_extraction.cpp in your favorite editor, and place the following inside it:

```cpp

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>


int 
main (int argc, char** argv)
{
  // Read in the cloud data
  pcl::PCDReader reader;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
  reader.read ("table_scene_lms400.pcd", *cloud);
  std::cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl; //*

  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  vg.setInputCloud (cloud);
  vg.setLeafSize (0.01f, 0.01f, 0.01f);
  vg.filter (*cloud_filtered);
  std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*

  // Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::PCDWriter writer;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.02);

  int i=0, nr_points = (int) cloud_filtered->points.size ();
  while (cloud_filtered->points.size () > 0.3 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);

    // Get the points associated with the planar surface
    extract.filter (*cloud_plane);
    std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_f);
    *cloud_filtered = *cloud_f;
  }

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud_filtered);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.02); // 2cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_indices);

  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
    std::stringstream ss;
    ss << "cloud_cluster_" << j << ".pcd";
    writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
    j++;
  }

  return (0);
}
```

The Explanation
Now, let’s break down the code piece by piece, skipping the obvious.

    // Read in the cloud data
    pcl::PCDReader reader;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
    reader.read ("table_scene_lms400.pcd", *cloud);
    std::cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl;

              .
              .
              .

    while (cloud_filtered->points.size () > 0.3 * nr_points)
    {

              .
              .
              .

            // Remove the plane inliers, extract the rest
      extract.setNegative (true);
      extract.filter (*cloud_f);
cloud_filtered = cloud_f;
    }
The code above is already described in other tutorials, so you can read the explanation there (in particular Plane model segmentation and Extracting indices from a PointCloud).

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud_filtered);
There we are creating a KdTree object for the search method of our extraction algorithm.

  std::vector<pcl::PointIndices> cluster_indices;
Here we are creating a vector of PointIndices, which contain the actual index information in a vector<int>. The indices of each detected cluster are saved here - please take note of the fact that cluster_indices is a vector containing one instance of PointIndices for each detected cluster. So cluster_indices[0] contains all indices of the first cluster in our point cloud.

  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.02); // 2cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_indices);
Here we are creating a EuclideanClusterExtraction object with point type PointXYZ since our point cloud is of type PointXYZ. We are also setting the parameters and variables for the extraction. Be careful setting the right value for setClusterTolerance(). If you take a very small value, it can happen that an actual object can be seen as multiple clusters. On the other hand, if you set the value too high, it could happen, that multiple objects are seen as one cluster. So our recommendation is to just test and try out which value suits your dataset.

We impose that the clusters found must have at least setMinClusterSize() points and maximum setMaxClusterSize() points.

Now we extracted the clusters out of our point cloud and saved the indices in cluster_indices. To separate each cluster out of the vector<PointIndices> we have to iterate through cluster_indices, create a new PointCloud for each entry and write all points of the current cluster in the PointCloud.

  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
Compiling and running the program
Add the following lines to your CMakeLists.txt

 1
 2
 3
 4
 5
 6
 7
 8
 9
10
11
12
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(cluster_extraction)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (cluster_extraction cluster_extraction.cpp)
target_link_libraries (cluster_extraction ${PCL_LIBRARIES})
After you have made the executable, you can run it. Simply do:

$ ./cluster_extraction
You will see something similar to:

PointCloud before filtering has: 460400 data points.
PointCloud after filtering has: 41049 data points.
[SACSegmentation::initSACModel] Using a model of type: SACMODEL_PLANE
[SACSegmentation::initSAC] Using a method of type: SAC_RANSAC with a model threshold of 0.020000
[SACSegmentation::initSAC] Setting the maximum number of iterations to 100
PointCloud representing the planar component: 20522 data points.
[SACSegmentation::initSACModel] Using a model of type: SACMODEL_PLANE
[SACSegmentation::initSAC] Using a method of type: SAC_RANSAC with a model threshold of 0.020000
[SACSegmentation::initSAC] Setting the maximum number of iterations to 100
PointCloud representing the planar component: 12429 data points.
PointCloud representing the Cluster: 4883 data points.
PointCloud representing the Cluster: 1386 data points.
PointCloud representing the Cluster: 320 data points.
PointCloud representing the Cluster: 290 data points.
PointCloud representing the Cluster: 120 data points.
You can also look at your outputs cloud_cluster_0.pcd, cloud_cluster_1.pcd, cloud_cluster_2.pcd, cloud_cluster_3.pcd and cloud_cluster_4.pcd:

$ ./pcl_viewer cloud_cluster_0.pcd cloud_cluster_1.pcd cloud_cluster_2.pcd cloud_cluster_3.pcd cloud_cluster_4.pcd
You are now able to see the different clusters in one viewer. You should see something similar to this: