#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/convolution_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>

#include <pcl/visualization/cloud_viewer.h>




void voxelGrid( pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &output_cloud, float leafSize)
{
  // Create the filtering object
  pcl::VoxelGrid<pcl::PointXYZRGB> sor;
  sor.setInputCloud (input_cloud);
  sor.setLeafSize (leafSize, leafSize, leafSize);
  sor.filter (*output_cloud);
	
}


void outlierRemoval( pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &output_cloud )
{

  // Create the filtering object
  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
  sor.setInputCloud (input_cloud);
  sor.setMeanK (200);
  sor.setStddevMulThresh (0.01);
  sor.filter (*output_cloud);

}

void spatialFilter( pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &output_cloud )
{
	
  // Create the filtering object
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud (input_cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (-1.310, 2);
  pass.filter (*output_cloud);
}

void smoothing( pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &output_cloud )
{

  // Create a KD-Tree
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);

  // Output has the PointNormal type in order to store the normals calculated by MLS
  pcl::PointCloud<pcl::PointXYZRGBNormal> mls_points;

  // Init object (second point type is for the normals, even if unused)
  pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGBNormal> mls;
 
  mls.setComputeNormals (true);

  // Set parameters
  mls.setInputCloud (input_cloud);
  mls.setPolynomialOrder (3);
  mls.setSearchMethod (tree);
  mls.setSearchRadius (10.0f);
  
  // Reconstruct
  mls.process (mls_points);
  
  pcl::copyPointCloud<pcl::PointXYZRGBNormal, pcl::PointXYZRGB>(mls_points, *output_cloud); 
  
}


int main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_voxeled(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_transformed (new pcl::PointCloud<pcl::PointXYZRGB> ());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_spatialized (new pcl::PointCloud<pcl::PointXYZRGB>);
    
  // Fill in the cloud data
  pcl::PCDReader reader;
  reader.read ("../../../RoviSamplePlugin/Scanner25D.pcd", *cloud); // Remember to download the file first!
  
  voxelGrid(cloud,cloud_voxeled, 0.008);
  
  Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
  transform_2.rotate (Eigen::AngleAxisf (-(3.1415*25.0)/180.0, Eigen::Vector3f::UnitX()));


  
  // You can either apply transform_1 or transform_2; they are the same
  pcl::transformPointCloud (*cloud_voxeled, *cloud_transformed, transform_2);

  spatialFilter(cloud_transformed,cloud_spatialized);
  
  int32_t rgb = (static_cast<uint32_t>(255) << 16 | static_cast<uint32_t>(255) << 8 | static_cast<uint32_t>(255));
    for(auto &p: cloud_spatialized->points) p.rgb=rgb;

  pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
 viewer.showCloud (cloud_spatialized);
while (!viewer.wasStopped ())
{
}
  

  return (0);
}
