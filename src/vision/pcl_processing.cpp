#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/convolution_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/cloud_viewer.h>


void voxelGrid( pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &output_cloud )
{
  // Create the filtering object
  pcl::VoxelGrid<pcl::PointXYZRGB> sor;
  sor.setInputCloud (input_cloud);
  sor.setLeafSize (1.0f, 1.0f, 1.0f);
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
  pass.setFilterLimits (260.0, 320.0);
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
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);

  // Fill in the cloud data
  pcl::PCDReader reader;
  reader.read ("../../../RoviSamplePlugin/Scanner25D.pcd", *cloud); // Remember to download the file first!

    pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
 viewer.showCloud (cloud);
while (!viewer.wasStopped ())
{
}
  

  return (0);
}
