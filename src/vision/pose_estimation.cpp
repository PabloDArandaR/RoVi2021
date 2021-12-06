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
#include <stdlib.h> 
#include <time.h>       
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/registration/transformation_estimation_svd.h>


void extract3Random(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr rands){
    rands->erase(rands->begin(),rands->end());
    int i = 0;
    while(i != 3){
        int r = rand() % cloud->size();
        const pcl::PointXYZRGB& rpoint = (*cloud)[r];
        bool same = false;
        for(int j = 0; j < rands->size(); j++){
            const pcl::PointXYZRGB& rpoint_prev = (*rands)[j];
            if(rpoint.x == rpoint_prev.x and rpoint.y == rpoint_prev.y and rpoint.z == rpoint_prev.z){
                same = true;
                break;
            }
        }
        if(!same){
            rands->push_back(pcl::PointXYZRGB(rpoint));
            i++;
        }
    }
}


Eigen::Matrix4f estimate_pose(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_1, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_2){
  pcl::registration::TransformationEstimationSVD<pcl::PointXYZRGB,pcl::PointXYZRGB> TESVD;
  Eigen::Matrix4f transformation;
  TESVD.estimateRigidTransformation (*cloud_1,*cloud_2,transformation);
  return transformation;

}


float distance_sq(const pcl::PointXYZRGB& p1, const pcl::PointXYZRGB& p2){
    return pow(p1.x-p2.x,2) + pow(p1.y-p2.y,2) + pow(p1.z-p2.z,2);
}

pcl::PointXYZRGB nearest_neighbor(pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene, const pcl::PointXYZRGB& p){
    float min_dist = distance_sq((*scene)[0],p);
    pcl::PointXYZRGB near = (*scene)[0];
    for(pcl::PointXYZRGB &p2: scene->points){
       float d = distance_sq(p,p2);
       if(d < min_dist){
            min_dist = d;
            near = p2;
       }
    }
    
    return near;
}

float calculate_error(pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene, pcl::PointCloud<pcl::PointXYZRGB>::Ptr bottle){
    float err = 0;
    for(pcl::PointXYZRGB &p: bottle->points){
        pcl::PointXYZRGB near = nearest_neighbor(scene,p);
        err += distance_sq(p,near);
    }
    
    return err;  
}


int main (int argc, char** argv){
  srand(time(NULL));
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_scene (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_bottle (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr rands_scene (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr rands_bottle (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr bottle_global (new pcl::PointCloud<pcl::PointXYZRGB>);
  
  
  
  // Fill in the cloud data
  pcl::PCDReader reader;
  reader.read ("../scene_cloud.pcd", *cloud_scene); // Remember to download the file first!
  reader.read ("../bottle_cloud.pcd", *cloud_bottle); // Remember to download the file first!
    
    
  float min_err = 10000000000000;
  Eigen::Matrix4f best_trans= Eigen::Matrix4f::Identity();
  int n = 1000;
  for(int i = 0; i < n; i++){
      if(i % (n/10) == 0)
        std::cout << ((float)100.0*i)/n << " %" << std::endl;  
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr bottle_trans (new pcl::PointCloud<pcl::PointXYZRGB>);    
      extract3Random(cloud_scene,rands_scene);
      extract3Random(cloud_bottle,rands_bottle);
      Eigen::Matrix4f trans = estimate_pose(rands_bottle,rands_scene);
      pcl::transformPointCloud (*cloud_bottle, *bottle_trans, trans);
      float err = calculate_error(cloud_scene,bottle_trans);
      if(err < min_err){
        min_err = err;
        best_trans = trans;
      }
  }
  
 
  pcl::transformPointCloud (*cloud_bottle, *bottle_global, best_trans);

  pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
 viewer.showCloud (cloud_scene,"Scene");
 viewer.showCloud (bottle_global,"Bottle");
 std::cout << calculate_error(cloud_scene,bottle_global) << std::endl;
while (!viewer.wasStopped ())
{
}



  return (0);
}
