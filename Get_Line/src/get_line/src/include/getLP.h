#pragma once

#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_cloud.h>

#include<pcl/filters/voxel_grid.h>

#include<sensor_msgs/PointCloud2.h>

#include <pcl/features/integral_image_normal.h>  //法线估计类头文件
#include <pcl/features/normal_3d.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/filters/extract_indices.h>
#include <vector>

#include <iostream>
#include <pcl/console/parse.h>
 
#include <boost/thread/thread.hpp>
#include <pcl/features/boundary.h>
#include <math.h>
#include <boost/make_shared.hpp>
 
#include <pcl/visualization/range_image_visualizer.h>

 
 
#include <pcl/filters/covariance_sampling.h>
#include <pcl/filters/normal_space.h>
#include <pcl/kdtree/kdtree_flann.h>
 
#include <pcl/io/ply_io.h>
 
#include <pcl/filters/statistical_outlier_removal.h>


typedef pcl::PointXYZI PointType;

class PclGetLP
{
  private:
    ros::Subscriber sub_point_cloud_;

    ros::Publisher pub_filtered_points_;
    ros::Publisher pub_feature_points_plane_;
	ros::Publisher pub_feature_points_line_;
    void point_feature(const sensor_msgs::PointCloud2ConstPtr& in_cloud);
  public: 
    PclGetLP(ros::NodeHandle &nh);
    ~PclGetLP();
    void Spin();
};

