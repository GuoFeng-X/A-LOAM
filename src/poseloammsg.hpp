#pragma once

#include <Eigen/Core>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
 
class PoseLoamMsg
{
public:
    PoseLoamMsg(ros::NodeHandle& nh);
private:
    void callback(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg, const sensor_msgs::PointCloud2ConstPtr& laserCloudMsgCorner, const sensor_msgs::PointCloud2ConstPtr& laserCloudMsgSurf, const nav_msgs::Odometry::ConstPtr &laserOdometry);
    /** ros params **/
    ros::NodeHandle nh_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> lidar_sub_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> lidar_corner_sub_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> lidar_surf_sub_;
    message_filters::Subscriber<nav_msgs::Odometry> pose_sub_;

    typedef message_filters::sync_policies::ExactTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, nav_msgs::Odometry> lidar_odom_fuse_policy;
    typedef message_filters::Synchronizer<lidar_odom_fuse_policy> Sync;
    boost::shared_ptr<Sync> sync;

    std::string lidar_topic_,  pose_topic_, lidar_corner_topic_, lidar_surf_topic_;
};