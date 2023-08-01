


#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>


#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h> 

#include <Eigen/Core>
#include <Eigen/Geometry>


using PointT = pcl::PointXYZI;
using PointCloudT = pcl::PointCloud<PointT>;
using CloudTPtr = PointCloudT::Ptr;


class SimpleMapping{
public:
  SimpleMapping(){
    sub_pc_ = nh_.subscribe("/velodyne_points", 1, &SimpleMapping::pc_cb, this,
                ros::TransportHints().tcpNoDelay());
    sub_pose_ = nh_.subscribe("/ardrone/ground_truth/odometry", 1, &SimpleMapping::uav_pose, this,
                ros::TransportHints().tcpNoDelay());
    pub_g_pc_ = nh_.advertise<sensor_msgs::PointCloud2>("/pc_map/map", 100);
  }

  ~SimpleMapping(){}

  void pc_cb(const sensor_msgs::PointCloud2::ConstPtr& msg){
    this_pc_ = *msg;
    flag_pc = true;
  }

  void uav_pose(const nav_msgs::Odometry::ConstPtr& msg){
    this_uav_pose_ = *msg;
    flag_pose = true;
  }

  void run(){
    ros::Rate loop(30);
    while(ros::ok()){
      ros::spinOnce();
      if(flag_pc && flag_pose){
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_pc(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(this_pc_, *pcl_pc);

        Eigen::Vector3f    odom_p;
        Eigen::Quaternionf odom_q;
        Eigen::Matrix4f    odom_t(Eigen::Matrix4f::Identity());

        odom_q.w() = this_uav_pose_.pose.pose.orientation.w;
        odom_q.x() = this_uav_pose_.pose.pose.orientation.x;
        odom_q.y() = this_uav_pose_.pose.pose.orientation.y;
        odom_q.z() = this_uav_pose_.pose.pose.orientation.z;

        odom_p[0] = this_uav_pose_.pose.pose.position.x;
        odom_p[1] = this_uav_pose_.pose.pose.position.y;
        odom_p[2] = this_uav_pose_.pose.pose.position.z;

        odom_t.block(0, 0, 3, 3) = odom_q.normalized().toRotationMatrix();
        odom_t.block(0, 3, 3, 1) = odom_p;

        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_pc(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*pcl_pc, *transformed_pc, odom_t);
        
        sensor_msgs::PointCloud2 transformed_msg;
        pcl::toROSMsg(*transformed_pc, transformed_msg);
        transformed_msg.header.stamp = this_pc_.header.stamp;
        transformed_msg.header.frame_id = "world";
        pub_g_pc_.publish(transformed_msg);
        flag_pose = false;
        flag_pc = false;
      }
      loop.sleep();
    }
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_pc_, sub_pose_;
  ros::Publisher pub_g_pc_;
  
  sensor_msgs::PointCloud2 this_pc_;
  nav_msgs::Odometry this_uav_pose_;

  bool flag_pose = false;
  bool flag_pc   = false;

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mark_finder");

  SimpleMapping simple_mapping;
  simple_mapping.run();

  return 0;
}