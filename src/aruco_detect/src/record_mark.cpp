

#include <cmath>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <Eigen/Core>
#include <Eigen/Geometry>


class RecordMark{
public:
RecordMark(){
  sub_mark_ = nh_.subscribe("/find_marks/pose", 1, &RecordMark::mark_cb, this,
                ros::TransportHints().tcpNoDelay());
  sub_pose_ = nh_.subscribe("/ardrone/ground_truth/odometry", 1, &RecordMark::uav_pose, this,
              ros::TransportHints().tcpNoDelay());
  pub_g_mark_ = nh_.advertise<visualization_msgs::Marker>("/global/mark/pose", 1);

  camera_in_base_t_.block(0, 0, 3, 3) << 0, 0, 1, -1, 0, 0, 0, -1, 0;
  camera_in_base_t_.block(0, 3, 3, 1) << 0, 0, 0;
}

~RecordMark(){}

void uav_pose(const nav_msgs::Odometry::ConstPtr& msg){
  this_uav_pose_ = *msg;
  flag_pose = true;
}

void mark_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
  mark_pose_ = *msg;
  flag_mark = true;
}

void run(){
  ros::Rate loop(30);
  while(ros::ok()){
    ros::spinOnce();
    if(flag_mark && flag_pose){
      // 相机坐标系下检测的 mark
      Eigen::Vector3f    mark_p;
      Eigen::Quaternionf mark_q;
      Eigen::Matrix4f    mark_t = Eigen::Matrix4f::Identity();

      mark_p[0] = mark_pose_.pose.pose.position.x;
      mark_p[1] = mark_pose_.pose.pose.position.y;
      mark_p[2] = mark_pose_.pose.pose.position.z;
      mark_q.w() = mark_pose_.pose.pose.orientation.w;
      mark_q.x() = mark_pose_.pose.pose.orientation.x;
      mark_q.y() = mark_pose_.pose.pose.orientation.y;
      mark_q.z() = mark_pose_.pose.pose.orientation.z;

      // 距离过滤
      if (mark_p[2] < 0.3 || mark_p[2] > 9.0){
        flag_pose = false;
        flag_mark = false;
        continue;
      }
      // fov 过滤   
      double roll_deg = std::atan2(mark_p[0], mark_p[2]) * 180 * M_1_PI;
      if (roll_deg > 20){
        flag_pose = false;
        flag_mark = false;
        continue;
      }

      Eigen::Vector3f    odom_p;
      Eigen::Quaternionf odom_q;
      Eigen::Matrix4f    odom_t = Eigen::Matrix4f::Identity();

      odom_q.w() = this_uav_pose_.pose.pose.orientation.w;
      odom_q.x() = this_uav_pose_.pose.pose.orientation.x;
      odom_q.y() = this_uav_pose_.pose.pose.orientation.y;
      odom_q.z() = this_uav_pose_.pose.pose.orientation.z;

      odom_p[0] = this_uav_pose_.pose.pose.position.x;
      odom_p[1] = this_uav_pose_.pose.pose.position.y;
      odom_p[2] = this_uav_pose_.pose.pose.position.z;

      odom_t.block(0, 0, 3, 3) = odom_q.normalized().toRotationMatrix();
      odom_t.block(0, 3, 3, 1) = odom_p;

      mark_t.block(0, 0, 3, 3) = mark_q.normalized().toRotationMatrix();
      mark_t.block(0, 3, 3, 1) = mark_p;

      Eigen::Matrix4f result = odom_t * camera_in_base_t_ * mark_t;
      Eigen::Vector3f result_p = result.block(0, 3 ,3, 1);
      Eigen::Matrix3f rotation_matrix = result.block(0, 0, 3, 3);
      Eigen::Quaternionf result_q(rotation_matrix);  

      visualization_msgs::Marker markPointMsg;
      markPointMsg.type = visualization_msgs::Marker::CUBE;
      markPointMsg.action = visualization_msgs::Marker::ADD;
      markPointMsg.ns = "res";
      markPointMsg.id = 0;
      markPointMsg.header.frame_id = "world";
      markPointMsg.color.a = 1.0;
      markPointMsg.color.b = 51/255.;
      markPointMsg.color.g = 51/255.;
      markPointMsg.color.r = 204/255.;
      markPointMsg.scale.x = 0.5;
      markPointMsg.scale.y = 0.8;
      markPointMsg.scale.z = 0.1;
      markPointMsg.pose.position.x = result_p[0];
      markPointMsg.pose.position.y = result_p[1];
      markPointMsg.pose.position.z = result_p[2];
      markPointMsg.pose.orientation.w = result_q.w();
      markPointMsg.pose.orientation.x = result_q.x();
      markPointMsg.pose.orientation.y = result_q.y();
      markPointMsg.pose.orientation.z = result_q.z();
      pub_g_mark_.publish(markPointMsg);

      flag_pose = false;
      flag_mark = false;
    }
    loop.sleep();
  }
}

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_mark_, sub_pose_;
  ros::Publisher pub_g_mark_;
  ros::Publisher tag_pub;
  
  nav_msgs::Odometry this_uav_pose_;
  geometry_msgs::PoseWithCovarianceStamped mark_pose_;

  Eigen::Matrix4f camera_in_base_t_ = Eigen::Matrix4f::Identity();

  bool flag_pose = false;
  bool flag_mark = false;
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "mark_finder");

  RecordMark record_mark;
  record_mark.run();

  return 0;
}