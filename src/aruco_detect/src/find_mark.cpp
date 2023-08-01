
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <iostream>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>
#include <aruco_msgs/MarkerArray.h>

#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_listener.h>
#include <std_msgs/UInt32MultiArray.h>
#include <image_transport/image_transport.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>


#include <opencv2/opencv.hpp>


#include "aruco_utils.hpp"


/**
 * @brief Camera parameters
  header: 
    seq: 20283
    stamp: 
      secs: 6541
      nsecs: 242000000
    frame_id: "camera_color_optical_frame"
  height: 1080
  width: 1920
  distortion_model: "plumb_bob"
  D: []
  K: [1386.4139404296875, 0.0, 960.0, 0.0, 1386.4139404296875, 540.0, 0.0, 0.0, 1.0]
  R: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  P: [1386.4139404296875, 0.0, 960.0, 0.0, 0.0, 1386.4139404296875, 540.0, 0.0, 0.0, 0.0, 1.0, 0.0]
  binning_x: 0
  binning_y: 0
  roi: 
    x_offset: 0
    y_offset: 0
    height: 0
    width: 0
    do_rectify: False
  ---
 */


const double g_marker_size = 0.4;
const double g_K[] = {1386.4139404296875, 0.0, 960.0, 0.0, 1386.4139404296875, 540.0, 0.0, 0.0, 1.0};

bool g_publishImage = false;
bool g_publishDebug = true;

class MarkFinder{
public:
  MarkFinder():
  it_(nh_)
  {
    sub_img_ = nh_.subscribe("/camera/color/image_raw", 1, &MarkFinder::vison_cb, this);
    image_pub_ = it_.advertise("/detect_marks/img/result", 1);
    debug_pub_ = it_.advertise("/detect_marks/img/debug", 1);
    pub_mark_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/detect_marks/pose", 100);
  }

  ~MarkFinder(){}

  void init_cam_para() {
    cv::Mat intrinsics(3, 3, CV_64F);
    cv::Mat distortion_coeff(5, 1, CV_64F);
    cv::Size image_size(1920, 1080);

    for (size_t i = 0; i < 3; i++) {
      for (size_t j = 0; j < 3; j++) {
        intrinsics.at<double>(i, j) = g_K[3 * i + j];
      }
    }

    for (size_t i = 0; i < 5; i++) {
      distortion_coeff.at<double>(i, 0) = 0;
    }
    camParam_.setParams(intrinsics, distortion_coeff, image_size);
  }

private:
  void vison_cb(const sensor_msgs::Image::ConstPtr& msg){
    this_img_ = *msg;
    ros::Time curr_stamp = msg->header.stamp;
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
      inImage_ = cv_ptr->image;
      markers_.clear();
      mDetector_.detect(inImage_, markers_, camParam_, g_marker_size, false);

      for (std::size_t i = 0; i < markers_.size(); ++i)
      {
        tf::Transform transform = aruco_visual::arucoMarker2Tf(markers_[i]);
        geometry_msgs::Pose mark_pose;
        tf::poseTFToMsg(transform, mark_pose);
        geometry_msgs::PoseWithCovarianceStamped msg_pub;
        msg_pub.header.stamp = curr_stamp;
        msg_pub.pose.pose = mark_pose;
        msg_pub.pose.covariance[0] = markers_.at(i).id;
        pub_mark_.publish(msg_pub);
      }

      if (g_publishImage)
      {
        cv_bridge::CvImage out_msg;
        out_msg.header.stamp = curr_stamp;
        out_msg.encoding = sensor_msgs::image_encodings::RGB8;
        out_msg.image = inImage_;
        image_pub_.publish(out_msg.toImageMsg());
      }

      if (g_publishDebug)
      {
        cv_bridge::CvImage debug_msg;
        debug_msg.header.stamp = curr_stamp;
        debug_msg.encoding = sensor_msgs::image_encodings::MONO8;
        debug_msg.image = mDetector_.getThresholdedImage();
        debug_pub_.publish(debug_msg.toImageMsg());
      }

    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }
  }


private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_img_;
  ros::Publisher pub_mark_;

  sensor_msgs::Image this_img_;

  aruco::MarkerDetector mDetector_;
  aruco::CameraParameters camParam_;
  std::vector<aruco::Marker> markers_;

  image_transport::ImageTransport it_;


  ros::Subscriber cam_info_sub_;
  aruco_msgs::MarkerArray::Ptr marker_msg_;

  cv::Mat inImage_;
  bool useCamInfo_;
  std_msgs::UInt32MultiArray marker_list_msg_;

  image_transport::Publisher image_pub_;
  image_transport::Publisher debug_pub_;

};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "mark_finder");

  MarkFinder mark_finder;
  mark_finder.init_cam_para();
  ros::spin();
  return 0;
}