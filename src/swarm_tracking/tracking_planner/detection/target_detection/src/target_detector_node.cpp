#include <ros/ros.h>
#include <Eigen/Geometry>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <target_detection/esikf_tracker.hpp>
#include <target_detection/uav_detector.hpp>
#include <swarm_msgs/TargetObsrv.h>

ros::Publisher target_odom_pub_;
ros::Publisher target_obsrv_pub_;

ros::Timer ekf_timer_;
ros::Subscriber target_true_odom_sub_;

uavDetector detector_;

void EKFCallback(const ros::TimerEvent& event)
{
  Vector3d target_pos, target_vel;
  ros::Time predict_T;
  nav_msgs::Odometry target_odom;
  if(detector_.runEKF(predict_T, target_pos, target_vel))
  {
    target_odom.header.stamp = predict_T;
    target_odom.header.frame_id = "world";
    target_odom.pose.pose.position.x = target_pos.x();
    target_odom.pose.pose.position.y = target_pos.y();
    target_odom.pose.pose.position.z = target_pos.z();
    target_odom.twist.twist.linear.x = target_vel.x();
    target_odom.twist.twist.linear.y = target_vel.y();
    target_odom.twist.twist.linear.z = target_vel.z();
    target_odom.pose.pose.orientation.w = 1.0;
    target_odom_pub_.publish(target_odom);
  }else{
    target_odom.header.stamp = predict_T;
    target_odom.pose.pose.position.x = DBL_MAX;
    target_odom.pose.pose.position.y = DBL_MAX;
    target_odom.pose.pose.position.z = DBL_MAX;
    target_odom.header.frame_id = "world";
    target_odom.pose.covariance[0] = -1.0;
    target_odom_pub_.publish(target_odom);
  }
}

void TeamMeasCallback(const swarm_msgs::TargetObsrvPtr& obsrv_msg)
{
  
  if(obsrv_msg->drone_id >= 0 && obsrv_msg->drone_id != detector_.getId())
  {
    detector_.setTeamMeas(obsrv_msg);
  }
     
}

void SelfMeasCallback(const sensor_msgs::PointCloud2ConstPtr& pcl_msg)
{
  ros::Time msg_t = pcl_msg->header.stamp;
  Vector3d pos_observation;
  if(detector_.setSelfMeas(pcl_msg, pos_observation))
  {
    swarm_msgs::TargetObsrv target_observation;

    target_observation.header.stamp = msg_t;
    
    target_observation.header.frame_id = "world";
    target_observation.drone_id = detector_.getId();
    target_observation.target_pos[0] = pos_observation.x();
    target_observation.target_pos[1] = pos_observation.y();
    target_observation.target_pos[2] = pos_observation.z();
    target_obsrv_pub_.publish(target_observation);
  }
}


void OdomCallback(const nav_msgs::OdometryConstPtr& odom_msg)
{
  detector_.setOdom(odom_msg);
}

void TargetOdomCallback(const nav_msgs::OdometryConstPtr& target_odom_msg)
{
  detector_.setTargetOdom(target_odom_msg);
}

void TriggerCallback(const geometry_msgs::PoseStampedConstPtr& msgPtr)
{
  detector_.setTrigger();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "target_ekf");
  ros::NodeHandle nh("~");
  ros::Subscriber team_obsrv_sub_, odom_sub_, pcl_sub_, trigger_sub_;

  detector_.init(nh);

  //Pub
  target_odom_pub_ = nh.advertise<nav_msgs::Odometry>("observe_target_odom", 200);
  target_obsrv_pub_ = nh.advertise<swarm_msgs::TargetObsrv>("target_obsrv_to_teammate", 200);

  //Sub
  team_obsrv_sub_ = nh.subscribe("target_obsrv_from_teammate", 10000, &TeamMeasCallback, ros::TransportHints().tcpNoDelay());
  odom_sub_ = nh.subscribe("odom", 100, &OdomCallback, ros::TransportHints().tcpNoDelay());
  pcl_sub_ = nh.subscribe("cloud", 100, &SelfMeasCallback, ros::TransportHints().tcpNoDelay());
  trigger_sub_ = nh.subscribe("trigger", 10, &TriggerCallback, ros::TransportHints().tcpNoDelay());
  target_true_odom_sub_ = nh.subscribe("target_true_odom", 10, &TargetOdomCallback, ros::TransportHints().tcpNoDelay());

  //Timer
  ekf_timer_ = nh.createTimer(ros::Duration(1.0 / detector_.getHz()), &EKFCallback);

  ros::spin();
  return 0;
}


