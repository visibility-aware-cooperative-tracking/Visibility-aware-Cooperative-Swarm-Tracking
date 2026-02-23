#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <traj_utils/PolyTraj.h>
#include <traj_utils/poly_traj_utils.hpp>
#include <quadrotor_msgs/PositionCommand.h>
#include <quadrotor_msgs/MpcPositionCommand.h>
#include <std_msgs/Empty.h>
#include <visualization_msgs/Marker.h>
#include <ros/ros.h>

#define DEBUG_FILE_DIR(name) (string(string(ROOT_DIR) + "log/"+name))
using namespace Eigen;
using namespace std;

ros::Publisher pos_cmd_pub, mpc_cmd_pub;

//Pos command
quadrotor_msgs::PositionCommand cmd;
quadrotor_msgs::MpcPositionCommand mpc_cmd;

//workflow control
bool receive_traj_ = false;
bool is_fov_omni_ = true;
bool is_target_ = false;
bool permanent_shut_ = false;
bool is_real_exp_ = false;
bool have_final_traj_ = false;

//params & utils
boost::shared_ptr<Trajectory> traj_;
boost::shared_ptr<Trajectory1D> traj_yaw_;
double traj_duration_;
ros::Time traj_start_time_;
int traj_id_;
ros::Time heartbeat_time_(0);
int mpc_horizon_{12};

// yaw & pos for mpc
double last_yaw_{0.0};
Eigen::Vector3d last_pos_(0,0,0);
double last_yaw_mpc_{0.0};
Eigen::Vector3d last_pos_mpc_(0,0,0);

void heartbeatCallback(std_msgs::EmptyPtr msg)
{
  heartbeat_time_ = ros::Time::now();
}

void polyTrajCallback(traj_utils::PolyTrajPtr msg)
{
  if (msg->order != 7)
  {
    ROS_ERROR("[traj_server] Only support trajectory order equals 7 now!");
    return;
  }
  if (msg->duration.size() * (msg->order + 1) != msg->coef_x.size())
  {
    ROS_ERROR("[traj_server] WRONG trajectory parameters, ");
    return;
  }

  if(msg->is_final_traj) have_final_traj_ = true;

  int piece_nums = msg->duration.size();
  std::vector<double> dura(piece_nums);
  std::vector<CoefficientMat> cMats(piece_nums);
 
  for (int i = 0; i < piece_nums; ++i)
  {
    int i8 = i * 8;
    cMats[i].row(0) << msg->coef_x[i8 + 0], msg->coef_x[i8 + 1], msg->coef_x[i8 + 2], msg->coef_x[i8 + 3], 
    msg->coef_x[i8 + 4], msg->coef_x[i8 + 5], msg->coef_x[i8 + 6], msg->coef_x[i8 + 7];
    cMats[i].row(1) << msg->coef_y[i8 + 0], msg->coef_y[i8 + 1], msg->coef_y[i8 + 2], msg->coef_y[i8 + 3],
    msg->coef_y[i8 + 4], msg->coef_y[i8 + 5], msg->coef_y[i8 + 6], msg->coef_y[i8 + 7];
    cMats[i].row(2) << msg->coef_z[i8 + 0], msg->coef_z[i8 + 1], msg->coef_z[i8 + 2], msg->coef_z[i8 + 3],
    msg->coef_z[i8 + 4], msg->coef_z[i8 + 5], msg->coef_z[i8 + 6], msg->coef_z[i8 + 7];

    dura[i] = msg->duration[i];
  }
  traj_.reset(new Trajectory(dura, cMats));

  if(!is_fov_omni_)
  {
    std::vector<CoefficientMat1D> cMatYaws(piece_nums);
    for (int i = 0; i < piece_nums; ++i)
    {
      int i4 = i * 4;
      cMatYaws[i].row(0) << msg->coef_yaw[i4 + 0], msg->coef_yaw[i4 + 1], msg->coef_yaw[i4 + 2], msg->coef_yaw[i4 + 3];
    }
    traj_yaw_.reset(new Trajectory1D(dura, cMatYaws));
  }

  traj_start_time_ = msg->start_time;
  traj_duration_ = traj_->getTotalDuration();
  traj_id_ = msg->traj_id;

  receive_traj_ = true;
}

void publish_cmd(Vector3d p, Vector3d v, Vector3d a, Vector3d j, double y, double yd)
{
  cmd.header.stamp = ros::Time::now();
  cmd.header.frame_id = "world";
  cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
  cmd.trajectory_id = traj_id_;

  cmd.position.x = p(0);
  cmd.position.y = p(1);
  cmd.position.z = p(2);
  cmd.velocity.x = v(0);
  cmd.velocity.y = v(1);
  cmd.velocity.z = v(2);
  cmd.acceleration.x = a(0);
  cmd.acceleration.y = a(1);
  cmd.acceleration.z = a(2);
  cmd.yaw = y;
  cmd.yaw_dot = yd;
  pos_cmd_pub.publish(cmd);

  last_pos_ = p;
}

void cmdCallback(const ros::TimerEvent &e)
{
  if (!receive_traj_)
    return;
  
  ros::Time time_now = ros::Time::now();

  double t_cur = (time_now - traj_start_time_).toSec();

  Eigen::Vector3d pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero()), acc(Eigen::Vector3d::Zero()), jer(Eigen::Vector3d::Zero());
  std::pair<double, double> yaw_yawdot(0, 0);

  static ros::Time time_last = ros::Time::now();
  if (t_cur < traj_duration_ && t_cur >= 0.0)
  {
    pos = traj_->getPos(t_cur);
    vel = traj_->getVel(t_cur);
    acc = traj_->getAcc(t_cur);
    jer = traj_->getJer(t_cur);
  
    if(is_fov_omni_)   
    {
      yaw_yawdot.first = 0.0;
      yaw_yawdot.second = 0.0;
    }else{  
      yaw_yawdot.first = traj_yaw_->getPos(t_cur);
      yaw_yawdot.second = traj_yaw_->getVel(t_cur);
    }
    time_last = time_now;
    last_yaw_ = yaw_yawdot.first;
    last_pos_ = pos;

    publish_cmd(pos, vel, acc, jer, yaw_yawdot.first, yaw_yawdot.second);
  }
}

void odomCallback(const nav_msgs::OdometryConstPtr &msg)
{
  if(!receive_traj_)
  {
    last_pos_(0) = msg->pose.pose.position.x;
    last_pos_(1) = msg->pose.pose.position.y;
    last_pos_(2) = msg->pose.pose.position.z;
    last_pos_mpc_ = last_pos_;

    tf::Quaternion q(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w
    );

    double roll, pitch, yaw;
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);

    last_yaw_ = yaw;
    last_yaw_mpc_ = yaw;
  }
}

void mpccmdCallback(const ros::TimerEvent &e)
{
  if(permanent_shut_)
  {
    ROS_ERROR_ONCE("[traj_server] Permanently shut.");
    return;
  }

  if (heartbeat_time_.toSec() <= 1e-5)
  {
    ROS_ERROR_ONCE("[traj_server] No heartbeat from the planner received");
    return;
  }

  if (!receive_traj_)
    return;
    
  bool receive_beat{true};
  static ros::Time last_t = ros::Time::now();

  std::pair<double, double> yyd;
  ros::Time cur_t = ros::Time::now();

  if ((!have_final_traj_) && (cur_t - heartbeat_time_).toSec() > 0.55)
  {
    ROS_ERROR("[traj_server] Lost heartbeat from the planner, is he dead?");
    receive_beat = false;
    permanent_shut_ = true;
  }
  
  double eval_t = (cur_t - traj_start_time_).toSec();
  mpc_cmd.mpc_horizon = mpc_horizon_;
  mpc_cmd.header.stamp = ros::Time::now();
  mpc_cmd.header.frame_id = "world";
  constexpr double mpc_dt = 0.01;
  double local_last_t = last_t.toSec();


  for (int i = 0; i < mpc_horizon_; i++) {
    double local_t = eval_t + mpc_dt * i;
    local_t = local_t > traj_duration_ ? traj_duration_ : local_t;

    Eigen::Vector3d p = traj_->getPos(local_t);
    Eigen::Vector3d v = traj_->getVel(local_t);
    Eigen::Vector3d a = traj_->getAcc(local_t);
    Eigen::Vector3d j = traj_->getJer(local_t);
    if(!is_fov_omni_)
    {
      yyd.first = traj_yaw_->getPos(local_t);
      yyd.second = traj_yaw_->getVel(local_t);
    }else{
      yyd.first = last_yaw_mpc_;
      yyd.second = 0.0;
    }
    if(!receive_beat){
      mpc_cmd.cmds[i].trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_EMER;
      mpc_cmd.cmds[i].trajectory_id = traj_id_;
      mpc_cmd.cmds[i].vel_norm = 0;
      mpc_cmd.cmds[i].acc_norm = 0;
      mpc_cmd.cmds[i].position.x = last_pos_mpc_(0);
      mpc_cmd.cmds[i].position.y = last_pos_mpc_(1);
      mpc_cmd.cmds[i].position.z = last_pos_mpc_(2);
      mpc_cmd.cmds[i].velocity.x = 0;
      mpc_cmd.cmds[i].velocity.y = 0;
      mpc_cmd.cmds[i].velocity.z = 0;
      mpc_cmd.cmds[i].acceleration.x = 0;
      mpc_cmd.cmds[i].acceleration.y = 0;
      mpc_cmd.cmds[i].acceleration.z = 0;
  	  mpc_cmd.cmds[i].jerk.x = 0.0;
  	  mpc_cmd.cmds[i].jerk.y = 0.0;
  	  mpc_cmd.cmds[i].jerk.z = 0.0;
      mpc_cmd.cmds[i].yaw = last_yaw_mpc_;
      mpc_cmd.cmds[i].yaw_dot = 0.0;
    }else{
      if(i==0){
        last_pos_mpc_ = p;
        last_yaw_mpc_ = yyd.first;
      }
      
      mpc_cmd.cmds[i].trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
      mpc_cmd.cmds[i].trajectory_id = traj_id_;
      mpc_cmd.cmds[i].vel_norm = v.norm();
      mpc_cmd.cmds[i].acc_norm = a.norm();
      mpc_cmd.cmds[i].position.x = p(0);
      mpc_cmd.cmds[i].position.y = p(1);
      mpc_cmd.cmds[i].position.z = p(2);
      mpc_cmd.cmds[i].velocity.x = v(0);
      mpc_cmd.cmds[i].velocity.y = v(1);
      mpc_cmd.cmds[i].velocity.z = v(2);
      mpc_cmd.cmds[i].acceleration.x = a(0);
      mpc_cmd.cmds[i].acceleration.y = a(1);
      mpc_cmd.cmds[i].acceleration.z = a(2);
      mpc_cmd.cmds[i].jerk.x = j(0);
      mpc_cmd.cmds[i].jerk.y = j(1);
      mpc_cmd.cmds[i].jerk.z = j(2);
      mpc_cmd.cmds[i].yaw = yyd.first;
      mpc_cmd.cmds[i].yaw_dot = yyd.second;
    }
  } 
  mpc_cmd_pub.publish(mpc_cmd);

  last_t = cur_t;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "traj_server");
  // ros::NodeHandle node;
  ros::NodeHandle nh("~");

  nh.param("traj_server/is_target", is_target_, false);
  nh.param("traj_server/is_fov_omni", is_fov_omni_, false);
  nh.param("traj_server/mpc_horizon", mpc_horizon_, 12);
  nh.param("traj_server/is_real_exp", is_real_exp_, false);
  if(is_target_) is_fov_omni_ = true;
  mpc_cmd.cmds.resize(mpc_horizon_);
  ros::Subscriber poly_traj_sub = nh.subscribe("planning/trajectory", 10, &polyTrajCallback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber self_odom_sub = nh.subscribe("self_odom", 10, &odomCallback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber heartbeat_sub;
  ros::Timer mpc_cmd_timer;
  ros::Timer cmd_timer;
  if(is_real_exp_)
  {
    heartbeat_sub = nh.subscribe("/heartbeat", 10, &heartbeatCallback, ros::TransportHints().tcpNoDelay());
    mpc_cmd_pub = nh.advertise<quadrotor_msgs::MpcPositionCommand>("/mpc_position_cmd", 10);
    mpc_cmd_timer = nh.createTimer(ros::Duration(0.01), &mpccmdCallback);
  }else{
    pos_cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 50);
    cmd_timer = nh.createTimer(ros::Duration(0.01), &cmdCallback);
  }
    
  ros::Duration(1.0).sleep();

  ROS_INFO("[traj_server]: ready.");

  ros::spin();
  
  return 0;
}