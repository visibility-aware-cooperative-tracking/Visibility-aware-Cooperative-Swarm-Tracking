#ifndef _TRAJ_SET_H_
#define _TRAJ_SET_H_

#include <Eigen/Eigen>
#include <vector>
#include <ros/ros.h>

#include <traj_utils/poly_traj_utils.hpp>

struct TrajData
{
  bool have_yaw{false};
  bool is_stop_traj{false};
  bool is_final_traj{false};
  bool is_preset_traj{true};
  Trajectory traj;
  int drone_id{-1}; // A negative value indicates no received trajectories.
  int traj_id;
  double duration;
  double start_time; // world time
  Eigen::Vector3d start_pos;

  Trajectory1D traj_yaw;
};

typedef std::vector<TrajData> SwarmTrajData;

class TrajSet
{
public:

  TrajData local_traj;
  SwarmTrajData swarm_traj;

  TrajSet()
  {
    local_traj.traj_id = 0;
  }
  ~TrajSet() {}

  void setTraj(const Trajectory &trajectory, const double &world_time, const int drone_id = -1, 
               bool is_stop = false, bool is_final = false)
  {
    local_traj.drone_id = drone_id;
    local_traj.traj_id++;
    local_traj.duration = trajectory.getTotalDuration();
    local_traj.start_pos = trajectory.getJuncPos(0);
    local_traj.start_time = world_time;
    local_traj.traj = trajectory;
    local_traj.is_stop_traj = is_final ? true : is_stop;
    local_traj.is_final_traj = is_final;
    local_traj.have_yaw = false;
  }

  void setTraj(const Trajectory &trajectory, const Trajectory1D &trajectory_yaw, const double &world_time, 
               const int drone_id = -1, bool is_stop = false, bool is_final = false)
  {
    local_traj.drone_id = drone_id;
    local_traj.traj_id++;
    local_traj.duration = trajectory.getTotalDuration();
    local_traj.start_pos = trajectory.getJuncPos(0);
    local_traj.start_time = world_time;
    local_traj.traj = trajectory;
    local_traj.is_stop_traj = is_final ? true : is_stop;
    local_traj.is_final_traj = is_final;
    local_traj.have_yaw = true;
    local_traj.traj_yaw = trajectory_yaw;
  }
}; 

#endif