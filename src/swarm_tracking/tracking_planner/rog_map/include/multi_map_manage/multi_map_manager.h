#ifndef _MULTI_MAP_MANAGER_H
#define _MULTI_MAP_MANAGER_H

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <ros/serialization.h>
#include <ros/package.h>

#include <swarm_msgs/ChunkStamps.h>
#include <swarm_msgs/LocalMapData.h>
#include <swarm_msgs/LocalMapList.h>

#include <swarm_msgs/SpatialTemporalOffset.h>
#include <swarm_msgs/SpatialTemporalOffsetStatus.h>
#include <std_msgs/Empty.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <ifaddrs.h>

#include <multi_map_manage/hilbert_2d.hpp>
#include <rog_map/rog_map.h>

#include <memory>
#include <random>
#include <vector>
#include <unordered_map>
#include <map>
#include <mutex>
#include <zlib.h>
#include <omp.h>

using std::shared_ptr;
using std::vector;
using std::unordered_map;
using std::map;

namespace rog_map {
class ROGMap;
class ProbMap;

struct MapChunk{
    swarm_msgs::LocalMapData map_msg_data_;
    bool empty_;
};

struct TeammateOffset
{   
    TeammateOffset(const swarm_msgs::SpatialTemporalOffset &msg)
    {
        this->drone_id = msg.teammate_id;
        this->time_offset = msg.time_offset;
        this->trans << msg.trans[0],
                       msg.trans[1],
                       msg.trans[2];
        this->quat.w() = msg.rot_quaternion[0];
        this->quat.x() = msg.rot_quaternion[1];
        this->quat.y() = msg.rot_quaternion[2];
        this->quat.z() = msg.rot_quaternion[3];
        this->rot = this->quat.toRotationMatrix();
    };

    void set(const swarm_msgs::SpatialTemporalOffset &msg)
    {
        this->drone_id = msg.teammate_id;
        this->time_offset = msg.time_offset;
        this->trans << msg.trans[0],
                       msg.trans[1],
                       msg.trans[2];
        this->quat.w() = msg.rot_quaternion[0];
        this->quat.x() = msg.rot_quaternion[1];
        this->quat.y() = msg.rot_quaternion[2];
        this->quat.z() = msg.rot_quaternion[3];
        this->rot = this->quat.toRotationMatrix();
    };

    int drone_id;
    double time_offset;
    Eigen::Vector3d trans;
    Eigen::Quaterniond quat;
    Eigen::Matrix3d rot;    
};

typedef map<int, TeammateOffset> id_offset_map;
typedef id_offset_map::value_type id_offset_pair;

class MultiMapManager {

public:
  int drone_id_;
  ros::NodeHandle node_;
  MultiMapManager();
  ~MultiMapManager();
  void setMap(ROGMap* map);
  void init();
 
  void addUpdatedVoxels(const Vec3f& pos,  const int& hash_id);


  bool get_local_ip(string &local_ip) {
      char ip[16];
      struct ifaddrs *ifAddrStruct;
      void *tmpAddrPtr=NULL;
      getifaddrs(&ifAddrStruct);
      while (ifAddrStruct != NULL) {
          if (ifAddrStruct->ifa_addr->sa_family==AF_INET) {
              tmpAddrPtr = &((struct sockaddr_in *) ifAddrStruct->ifa_addr)->sin_addr;
              inet_ntop(AF_INET, tmpAddrPtr, ip, INET_ADDRSTRLEN);
              if ((ip[0] == '1' && ip[1] == '0' && ip[3] == '0')||
                  (ip[8] == '2' && ip[9] == '3' && ip[10] == '4' && ip[12] == '1')) {
                  printf("%s IP Address: %s\n", ifAddrStruct->ifa_name, ip);
                  local_ip = ip;
                
                  return true;
              }
          }
          ifAddrStruct=ifAddrStruct->ifa_next;
      }
      
      freeifaddrs(ifAddrStruct);
      return false;
  }

  void StringIp2CharIp(string &str_ip, uint8_t *ch_ip) {
      std::stringstream s(str_ip);
      int data[4];
      char ch; //to temporarily store the '.'
      s >> data[0] >> ch >> data[1] >> ch >> data[2] >> ch >> data[3];
      for (int i = 0; i < 4; i++) {
          ch_ip[i] = data[i];
      }
  }

private:
  std::vector<int> getTwobitsStatus_highreshilbert_update(const Eigen::Vector3i& box_min, const Eigen::Vector3i& box_max, bool flag_update_all);
  void sortHilbertAndReorderAdrs(std::vector<uint32_t>& hilbert_idx, std::vector<Eigen::Vector2i>& voxel_adrs);
  void insertLocalMap_hilbert(const Eigen::Vector3i& box_min, const Eigen::Vector3i& box_max, const int& from_id,
                              std::vector<int> attr, const double& pack_time);
  void sort_2D_hilbert(int x_min, int x_max, int y_min, int y_max, std::vector<uint64_t>& hilbert_2d_index);

  // check mechanism at the application layer
  void sendChunks(const int& to_drone_id, const vector<int>& idx_list);
  void stampTimerCallback(const ros::TimerEvent& e);
  void stampMsgCallback(const swarm_msgs::ChunkStampsConstPtr& msg);
  void chunkMsgCallback(const swarm_msgs::LocalMapListConstPtr& msg_list);
  void packChunkTimerCallback(const ros::TimerEvent& e);

  // Operations on the chunk idx list
  void findMissedChunkIds(
      const vector<int>& self_idx_list, const vector<int>& other_idx_list, vector<int>& miss_ids);
  bool findIntersect(
      const int& min1, const int& max1, const int& min2, const int max2, int& minr, int& maxr);
  bool isIdsInclude(const vector<int>& idx_list, const int& id);
  void trimChunkIds(vector<int>& idx_list, const int& trim_id);
  
  void mergeChunkIds(const vector<int>& list1, const vector<int>& list2, vector<int>& output);
  void insertLocalmap(const swarm_msgs::LocalMapData& msg, const int& from_id);
  int getIdsListNum(const vector<int>& idx_list);
  void packAndSendRemedyChunk(const vector<int>& missed_idx_list, int to_drone_id);

  // compress data
  std::vector<unsigned char> packData(const std::vector<int>& data); 
  std::vector<int> unpackData(const std::vector<unsigned char>& packedData, size_t originalSize);
  std::vector<unsigned char> compressData(const std::vector<unsigned char>& packedData);
  std::vector<unsigned char> decompressData(const std::vector<unsigned char>& compressedData, uLongf originalSize);

  // Extrinsic related
  // Important Note: Extrinsic module only handles same global map origin at Zeros
  template<typename arr_3>
  void SE3offset(arr_3& v, const Eigen::Matrix3d& R, const Eigen::Vector3d& t);
  void STOffsetCallback(const swarm_msgs::SpatialTemporalOffsetStatus::ConstPtr &msg);
  bool OffsetTmtPt(Eigen::Vector3d& pos, int tmt_id);

  // data
  int map_num_;

  ros::Publisher localmap_pub_, localmaplist_pub_, all_map_pub_, localmap_box_pub, stamp_pub_, missed_chunk_pub_, empty_pub_;
  ros::Subscriber localmap_sub_, stamp_sub_, chunk_sub_, ST_offset_sub_;
  ros::Timer trans_timer_, map_pub_timer_, stamp_timer_, pack_chunk_timer_;
  ros::Time pre_t_, last_packup_time_, u_time_;

  // Swarm Extrinsic
  double last_offset_stamp_{-1.0};
  id_offset_map ST_offsets_;

  Eigen::Vector3d drone_pos_;
  Eigen::Vector3d sensor_pos_;
  std::ofstream transmit_log_, recv_log_, timediff_log_, timediff_offset_log_;
  std::ofstream pack_send_diff_log_, recv_insert_diff_log_, packing_time_log_, insert_compare_log_;

  bool high_res_ = false;
  ROGMap* map_;

  // data for check
  vector<MapChunk> self_map_chunks_;
  vector<vector<int>> swarm_chunk_idx_list_; 

  vector<vector<swarm_msgs::LocalMapData>> chunk_to_insert_buffer_;  

  vector<pair<int, int>> last_send_idx_;
  vector<vector<int>> missed_list_;

  vector<uint32_t> updated_voxel_ids_buffer_;
  std::vector<int> is_voxel_update_buffer_;
  Eigen::Vector3i localupdate_min_; 
  Eigen::Vector3i localupdate_max_;

  vector<int> remedy_upto_idx_;


  bool is_real_experiment_{false};

  int voxel_num_threshold_;
  double time_threshold_;
  int remedy_idx_num_threshold_;

  int buffer_size_;
  int total_bw_; 

  
};

}  

#endif