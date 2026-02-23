#include <ros/ros.h>
#include <boost/thread.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include "udp_bridge/protocol.h"
#include "algorithm"
#include <string>
#include <map>
#include "ros/package.h"
#include "swarm_msgs/TeamStatus.h"
#include "swarm_msgs/TeammateInfo.h"
#include "swarm_msgs/QuadStatePub.h"
#include "swarm_msgs/ObserveTeammate.h"
#include "swarm_msgs/GlobalExtrinsicStatus.h"
#include "swarm_msgs/GlobalExtrinsic.h"
#include "swarm_msgs/SpatialTemporalOffset.h"
#include "swarm_msgs/SpatialTemporalOffsetStatus.h"
#include "swarm_msgs/MincoTraj.h"
#include "swarm_msgs/TargetObsrv.h"
#include "swarm_msgs/ChunkStamps.h"
#include "swarm_msgs/LocalMapData.h"
#include "swarm_msgs/LocalMapList.h"
#include "swarm_msgs/TeammateIDList.h"

#include <ifaddrs.h>
#include "Teammate.hpp"
#include <errno.h>
#include <ctime>
#include <csignal>
#include <boost/filesystem.hpp>
#include <unordered_map>
#include <Eigen/Eigen>
#include <std_msgs/Empty.h>

using namespace Eigen;
using namespace std;
#define UDP_PORT 8830
#define BUFFER_SIZE 500000
#define OBSERVE_MSG_TYPE 0x04u

using namespace udp_bridge;
typedef unordered_map<int, Teammate> id_teammate_map;
typedef id_teammate_map::value_type position;

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

bool exit_process = false;
void SigHandle(int sig) {
    ROS_WARN("Exit the process, catch sig %d", sig);
    exit_process = true;
}

class UdpBridge {
private:
    int udp_server_fd_;
    int udp_send_ip_fd_ptr_;
    char udp_recv_buf_[BUFFER_SIZE];
    string local_ip;
    int local_id;
    int leader_id_;
    id_teammate_map teammates;
    string broadcast_ip, offset_path;
    ofstream save_offset, write_team_status;

    Eigen::Vector3d relative_p_org_;
    Eigen::Vector3d relative_p_;


    double last_offset_stamp_{-1.0};
    id_offset_map ST_offsets_;
    int local_drone_id_{-1};

    sockaddr_in addr_udp_send_ip_;
    boost::thread *udp_callback_thread_;
    ros::NodeHandle nh_;
    ros::Publisher team_status_pub_;
    ros::Timer sync_timer_, broadcast_timer_;
    
    ros::Publisher MincoTraj_pub_, ChunkStamp_pub_, MapLocal_pub_, TargetObsrv_pub_;
    ros::Subscriber MincoTraj_sub_, ChunkStamp_sub_, MapLocal_sub_, TargetObsrv_sub_;
    ros::Subscriber ST_offset_sub_;

    ros::Subscriber teammate_id_list_sub_; 
    
    int tmt_id_num_{0}; 
    std::vector<int> swarm_id_list_;

    

private:
    struct IpIdData {
        ros::Timer process_timer;
        bool rcv_new_msg{false};
        mutex update_lock_;
        IpIdMsgCvt latest_msg, processing_msg;
        ros::Time rcv_WT; 
    } ip_id_data_;

    void IpIdMsgCallback(const ros::TimerEvent &e) {
        if (!ip_id_data_.rcv_new_msg) {
            return;
        }
        ip_id_data_.update_lock_.lock();
        ip_id_data_.rcv_new_msg = false;
        ip_id_data_.processing_msg = ip_id_data_.latest_msg;
        ip_id_data_.update_lock_.unlock();
        

        //Record IP
        uint8_t *data = ip_id_data_.processing_msg.data.local_ip;
        string rcv_ip;
        CharIp2StringIp(data, rcv_ip);

        //Record ID
        int rcv_id = ip_id_data_.processing_msg.data.local_id;
        if (rcv_id == local_id) {
            return;
        }
        auto iter = teammates.find(rcv_id);
        if (iter == teammates.end()) {
            Teammate drone(rcv_ip, rcv_id, ip_id_data_.rcv_WT.toSec());
            drone.udp_send_fd_ptr_ = InitUdpUnicast(drone, UDP_PORT);
            teammates.insert(position(rcv_id, drone));
            ROS_WARN_STREAM(" -- [Found new teammate]: Drone " <<rcv_id << ", " << rcv_ip <<"\n");
        } else {
            iter->second.last_rcv_time_ = ip_id_data_.rcv_WT.toSec();
        }
    }

//// Minco Traj MSG ////
private:
    struct MincoTrajData{
        ros::Timer process_timer;
        bool rcv_new_msg{false};
        mutex update_lock_;
        swarm_msgs::MincoTraj latest_msg, processing_msg;
        ros::Time rcv_WT;
    }minco_traj_data_;

    void MincoTrajThreadCallback(const ros::TimerEvent &e) {
        if(!minco_traj_data_.rcv_new_msg){
            return;
        }
        minco_traj_data_.update_lock_.lock();
        minco_traj_data_.rcv_new_msg = false;
        minco_traj_data_.processing_msg = minco_traj_data_.latest_msg;
        minco_traj_data_.update_lock_.unlock();

       
        OffsetMincoTraj(minco_traj_data_.processing_msg);
        MincoTraj_pub_.publish(minco_traj_data_.processing_msg);
    }

//// Target Obsrv MSG ////
private:
    struct TargetObsrvData{
        ros::Timer process_timer;
        bool rcv_new_msg{false};
        mutex update_lock_;
        swarm_msgs::TargetObsrv latest_msg, processing_msg;
        ros::Time rcv_WT;
    }target_obsrv_data_;

    void TargetObsrvThreadCallback(const ros::TimerEvent &e) {
        if(!target_obsrv_data_.rcv_new_msg){
            return;
        }
        target_obsrv_data_.update_lock_.lock();
        target_obsrv_data_.rcv_new_msg = false;
        target_obsrv_data_.processing_msg = target_obsrv_data_.latest_msg;
        target_obsrv_data_.update_lock_.unlock();

        OffsetTargetObsrv(target_obsrv_data_.processing_msg);
        TargetObsrv_pub_.publish(target_obsrv_data_.processing_msg);
    }


////// ChunkStamp MSG //////
private:
    struct ChunkStampData{
        ros::Timer process_timer;
        bool rcv_new_msg{false};
        mutex update_lock_;
        swarm_msgs::ChunkStamps latest_msg, processing_msg;
        ros::Time rcv_WT;
    }chunk_stamp_data_;

    void ChunkStampThreadCallback(const ros::TimerEvent &e) {
        if(!chunk_stamp_data_.rcv_new_msg){
            return;
        }
        chunk_stamp_data_.update_lock_.lock();
        chunk_stamp_data_.rcv_new_msg = false;
        chunk_stamp_data_.processing_msg = chunk_stamp_data_.latest_msg;
        chunk_stamp_data_.update_lock_.unlock();
        
        ChunkStamp_pub_.publish(chunk_stamp_data_.processing_msg);
    }

////// MapLocal MSG //////
private:
    struct MapLocalData{
        ros::Timer process_timer;
        bool rcv_new_msg{false};
        mutex update_lock_;
        swarm_msgs::LocalMapList latest_msg, processing_msg;
        ros::Time rcv_WT;
    }map_local_data_;

    void MapLocalThreadCallback(const ros::TimerEvent &e) {
        if(!map_local_data_.rcv_new_msg){
            return;
        }
        map_local_data_.update_lock_.lock();
        map_local_data_.rcv_new_msg = false;
        map_local_data_.processing_msg = map_local_data_.latest_msg;
        map_local_data_.update_lock_.unlock();
        
        MapLocal_pub_.publish(map_local_data_.processing_msg);
    }

    void initDateCallback(){
        ip_id_data_.process_timer = nh_.createTimer(ros::Duration(0.0002), &UdpBridge::IpIdMsgCallback, this);
        minco_traj_data_.process_timer = nh_.createTimer(ros::Duration(0.0001), &UdpBridge::MincoTrajThreadCallback, this);
        chunk_stamp_data_.process_timer = nh_.createTimer(ros::Duration(0.003), &UdpBridge::ChunkStampThreadCallback, this);
        map_local_data_.process_timer = nh_.createTimer(ros::Duration(0.003), &UdpBridge::MapLocalThreadCallback, this);
        target_obsrv_data_.process_timer = nh_.createTimer(ros::Duration(0.001), &UdpBridge::TargetObsrvThreadCallback, this);
    }

public:

    UdpBridge(ros::NodeHandle &nh) {
        nh_ = nh;

        signal(SIGINT, SigHandle);
        //Acquire LOCAL IP
        char ip[16];
        memset(ip, 0, sizeof(ip));
        get_local_ip(ip);
        local_ip = ip;
         
        //Set DRONE ID
        uint8_t *ip_c = new uint8_t[4];
        StringIp2CharIp(local_ip, ip_c);
        local_id = ip_c[3] - 100;
        local_drone_id_ = local_id;
        
        ROS_WARN_STREAM("UDP UniCast Launching.");
        ROS_WARN_STREAM("[UDP] Local Drone ID: " << local_drone_id_);

        //Set Broadcast IP
        ip_c[3] = 255;
        CharIp2StringIp(ip_c, broadcast_ip);

        ROS_WARN_STREAM(" -- [BROAD IP]: {}\n" <<  broadcast_ip);
        ROS_WARN_STREAM(" -- [LOCAL IP]: {}\n" <<  local_ip);
        ROS_WARN_STREAM(" -- [DRONE ID]: {}\n" << local_id);

        // write to log and shutdown
        offset_path = ros::package::getPath("udp_bridge");
        offset_path += "../../../../config";
        boost::filesystem::create_directories(offset_path);
        offset_path += "/teammate_" + GetSystemTime() + ".txt";
        save_offset.open(offset_path, ios::out);

        nh_.param("optimization/leader_id", leader_id_, -1);
        nh_.param("optimization/relative_x", relative_p_org_(0), 2.0);
        nh_.param("optimization/relative_y", relative_p_org_(1), 2.0);
        nh_.param("optimization/relative_z", relative_p_org_(2), 0.0);
        relative_p_ = relative_p_org_;
        ROS_WARN_STREAM("[UDP UNICAST] optimization/leader_id: " << leader_id_);
     
        //Init fd for sending IP and drone ID
        udp_send_ip_fd_ptr_ = InitUdpBoardcast(UDP_PORT);
        udp_callback_thread_ = new boost::thread(boost::bind(&UdpBridge::UdpCallback, this));
        broadcast_timer_ = nh_.createTimer(ros::Duration(1), &UdpBridge::BroadcastCallback, this);
    
        MincoTraj_pub_ = nh_.advertise<swarm_msgs::MincoTraj>("/minco_traj_from_teammate", 10);
        MincoTraj_sub_ = nh_.subscribe("/minco_traj_to_teammate", 10, &UdpBridge::MincoTrajMsgCallback, this, ros::TransportHints().tcpNoDelay());
        ST_offset_sub_ = nh_.subscribe("/spatial_temporal_offset", 1000, &UdpBridge::STOffsetCallback, this, ros::TransportHints().tcpNoDelay());
        
        TargetObsrv_pub_ = nh_.advertise<swarm_msgs::TargetObsrv>("/target_obsrv_from_teammate", 10000);
        TargetObsrv_sub_ = nh_.subscribe("/target_obsrv_to_teammate", 1000, &UdpBridge::TargetObsrvCallback, this);
        ChunkStamp_pub_ = nh_.advertise<swarm_msgs::ChunkStamps>("/chunk_stamp_from_teammate", 10);
        ChunkStamp_sub_ = nh_.subscribe("/chunk_stamp_to_teammate", 10, &UdpBridge::ChunkStampMsgCallback, this, ros::TransportHints().tcpNoDelay());
        MapLocal_pub_ = nh_.advertise<swarm_msgs::LocalMapList>("/local_map_from_teammate", 10);
        MapLocal_sub_ = nh_.subscribe("/local_map_to_teammate", 10, &UdpBridge::MapLocalMsgCallback, this, ros::TransportHints().tcpNoDelay());
        
        teammate_id_list_sub_ = nh.subscribe("/teammate_id_list", 10, &UdpBridge::teammateIDListCallback, this, ros::TransportHints().tcpNoDelay());


        initDateCallback();
        ros::Duration(0.1).sleep();

    }

    ~UdpBridge() {
        close(udp_server_fd_);
    }

    string GetSystemTime()
    {
    time_t now = time(NULL);
        tm* t = localtime(&now);
        stringstream ss;
        ss << t->tm_mon + 1 << "_" <<
            t->tm_mday << "_" << t->tm_hour << "_" << t->tm_min << "_" << t->tm_sec;
    return ss.str();
    }

    double str2double(string s) {
        double d;
        stringstream ss;
        ss << s;
        ss >> setprecision(16) >> d;
        ss.clear();
        return d;
    }

    void BroadcastCallback(const ros::TimerEvent &e) {
        //Write team status into .txt
        if (exit_process) {
            //Write down the time offset

//                   |             |               |
//         local_id  | teammate_id | offset_time(s)| teammate_ip
//                   |             |               |

            for (auto it = teammates.begin(); it != teammates.end(); it++) {
                auto &drone = it->second;
                if (!drone.write_done_) {
                    if (drone.sync_done_) {
                        save_offset << local_id << " " << drone.id_ << " " << drone.offset_ts_[10] << " " << drone.ip_
                                    << endl;
                        drone.write_done_ = true;
                    }
                }
            }
            save_offset.close();
            ros::shutdown();
        }
        //Broadcast local Ip and Id
        SendLocalIp();
    }

    void UdpCallback() {
        int valread;
        struct sockaddr_in addr_client;
        socklen_t addr_len;

        // Connect
        if (BindToUdpPort(UDP_PORT, udp_server_fd_) < 0) {
            ROS_ERROR("[bridge_node]Socket receiver creation error!");
            exit(EXIT_FAILURE);
        }

        while (true) {
            if ((valread = recvfrom(udp_server_fd_, udp_recv_buf_, BUFFER_SIZE, 0, (struct sockaddr *) &addr_client,
                                    (socklen_t *) &addr_len)) < 0) {
                perror("recvfrom() < 0, error:");
                exit(EXIT_FAILURE);
            }
            ros::Time t2 = ros::Time::now();
    
            char *ptr = udp_recv_buf_;
            switch (*((MESSAGE_TYPE *) ptr)) {
         
                case MESSAGE_TYPE::IP_ID: {
                    IpIdMsgCvt rcv_msg;
                    DecodeMsgFromBuffer(rcv_msg);
                    ip_id_data_.update_lock_.lock();
                    ip_id_data_.latest_msg = rcv_msg;
                    ip_id_data_.rcv_WT = t2;
                    ip_id_data_.rcv_new_msg = true;
                    ip_id_data_.update_lock_.unlock();
                    break;
                }

                case MESSAGE_TYPE::MINCO_TRAJ: {
                    minco_traj_data_.update_lock_.lock();
                    int len = DeserializeMsgFromBuffer(minco_traj_data_.latest_msg);
                    minco_traj_data_.rcv_WT = t2;
                    minco_traj_data_.rcv_new_msg = true;
                    minco_traj_data_.update_lock_.unlock();
                    break;
                }

                case MESSAGE_TYPE::CHUNK_STAMP: {
                    chunk_stamp_data_.update_lock_.lock();
                    int len = DeserializeMsgFromBuffer(chunk_stamp_data_.latest_msg);
                    chunk_stamp_data_.rcv_WT = t2;
                    chunk_stamp_data_.rcv_new_msg = true;
                    chunk_stamp_data_.update_lock_.unlock();
                    break;
                }

                case MESSAGE_TYPE::MAP_LOCAL: {
                    map_local_data_.update_lock_.lock();
             
                    int len = DeserializeMsgFromBuffer(map_local_data_.latest_msg);
                    map_local_data_.rcv_WT = t2;
                    map_local_data_.rcv_new_msg = true;
                    map_local_data_.update_lock_.unlock();
                    break;
                }

                case MESSAGE_TYPE::TARGET_OBSRV: {
                    target_obsrv_data_.update_lock_.lock();
                    int len = DeserializeMsgFromBuffer(target_obsrv_data_.latest_msg);
                    target_obsrv_data_.rcv_WT = t2;
                    target_obsrv_data_.rcv_new_msg = true;
                    target_obsrv_data_.update_lock_.unlock();
                    break;
                }

                default:
                    break;
            }
        }
    }

private:

    void teammateIDListCallback(const swarm_msgs::TeammateIDListConstPtr &msg)
    {
        //Load the teammateIDList to swarm_id_list
        std::vector<int> msg_list; msg_list.clear();
        for(auto id : msg->teammate_ids)
        msg_list.emplace_back((int)id);

        swarm_id_list_ = {local_drone_id_};
        if(msg_list.empty()){
            tmt_id_num_ = 0;
        }else{
            for(auto id : msg_list)
            {
                if(id == local_drone_id_) continue;
                swarm_id_list_.emplace_back(id);
            }
            std::sort(swarm_id_list_.begin(), swarm_id_list_.end());
            tmt_id_num_ = swarm_id_list_.size() - 1;
        }
    }

    template<typename arr_3>
    void SE3offset(arr_3& v, const Eigen::Matrix3d& R, const Eigen::Vector3d& t)
    {
        Eigen::Vector3d vec1(v[0],v[1],v[2]);
        Eigen::Vector3d vec2 = R * vec1 + t;
        v[0] = vec2(0); v[1] = vec2(1); v[2] = vec2(2);
    }

    void OffsetMincoTraj(swarm_msgs::MincoTraj& minco_traj)
    {   
        auto offset_iter = ST_offsets_.find(minco_traj.drone_id);
        if(offset_iter != ST_offsets_.end())
        {   
            //temporal offset
            minco_traj.start_time.fromSec(minco_traj.start_time.toSec() - offset_iter->second.time_offset);
            //spatial offset
            Eigen::Matrix3d rot = offset_iter->second.rot;
            Eigen::Vector3d trans = offset_iter->second.trans;
            Eigen::Vector3d zeros(0,0,0);
            SE3offset(minco_traj.start_p, rot, trans);
            SE3offset(minco_traj.start_v, rot, zeros);
            SE3offset(minco_traj.start_a, rot, zeros);
            SE3offset(minco_traj.start_j, rot, zeros);
            SE3offset(minco_traj.end_p, rot, trans);
            SE3offset(minco_traj.end_v, rot, zeros);
            SE3offset(minco_traj.end_a, rot, zeros);
            SE3offset(minco_traj.end_j, rot, zeros);
            for(int i = 0; i < minco_traj.inner_x.size(); i++)
            {
                Eigen::Vector3d innerPt(minco_traj.inner_x[i], minco_traj.inner_y[i], minco_traj.inner_z[i]);
                SE3offset(innerPt, rot, trans);
                minco_traj.inner_x[i] = innerPt(0);
                minco_traj.inner_y[i] = innerPt(1);
                minco_traj.inner_z[i] = innerPt(2);
            }
            ROS_WARN_STREAM("Corrected One Traj from drone : " << minco_traj.drone_id << " , Traj ID: " << minco_traj.traj_id << ", Now offset num: " << ST_offsets_.size());
        }
    }

    void OffsetTargetObsrv(swarm_msgs::TargetObsrv& target_obsrv)
    {
        auto offset_iter = ST_offsets_.find(target_obsrv.drone_id);
        if(offset_iter != ST_offsets_.end())
        {   
            //temporal offset
            target_obsrv.header.stamp.fromSec( target_obsrv.header.stamp.toSec() - offset_iter->second.time_offset);
            //spatial offset
            Eigen::Matrix3d rot = offset_iter->second.rot;
            Eigen::Vector3d trans = offset_iter->second.trans;
            Eigen::Vector3d zeros(0,0,0);
            
            SE3offset(target_obsrv.target_pos, rot, trans);
        }
    }

    void STOffsetCallback(const swarm_msgs::SpatialTemporalOffsetStatus::ConstPtr &msg)
    {  
        if(local_drone_id_ != msg->drone_id)
        {
            return;
        }
        if(msg->header.stamp.toSec() < last_offset_stamp_)
        {
            ROS_ERROR("[bridge_node] Outdated GEs.");
            return;
        }
        for(int i = 0; i < msg->st_offset.size(); i++)
        {
            int tmt_id = msg->st_offset[i].teammate_id;
            auto iter = ST_offsets_.find(tmt_id);
            if(iter != ST_offsets_.end())
            {
                iter->second.set(msg->st_offset[i]);
            }else{
                TeammateOffset new_offset(msg->st_offset[i]);
                ST_offsets_.insert(id_offset_pair(tmt_id, new_offset));
                ROS_WARN_STREAM("\033[32;1m[bridge_node] New teammate " << tmt_id << " offset INSERTED.\033[0m");
            }
        }
    }

    void SendLocalIp() {
        IpIdMsgCvt cvt;
        StringIp2CharIp(local_ip, cvt.data.local_ip);
        cvt.data.local_id = local_id;
        int len = sizeof(cvt.binary) + 2 * sizeof(uint32_t);
        char send_buf[len * 5];
        EncodeMsgToBuffer(MESSAGE_TYPE::IP_ID, cvt, send_buf);
        if (sendto(udp_send_ip_fd_ptr_, send_buf, len, 0, (struct sockaddr *) &addr_udp_send_ip_,
                   sizeof(addr_udp_send_ip_)) <= 0) {
            ROS_ERROR("UDP BROADCAST ERROR !!!");
        }
//     print("Broadcast local Ip and Id.\n");
    }

    /*
     * In this function, we will encode a ros_msg type and its type id to a
     * serialzed msg(uint32_t), and the msg type id should be in the first
     * bite.
     * */
    template<typename union_msg>
    int EncodeMsgToBuffer(const MESSAGE_TYPE msg_type_id, union_msg &msg, char *send_buf_) {
        uint32_t msg_size = sizeof(msg.binary);
        int len = msg_size + 2 * sizeof(uint32_t);
        auto ptr = (uint8_t *) (send_buf_);
        *((MESSAGE_TYPE *) ptr) = msg_type_id;
        ptr += sizeof(MESSAGE_TYPE);
        *((uint32_t *) ptr) = msg_size;
        ptr += sizeof(uint32_t);
        memcpy(ptr, msg.binary, msg_size);
        return len;
    }

    /*
        * In this function, we will decode a ros_msg
        * */
    template<typename union_msg>
    int DecodeMsgFromBuffer(union_msg &msg) {
        auto ptr = (uint8_t *) (udp_recv_buf_ + sizeof(uint32_t));
        uint32_t msg_size = *((uint32_t *) ptr);
        ptr += sizeof(uint32_t);
        memcpy(msg.binary, ptr, msg_size);
        return msg_size + sizeof(uint32_t) * 2;
    }

    /*
       * In this function, we will decode a ros_msg
       * */
    template<typename union_msg>
    int DecodeMsgFromBuffer(union_msg &msg, const char *recv_buf) {
        auto ptr = (uint8_t *) (recv_buf + sizeof(uint32_t));
        uint32_t msg_size = *((uint32_t *) ptr);
        ptr += sizeof(uint32_t);
        memcpy(msg.binary, ptr, msg_size);
        return msg_size + sizeof(uint32_t) * 2;
    }

    template <typename ros_msg>
    int DeserializeMsgFromBuffer(ros_msg &msg)
    {
        auto ptr = (uint8_t *)(udp_recv_buf_ + sizeof(MESSAGE_TYPE));
        uint32_t msg_size = *((uint32_t *)ptr);
        ptr += sizeof(uint32_t);
        namespace ser = ros::serialization;
        ser::IStream stream(ptr, msg_size);
        ser::deserialize(stream, msg);
        return msg_size + 2 * sizeof(uint32_t);
    }

    template <typename ros_msg>
    int SerializeMsgToBuffer(const MESSAGE_TYPE msg_type, const ros_msg &msg, char *send_buf_)
    {
        auto ptr = (uint8_t *)(send_buf_);
        *((MESSAGE_TYPE*)ptr) = msg_type;
        ptr += sizeof(MESSAGE_TYPE);
        namespace ser = ros::serialization;
        uint32_t msg_size = ser::serializationLength(msg);
        *((uint32_t *)ptr) = msg_size;
        ptr += sizeof(uint32_t);
        ser::OStream stream(ptr, msg_size);
        ser::serialize(stream, msg);
        return msg_size + 2 * sizeof(uint32_t);
    }

    int InitUdpUnicast(Teammate &drone, const int &port) {
        string ip_s = drone.ip_;
        const char *ip = ip_s.c_str();
        int fd;

        if ((fd = socket(AF_INET, SOCK_DGRAM, 0)) <= 0) {
            ROS_ERROR("[udo_bridge] Socket sender creation error!");
            exit(EXIT_FAILURE);
        }


        drone.addr_udp_send_.sin_family = AF_INET;
        drone.addr_udp_send_.sin_port = htons(port);

        if (inet_pton(AF_INET, ip, &drone.addr_udp_send_.sin_addr) <= 0) {
            printf("\nInvalid address/ Address not supported \n");
            return -1;
        }
        return fd;
    }

    int InitUdpBoardcast(const int port) {
        int fd;

        if ((fd = socket(AF_INET, SOCK_DGRAM, 0)) <= 0) {
            ROS_ERROR("[udo_bridge] Socket sender creation error!");
            exit(EXIT_FAILURE);
        }

        int so_broadcast = 1;
        if (setsockopt(fd, SOL_SOCKET, SO_BROADCAST, &so_broadcast, sizeof(so_broadcast)) < 0) {
            cout << "Error in setting Broadcast option";
            exit(EXIT_FAILURE);
        }

        addr_udp_send_ip_.sin_family = AF_INET;
        addr_udp_send_ip_.sin_port = htons(port);

        if (inet_pton(AF_INET, broadcast_ip.c_str(), &addr_udp_send_ip_.sin_addr) <= 0) {
            printf("\nInvalid address/ Address not supported \n");
            return -1;
        }

        return fd;
    }

    int BindToUdpPort(const int port, int &server_fd) {
        struct sockaddr_in address;
        int opt = 1;

        // Creating socket file descriptor
        if ((server_fd = socket(AF_INET, SOCK_DGRAM, 0)) == 0) {
            perror("socket failed");
            exit(EXIT_FAILURE);
        }

        // Forcefully attaching socket to the port
        if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT,
                       &opt, sizeof(opt))) {
            perror("setsockopt");
            exit(EXIT_FAILURE);
        }
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = INADDR_ANY;
        address.sin_port = htons(port);

        // Forcefully attaching socket to the port
        if (bind(server_fd, (struct sockaddr *) &address,
                 sizeof(address)) < 0) {
            perror("bind failed");
            exit(EXIT_FAILURE);
        }

        return server_fd;
    }

    void CharIp2StringIp(uint8_t *ch_ip, string &str_ip) {
        str_ip.clear();
        str_ip += to_string(ch_ip[0]);
        str_ip.push_back('.');
        str_ip += to_string(ch_ip[1]);
        str_ip.push_back('.');
        str_ip += to_string(ch_ip[2]);
        str_ip.push_back('.');
        str_ip += to_string(ch_ip[3]);
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

    int get_local_ip(char *ip) {
        struct ifaddrs *ifAddrStruct;
        void *tmpAddrPtr=NULL;
        getifaddrs(&ifAddrStruct);
        while (ifAddrStruct != NULL) {
            if (ifAddrStruct->ifa_addr->sa_family==AF_INET) {
                tmpAddrPtr = &((struct sockaddr_in *) ifAddrStruct->ifa_addr)->sin_addr;
                inet_ntop(AF_INET, tmpAddrPtr, ip, INET_ADDRSTRLEN);
                if((ip[0] =='1' && ip[1] == '0' && ip[3] == '0') || (ip[8] =='2' && ip[9] == '3' && ip[10] == '4' && ip[12] == '1')){
                    printf("%s IP Address: %s\n", ifAddrStruct->ifa_name, ip);
                    // freeifaddrs(ifAddrStruct);
                    return 0;
                }
            }
            ifAddrStruct=ifAddrStruct->ifa_next;
        }
        //free ifaddrs
        freeifaddrs(ifAddrStruct);
        return 0;
    }


    void MincoTrajMsgCallback(const swarm_msgs::MincoTraj::ConstPtr &msg)
    {
        uint32_t msg_size = ros::serialization::serializationLength(*msg);
        int len = msg_size + 2 * sizeof(uint32_t);
        char send_buf[len * 2];
        
        SerializeMsgToBuffer(MESSAGE_TYPE::MINCO_TRAJ, *msg, send_buf);
        for (auto iter = teammates.begin(); iter != teammates.end(); iter++) {
            auto &drone = iter->second;
            if (sendto(drone.udp_send_fd_ptr_, send_buf, len, 0, (struct sockaddr *) &drone.addr_udp_send_,
                       sizeof(drone.addr_udp_send_)) <= 0) {
                ROS_ERROR_STREAM("MINCOTRAJ SEND ERROR : " << strerror(errno));
            } 
        }
    }

    void TargetObsrvCallback(const swarm_msgs::TargetObsrv::ConstPtr &msg)
    {
        if(tmt_id_num_ == 0) return;

        uint32_t msg_size = ros::serialization::serializationLength(*msg);
        int len = msg_size + 2 * sizeof(uint32_t);
        char send_buf[len * 2];
        
        SerializeMsgToBuffer(MESSAGE_TYPE::TARGET_OBSRV, *msg, send_buf); 

        for(int &id : swarm_id_list_)
        {
            auto teammate_iter = teammates.find(id);
            if(teammate_iter != teammates.end())
            {
                auto &drone = teammate_iter->second;
                if (sendto(drone.udp_send_fd_ptr_, send_buf, len, 0, (struct sockaddr *) &drone.addr_udp_send_,
                        sizeof(drone.addr_udp_send_)) <= 0) {
                    ROS_ERROR_STREAM("TARGET OBSRV SEND ERROR : " << strerror(errno));
                } 
            }
        }

    }

    void ChunkStampMsgCallback(const swarm_msgs::ChunkStamps::ConstPtr &msg)
    {
        uint32_t msg_size = ros::serialization::serializationLength(*msg);
        int len = msg_size + 2 * sizeof(uint32_t);
        char send_buf[len * 2];
        
        SerializeMsgToBuffer(MESSAGE_TYPE::CHUNK_STAMP, *msg, send_buf);
        for (auto iter = teammates.begin(); iter != teammates.end(); iter++) {
            auto &drone = iter->second;

            if (sendto(drone.udp_send_fd_ptr_, send_buf, len, 0, (struct sockaddr *) &drone.addr_udp_send_,
                       sizeof(drone.addr_udp_send_)) <= 0) {
                ROS_ERROR_STREAM("CHUNKSTAMP SEND ERROR : " << strerror(errno));
            } 
        }
    }


    void MapLocalMsgCallback(const swarm_msgs::LocalMapList::ConstPtr &msg)
    {

        auto teammate_iter = teammates.find(msg->to_drone_id);
        if(teammate_iter != teammates.end())
        {
            uint32_t msg_size = ros::serialization::serializationLength(*msg);
            int len = msg_size + 2 * sizeof(uint32_t);
            char send_buf[len * 2];
            SerializeMsgToBuffer(MESSAGE_TYPE::MAP_LOCAL, *msg, send_buf);
            auto &drone = teammate_iter->second;

            if (sendto(drone.udp_send_fd_ptr_, send_buf, len, 0, (struct sockaddr *) &drone.addr_udp_send_,
                       sizeof(drone.addr_udp_send_)) <= 0) {
                ROS_ERROR_STREAM("LOCALMAP SEND ERROR : " << strerror(errno));
            } 
        }else{
            ROS_ERROR_STREAM("LOCALMAP SEND ERROR: No teammateInfo for : " << msg->to_drone_id);
            return;
        }
    }



};

int main(int argc, char **argv) {
    ros::init(argc, argv, "udp_st_offset");
    ros::NodeHandle nh("~");

    UdpBridge brg(nh);

    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}
