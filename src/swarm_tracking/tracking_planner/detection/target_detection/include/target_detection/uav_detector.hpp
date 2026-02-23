#ifndef UAV_DETECTOR_HPP
#define UAV_DETECTOR_HPP

#include <Eigen/Core>
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/search/kdtree.h>
#include <pcl/impl/instantiate.hpp>
#include <pcl/segmentation/extract_clusters.h>
#include <nav_msgs/Odometry.h>
#include <target_detection/esikf_tracker.hpp>
#include <swarm_msgs/TargetObsrv.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <mutex>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <ifaddrs.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>
#include <visualization_msgs/Marker.h>
#include <target_detection/FEC.h>
#include <traj_utils/scope_timer.hpp>

#define DEBUG_FILE_DIR(name) (string(string(ROOT_DIR) + "log/"+name))

typedef pcl::PointCloud<pcl::PointXYZI> PointCloudXYZI;

using namespace Eigen;

struct Cluster
{
    Cluster(const Vector3d& pos, const bool &is_high_inten, const double &max_dist) {
        this->pos_in_world = pos;
        this->is_high_intensity = is_high_inten;
        this->max_dist = max_dist;
    };

    Cluster(const Cluster &b) {
        this->pos_in_world = b.pos_in_world;
        this->is_high_intensity = b.is_high_intensity;
        this->max_dist = b.max_dist;
    };

    Vector3d pos_in_world;
    bool is_high_intensity;
    double max_dist;
};

struct TeamObsrv
{   
    void set(const swarm_msgs::TargetObsrvPtr& obsrv_msg)
    {
        this->drone_id = obsrv_msg->drone_id;
        this->update_time = obsrv_msg->header.stamp.toSec();
        this->target_obsrv << obsrv_msg->target_pos[0],
                              obsrv_msg->target_pos[1],
                              obsrv_msg->target_pos[2];
    };

    double update_time;
    int drone_id;
    Vector3d target_obsrv;
};

struct OdomData
{   
    OdomData(const nav_msgs::OdometryConstPtr& odom_msg){
        this->pos(0) = odom_msg->pose.pose.position.x;
        this->pos(1) = odom_msg->pose.pose.position.y;
        this->pos(2) = odom_msg->pose.pose.position.z;
        this->quat.w() = odom_msg->pose.pose.orientation.w;
        this->quat.x() = odom_msg->pose.pose.orientation.x;
        this->quat.y() = odom_msg->pose.pose.orientation.y;
        this->quat.z() = odom_msg->pose.pose.orientation.z;
        this->rot = this->quat.toRotationMatrix(); 
        this->stamp = odom_msg->header.stamp.toSec();
    };

    OdomData() {};
    OdomData(const OdomData &odom_){
        this->pos = odom_.pos;
        this->quat = odom_.quat;
        this->rot = odom_.rot;
        this->stamp = odom_.stamp;
    };

    Quaterniond quat;
    Matrix3d rot;
    Vector3d pos;
    double stamp;
};

typedef std::vector<Cluster> ClusterSet;
typedef std::vector<TeamObsrv> TeamObsrvSet;

class uavDetector{

private:
    
    //Utils 
    ClusterSet cluster_set_;
    TeamObsrvSet team_obsrvs_;
    ESIKF ekf_tracker_;
    MeasData meas_data_;
    PointCloudXYZI::Ptr pcl_accumulate_;
    ros::Publisher accumu_pcl_pub_, cluster_pub_, tracker_pub_, refined_pcl_pub_;
    std::mutex mtx_;
    std::ofstream detector_log_;


    //depth map
    std::vector<uint8_t> dpt_buffer_;
    double resolution_;
    double resolution_inv_;
    Eigen::Vector2d dpt_origin_;
    int dpt_grid_size_;
    Eigen::Vector2d dpt_angle_size_;
    Eigen::Vector2i dpt_grid_num_;

    
    bool use_dpt_refine_{false};
    bool only_use_high_intensity_{false};
    
    //Params
    int drone_id_;
    int pcl_frame_num_;
    int ekf_hz_;
    double valid_cluster_dist_thresh_; 
    double valid_cluster_size_thresh_; 
    double valid_target_odom_dist_thresh_;
    int min_cluster_size_;
    int min_high_inten_cluster_size_;
    double predict_region_radius_;
    double same_obj_thresh_;
    int intensity_thresh_;
    double team_meas_noise_;
    double self_meas_noise_;

    //Flags & stamps
    bool has_valid_tracker_;
    bool has_trigger_;
    bool exist_self_meas_;
    double batch_last_stamp_;
    int scan_count_;
    int ekf_count_;
    int pred_update_rate_;

    bool has_target_odom_{false};


    //Odoms & Meas
    OdomData odom_, odom_recv_;
    OdomData target_odom_;
    Vector3d meas_of_tracker_;//defined in world frame

    int pos2addr(const Eigen::Vector3d &tgt_pos, const Eigen::Vector3d &ctr_pos);

    void pos2sph(const Eigen::Vector3d &xyz, Eigen::Vector2d &tp);

    void sph2idx(const Eigen::Vector2d &sph, Eigen::Vector2i &idx);

    double aCos(const double val);
    
    void ClusterExtractPredictRegion(const PointCloudXYZI::Ptr cur_pcl);
    
    void ClusterExtractHighIntensity(const PointCloudXYZI::Ptr cur_pcl);
    
    void initTrackerBySelfMeas(const double& meas_stamp);

    void initTrackerByTeamMeas(const double& meas_stamp, const Eigen::Vector3d &target_pos);

    bool CheckClusterValidation(const double& meas_stamp, Vector3d& observation);

    void visualizeAccumuPC(const PointCloudXYZI::Ptr& accumu_pcl);

    void visualizeCluster(const double &lidar_end_time, const Eigen::Vector3d &pos, const int &cluster_index, bool is_high_inten);
    
    void visualizeTracker(const double &lidar_end_time);

    void visualizeDeleteAllCluster(const double &time);
public:

    uavDetector(){};
    ~uavDetector(){ detector_log_.close(); };

    void init(ros::NodeHandle& nh);
    
    bool runEKF(ros::Time& time_T, 
                Eigen::Vector3d& target_pos,
                Eigen::Vector3d& target_vel);

    bool setSelfMeas(const sensor_msgs::PointCloud2ConstPtr& pcl_msg,
                     Vector3d& target_observation);
    
    void setTeamMeas(const swarm_msgs::TargetObsrvPtr& obsrv_msg);

    void setOdom(const nav_msgs::OdometryConstPtr& odom_msg);

    void setTargetOdom(const nav_msgs::OdometryConstPtr& odom_msg);

    void setTrigger();

    /* get ID from IP address */
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
                    // freeifaddrs(ifAddrStruct);
                    return true;
                }
            }
            ifAddrStruct=ifAddrStruct->ifa_next;
        }
        //free ifaddrs
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

    //Helpers
    inline int getId() { return drone_id_; }

    inline int getHz() { return ekf_hz_; }
};

#endif