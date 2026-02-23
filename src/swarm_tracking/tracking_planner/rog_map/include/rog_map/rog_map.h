#pragma once

#include <ros/ros.h>
#include "boost/thread.hpp"
#include <rog_map/prob_map.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_broadcaster.h>
#include <ikd_tree/ikd_Tree.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ciri_utils/geometry_utils.h>
#include <cmath>
#include <rog_map/scope_timer.h>


namespace rog_map {
    using namespace type_utils;
    using namespace geometry_utils;
    using namespace std;
    class MultiMapManager;

    typedef pcl::PointXYZI PointType;
    typedef pcl::PointCloud<PointType> PointCloudXYZIN;


    class ROGMap : public ProbMap {
    public:
        bool isLineFree(const Vec3f &start_pt, const Vec3f &end_pt, const double &max_dis = 999999,
                        const vec_Vec3i &neighbor_list = vec_Vec3i{}) const;

        bool
        isLineFree(const Vec3f &start_pt, const Vec3f &end_pt, Vec3f &free_local_goal, const double &max_dis = 999999,
                   const vec_Vec3i &neighbor_list = vec_Vec3i{}) const;
        
        bool isLineFreeInflate(const Vec3f &start_pt,
                               const Vec3f &end_pt,
                               const double max_dis = 9999);

        void updateRobotState(const Pose &pose);

        void updateMap(const PointCloud &cloud, const Pose &pose);

        RobotState getRobotState() const;

        Vec3f getRobotPos() const;

        double getCeilHeight() const;

        double getGroundHeight() const;

        friend MultiMapManager;
    private:
        ros::NodeHandle nh_;
        RobotState robot_state_;

        struct ROSCallback {
            ros::Subscriber odom_sub, cloud_sub;
            ros::Subscriber target_sub;
            int unfinished_frame_cnt{0};
            Pose pc_pose;
            PointCloud pc;
            ros::Timer update_timer;
            mutex update_lock;
        } rc_;

        struct VisualizeMap {
            ros::Publisher occ_pub, unknown_pub,
                    occ_inf_pub, unknown_inf_pub,
                    mkr_arr_pub, kd_tree_pub, frontier_pub,
                    occ_own_pub, occ_tmt_pub,
                    downsample_occ_pub;
            ros::Timer viz_timer;
            ros::Subscriber plan_started_sub;
            bool plan_started{false};
        } vm_;

        std::ofstream time_log_file_, map_info_log_file_;

        KD_TREE<PointType> ikdtree;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        typedef shared_ptr<ROGMap> Ptr;

        ROGMap(const ros::NodeHandle &nh, ROGMapConfig &cfg);

        ~ROGMap() = default;

    
    public:
        double mp_cnt{0}, mp_t{0};

        double getLastMappingAveTime() {
            if (mp_cnt == 0) {
                return -1;
            }
            double ave_t = mp_t / mp_cnt;
            mp_cnt = 0;
            mp_t = 0;
            return ave_t;
        }

        void NearestNeighborSearch(const Vec3f &pt_in,
                                   Vec3f &pt_out,
                                   double &dis,
                                   const double &max_dis = 10.0);


        const ROGMapConfig &getCfg() const {
            return cfg_;
        }

        // For ros call back
    private:
        void odomCallback(const nav_msgs::OdometryConstPtr &odom_msg);

        void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);

        void targetOdomCallback(const nav_msgs::OdometryConstPtr &target_odom_msg);

        void updateCallback(const ros::TimerEvent &event);

        static void vecEVec3fToPC2(const type_utils::vec_E<Vec3f> &points, sensor_msgs::PointCloud2 &cloud);

        void vizCallback(const ros::TimerEvent &event);

        void vizCallbackPreload(const ros::TimerEvent &event);

    };
}
