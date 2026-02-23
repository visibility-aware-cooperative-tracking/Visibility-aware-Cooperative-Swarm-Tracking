#pragma once

#include <rog_map/inf_map.h>
#include <rog_map/free_cnt_map.h>
#include <queue>
#include <rog_utils/raycaster.h>
#include <mutex>


//#define USE_UNKNOWN_FLAG
namespace rog_map {
    class MultiMapManager;
    using namespace geometry_utils;
    using std::cout;
    using std::endl;

    class ProbMap : public SlidingMap {
    public:
        // Query result
        GridType getGridType(Vec3i &id_g) const;

        bool isOccupied(const Vec3f &pos) const;

        bool isOccupied(const Vec3i &id_g) const;

        bool isOccupiedOwn(const Vec3i &id_g) const;

        bool isOccupiedTmt(const Vec3i &id_g) const;

        bool isOccupiedInflate(const Vec3f &pos) const;

        bool isOccupiedInflate(const Vec3i &id_g) const;

        bool isOccupiedInflateWithNbr(const Vec3f &pos) const;

        bool isOccupiedInflateWithNbr(const Vec3i &id_g) const;

        bool isOccupiedInflateWithNbr(const Vec3f &pos, Vec3f& res_nbr_pos) const;

        bool isOccupiedInflateWithNbr(const Vec3i &id_g, Vec3f& res_nbr_pos) const;

        bool isFree(const Vec3f &pos) const;

        bool isFreeInflate(const Vec3f &pos) const;

        bool isUnknown(const Vec3f &pos) const;

        bool isUnknownInflate(const Vec3f &pos) const;

        bool isFrontier(const Vec3f &pos) const;

        bool isFrontier(const Vec3i &id_g) const;

        GridType getGridType(const Vec3f &pos) const;

        GridType getInfGridType(const Vec3f &pos) const;

        double getMapValue(const Vec3f &pos) const;

        void boxSearch(const Vec3f &_box_min, const Vec3f &_box_max,
                       const GridType &gt, type_utils::vec_E<Vec3f> &out_points, 
                       const int stride = 1,
                       int ownership = OwnerType::UND) const;

        void boxSearchInflate(const Vec3f &box_min, const Vec3f &box_max,
                              const GridType &gt, type_utils::vec_E<Vec3f> &out_points) const;

        void boundBoxByLocalMap(Vec3f &box_min, Vec3f &box_max) const;

        friend MultiMapManager;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    protected:
        ROGMapConfig cfg_;
        InfMap::Ptr inf_map_;
        FreeCntMap::Ptr fcnt_map_;
        /// Spherical neighborhood lookup table
        std::vector<float> occupancy_buffer_;
        std::vector<uint8_t> owner_buffer_;
        std::vector<double> stamp_buffer_;
        std::shared_ptr<MultiMapManager> mm_;
        std::mutex update_mutex_;

        bool map_empty_{true};
        struct RaycastData {
            geometry_utils::raycaster::RayCaster raycaster;
            std::queue<Vec3i> update_cache_id_g;
            std::vector<uint16_t> operation_cnt;
            std::vector<uint16_t> hit_cnt;
            Vec3f cache_box_max, cache_box_min, local_update_box_max, local_update_box_min;
            int batch_update_counter{0};

            bool have_target_odom{false}; 
            Vec3f target_odom;
        } raycast_data_;

        vector<double> time_consuming_;
        vector <string> time_consuming_name_{"Total", "Raycast", "Update_cache", "Inflation", "PointCloudNumber",
                                             "CacheNumber", "InflationNumber"};


    public:
        typedef std::shared_ptr<ProbMap> Ptr;

        ProbMap(ROGMapConfig &cfg);

        ~ProbMap() = default;

        void updateOccPointCloud(const PointCloud &input_cloud);

        void writeTimeConsumingToLog(std::ofstream &log_file);

        void writeMapInfoToLog(std::ofstream &log_file);

        void updateProbMap(const PointCloud &cloud, const Pose &pose);

    protected:
        // warning using this function will cause memory leak if the id_g is not in the map
        type_utils::Vec3f lineBoxIntersectPoint(const Vec3f &pt, const Vec3f &pos,
                                                        const Vec3f &box_min, const Vec3f &box_max)
                                                        {
                                                            Eigen::Vector3d diff = pt - pos;
            Eigen::Vector3d max_tc = box_max - pos;
            Eigen::Vector3d min_tc = box_min - pos;

            double min_t = 1000000;

            for (int i = 0; i < 3; ++i) {
                if (fabs(diff[i]) > 0) {

                    double t1 = max_tc[i] / diff[i];
                    if (t1 > 0 && t1 < min_t)
                        min_t = t1;

                    double t2 = min_tc[i] / diff[i];
                    if (t2 > 0 && t2 < min_t)
                        min_t = t2;
                }
            }

            return pos + (min_t - 1e-3) * diff;
                                                        }

        bool isUnknown(const Vec3i &id_g) const;

        bool isFree(const Vec3i &id_g) const;

        //====================================================================

        void clearMemoryOutOfMap(const vector<int> &clear_id, const int &i) override;

        void probabilisticMapFromCache();

        void sharedPointUpdate(const Vec3i &id_g, const int &hash_id, const int &shared_type, const double &pack_stamp);

        void hitPointUpdate(const Vec3f &pos, const int &hash_id, const int &hit_num);

        void missPointUpdate(const Vec3f &pos, const int &hash_id, const int &hit_num);

        void raycastProcess(const PointCloud &input_cloud, const Vec3f &cur_odom);

        void insertUpdateCandidate(const Vec3i &id_g, bool is_hit);

        void updateLocalBox(const Vec3f &cur_odom);

        void resetLocalMap() override;

        void setTargetOdom(double x, double y, double z);
            
    };
}