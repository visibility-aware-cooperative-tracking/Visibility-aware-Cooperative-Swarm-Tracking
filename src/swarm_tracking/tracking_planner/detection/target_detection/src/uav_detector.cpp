#include <target_detection/uav_detector.hpp>

double uavDetector::aCos(const double val)
{
  if(val <= -1.0){
    return M_PI;
  }else if(val >= 1.0){
    return 0;
  }else{
    return acos(val);
  }
}

void uavDetector::sph2idx(const Eigen::Vector2d &sph, Eigen::Vector2i &idx)
{
    idx = ((sph - dpt_origin_) * resolution_inv_).array().floor().cast<int>();
}

void uavDetector::pos2sph(const Eigen::Vector3d &xyz, Eigen::Vector2d &tp)
{
    double r_dist = xyz.norm();
    tp(0) = aCos(xyz(2) / r_dist);
    double at = atan2(xyz(1), xyz(0));
    tp(1) = at < 0 ? at + 2 * M_PI : at;
}

int uavDetector::pos2addr(const Eigen::Vector3d &tgt_pos, const Eigen::Vector3d &ctr_pos)
{
    Eigen::Vector2d sph;
    Eigen::Vector2i idx;
    pos2sph(tgt_pos - ctr_pos, sph);
    sph2idx(sph, idx);
    return idx(1) * dpt_grid_num_(0) + idx(0);
}

void uavDetector::ClusterExtractPredictRegion(const PointCloudXYZI::Ptr cur_pcl)
{
    cluster_set_.clear();
    if(!has_valid_tracker_) return;
    if(!has_target_odom_) return;
    
    pcl::PointCloud<pcl::PointXYZ> cluster_input_cloud;
    cluster_input_cloud.clear();
    cluster_input_cloud.reserve(cur_pcl->size());

    visualizeTracker(ros::Time::now().toSec());

    for (auto it_pcl = cur_pcl->points.begin(); it_pcl != cur_pcl->points.end(); it_pcl++) {
        Vector3d cur_point(it_pcl->x, it_pcl->y, it_pcl->z);
        if ((cur_point - target_odom_.pos).norm() < predict_region_radius_) {
            pcl::PointXYZ pt_add;
            pt_add.x = it_pcl->x;
            pt_add.y = it_pcl->y;
            pt_add.z = it_pcl->z;

            cluster_input_cloud.push_back(pt_add);
        }    
    }

    if (cluster_input_cloud.empty()) 
        return;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud(
            new pcl::PointCloud<pcl::PointXYZ>(cluster_input_cloud.size(), 1));
    *cluster_cloud = cluster_input_cloud;

    std::vector<pcl::PointIndices> clusters;
    clusters.clear();
    clusters = FEC::FastEuclideanClustering(cluster_cloud, min_cluster_size_, 0.3, 500);


    
    for (auto iter = clusters.begin(); iter != clusters.end(); iter++) {

        if(use_dpt_refine_)
        {
            std::vector<Eigen::Vector3d> pt_in_cluster; pt_in_cluster.clear();
            int cluster_pt_size = iter->indices.size();
            for (auto index = iter->indices.begin(); index != iter->indices.end(); index++) {
                pt_in_cluster.emplace_back(cluster_cloud->points[*index].x, cluster_cloud->points[*index].y, cluster_cloud->points[*index].z);
            }

            Eigen::Vector3d ego_pos = odom_.pos;
            auto compare_distance = [&ego_pos](const Eigen::Vector3d& p1, const Eigen::Vector3d& p2) {
                double dist1 = (p1 - ego_pos).norm();
                double dist2 = (p2 - ego_pos).norm();
                return dist1 < dist2;
            };

            std::sort(pt_in_cluster.begin(), pt_in_cluster.end(), compare_distance);


            int cnt = 0;
            Eigen::Vector3d cluster_pos(0,0,0);
            Eigen::Vector3d max_value(INT_MIN, INT_MIN, INT_MIN);
            Eigen::Vector3d min_value(INT_MAX, INT_MAX, INT_MAX);

            for(auto &pt : pt_in_cluster)
            {
                if(dpt_buffer_[pos2addr(pt, ego_pos)] == 0)
                {
                    cnt++;
                    dpt_buffer_[pos2addr(pt, ego_pos)] = 1;
                
                    cluster_pos += (pt - cluster_pos) / cnt;
                    if (pt.x() > max_value.x()) max_value.x() = pt.x();
                    if (pt.y() > max_value.y()) max_value.y() = pt.y();
                    if (pt.z() > max_value.z()) max_value.z() = pt.z();
                    if (pt.x() < min_value.x()) min_value.x() = pt.x();
                    if (pt.y() < min_value.y()) min_value.y() = pt.y();
                    if (pt.z() < min_value.z()) min_value.z() = pt.z();

                } 
                
            }
            double max_dist = (max_value - min_value).norm();
            cluster_set_.push_back(Cluster(cluster_pos, false, max_dist));
            
            for(auto &pt : pt_in_cluster)     
                dpt_buffer_[pos2addr(pt, ego_pos)] = 0; 

        }else{
            
            int cnt = 0;
            Vector3d cluster_pos(0,0,0);
            Vector3d max_value(INT_MIN, INT_MIN, INT_MIN);
            Vector3d min_value(INT_MAX, INT_MAX, INT_MAX);
            for (auto index = iter->indices.begin(); index != iter->indices.end(); index++) {
                cnt++;
                Vector3d point_cur(cluster_cloud->points[*index].x, 
                                   cluster_cloud->points[*index].y,
                                   cluster_cloud->points[*index].z);
                //Position of the cluster center
                cluster_pos += (point_cur - cluster_pos) / cnt;
                //Compute max and min xyz value
                if (point_cur.x() > max_value.x()) max_value.x() = point_cur.x();
                if (point_cur.y() > max_value.y()) max_value.y() = point_cur.y();
                if (point_cur.z() > max_value.z()) max_value.z() = point_cur.z();
                if (point_cur.x() < min_value.x()) min_value.x() = point_cur.x();
                if (point_cur.y() < min_value.y()) min_value.y() = point_cur.y();
                if (point_cur.z() < min_value.z()) min_value.z() = point_cur.z();
                
            }
            double max_dist = (max_value - min_value).norm();
            cluster_set_.push_back(Cluster(cluster_pos, false, max_dist));
        }   

    }
 
}

void uavDetector::ClusterExtractHighIntensity(const PointCloudXYZI::Ptr cur_pcl)
{
    if(!has_target_odom_) return;
    if( (odom_.pos - target_odom_.pos).norm() > valid_target_odom_dist_thresh_ ) return;

    pcl::PointCloud<pcl::PointXYZ> cluster_input_cloud;
    cluster_input_cloud.clear();
    cluster_input_cloud.reserve(cur_pcl->size());

    if(has_valid_tracker_) 
    {
        visualizeTracker(ros::Time::now().toSec());
    }
    for (auto it_pcl = cur_pcl->points.begin(); it_pcl != cur_pcl->points.end(); it_pcl++) {

        Vector3d cur_point(it_pcl->x, it_pcl->y, it_pcl->z);

        bool select_bar = 
             it_pcl->intensity > intensity_thresh_
             && 
             (cur_point - target_odom_.pos).norm() < predict_region_radius_;
      
        if (select_bar) {
            pcl::PointXYZ pt_add;
            pt_add.x = it_pcl->x;
            pt_add.y = it_pcl->y;
            pt_add.z = it_pcl->z;
        
            cluster_input_cloud.push_back(pt_add);
        }
    }

    if (cluster_input_cloud.empty())
    {
        return;
    }else{
    }
        
    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud(
    new pcl::PointCloud<pcl::PointXYZ>(cluster_input_cloud.size(), 1));
    *cluster_cloud = cluster_input_cloud;

    std::vector<pcl::PointIndices> clusters;
    clusters.clear();
    clusters = FEC::FastEuclideanClustering(cluster_cloud, min_high_inten_cluster_size_, 0.3, 1000);

   
    for (auto iter = clusters.begin(); iter != clusters.end(); iter++) {
        
        int cnt = 0;
        Vector3d cluster_pos(0,0,0);
        Vector3d max_value(INT_MIN, INT_MIN, INT_MIN);
        Vector3d min_value(INT_MAX, INT_MAX, INT_MAX);
        for (auto index = iter->indices.begin(); index != iter->indices.end(); index++) {
            cnt++;
            Vector3d point_cur(cluster_cloud->points[*index].x, cluster_cloud->points[*index].y,
                                cluster_cloud->points[*index].z);
            //Position of the cluster center
            cluster_pos += (point_cur - cluster_pos) / cnt;
            //Compute max and min xyz value
            if (point_cur.x() > max_value.x()) max_value.x() = point_cur.x();
            if (point_cur.y() > max_value.y()) max_value.y() = point_cur.y();
            if (point_cur.z() > max_value.z()) max_value.z() = point_cur.z();
            if (point_cur.x() < min_value.x()) min_value.x() = point_cur.x();
            if (point_cur.y() < min_value.y()) min_value.y() = point_cur.y();
            if (point_cur.z() < min_value.z()) min_value.z() = point_cur.z();
        }
        double max_cluster_dist = (max_value - min_value).norm();

        if(!has_valid_tracker_) 
        {
            cluster_set_.push_back(Cluster(cluster_pos, true, max_cluster_dist));
            continue;
        }
        
        bool same_object = false; 
        //to update the predicted cluster with high-intensity measure
        for (int i = 0; i < cluster_set_.size(); ++i) {
            Vector3d dist_vec = cluster_set_[i].pos_in_world - cluster_pos;
            if (dist_vec.norm() < same_obj_thresh_) {
                cluster_set_[i].is_high_intensity = true; 

                if(max_cluster_dist > cluster_set_[i].max_dist) 
                {
                    cluster_set_[i].pos_in_world = cluster_pos;
                    cluster_set_[i].max_dist = max_cluster_dist;
                }
                same_object = true;
            }
        }

        if(!same_object)
            cluster_set_.push_back(Cluster(cluster_pos, true, max_cluster_dist));
        
    }
}


void uavDetector::initTrackerBySelfMeas(const double& meas_stamp)
{
   
    //select the closest cluster
    double min_dist{DBL_MAX};
    Vector3d min_pos_in_world;
    for (int i = 0; i < cluster_set_.size(); i++) {
        Vector3d pos_in_world = cluster_set_[i].pos_in_world;

        double cluster_odom_dist = (pos_in_world - odom_.pos).norm();

        if (cluster_set_[i].is_high_intensity && 
            cluster_set_[i].max_dist < valid_cluster_size_thresh_ &&
            cluster_odom_dist < valid_target_odom_dist_thresh_ && cluster_odom_dist < min_dist) {
            min_dist = cluster_odom_dist;
            min_pos_in_world = pos_in_world;
        }
    }

    if(min_dist >= valid_target_odom_dist_thresh_)  
    { return; }

    ekf_tracker_.setDim(6,3,3);
    ekf_tracker_.init(min_pos_in_world, meas_stamp);
    has_valid_tracker_ = true;
}

void uavDetector::initTrackerByTeamMeas(const double& meas_stamp, const Eigen::Vector3d &target_pos)
{
    if(has_valid_tracker_) return;
    if(!has_target_odom_) return;
    
    if( (target_pos - odom_.pos).norm() > valid_target_odom_dist_thresh_ ) return;

    //set Tracker
    ekf_tracker_.setDim(6,3,3);
    ekf_tracker_.init(target_pos, meas_stamp);
    has_valid_tracker_ = true;
}


bool uavDetector::CheckClusterValidation(const double& meas_stamp, Vector3d& observation)
{   
    //visualize clusters
    int cluster_idx{0};
    for(auto cluster : cluster_set_)
        visualizeCluster(ros::Time::now().toSec(), cluster.pos_in_world, cluster_idx++, cluster.is_high_intensity);

    if(!has_valid_tracker_) 
    { return false; }

    if(!has_target_odom_) return false;

    exist_self_meas_ = false;
    Vector3d self_meas(0,0,0);
   
    double min_dist{DBL_MAX};
    
    //Check if valid self meas exist
    for (int j = 0; j < cluster_set_.size(); j++) {
        if (cluster_set_[j].max_dist >= valid_cluster_size_thresh_)
        {
            continue;
        }
            
        Vector3d resi_vec = cluster_set_[j].pos_in_world - target_odom_.pos;
        if (resi_vec.norm() < valid_cluster_dist_thresh_ && resi_vec.norm() < min_dist) {
            min_dist = resi_vec.norm();
            exist_self_meas_ = true;
            self_meas = cluster_set_[j].pos_in_world;
            cluster_set_.erase(cluster_set_.begin() + j); 
            j--;
        }
    }

    if(exist_self_meas_){
        observation = self_meas; 
        std::lock_guard<std::mutex> lock(mtx_);
        meas_data_.emplace_back(self_meas, meas_stamp, self_meas_noise_);
    }
    return exist_self_meas_;
}


void uavDetector::init(ros::NodeHandle& nh){
    nh.param("detector/drone_id", drone_id_, -1);
    nh.param("detector/pcl_frame_num", pcl_frame_num_, 1);
    nh.param("detector/ekf_hz", ekf_hz_, 60); 
    nh.param("detector/valid_cluster_dist_thresh", valid_cluster_dist_thresh_, 0.7);
    nh.param("detector/valid_cluster_size_thresh", valid_cluster_size_thresh_, 1.0);
    nh.param("detector/predict_region_radius", predict_region_radius_, 1.2);
    nh.param("detector/same_obj_thresh", same_obj_thresh_, 0.4);
    nh.param("detector/min_cluster_size", min_cluster_size_, 10);
    nh.param("detector/min_high_inten_cluster_size", min_high_inten_cluster_size_, 5);
    nh.param("detector/intensity_thresh", intensity_thresh_, 240);
    nh.param("detector/pred_update_rate", pred_update_rate_, 5);
    nh.param("detector/self_meas_noise", self_meas_noise_, 1e-5);
    nh.param("detector/team_meas_noise", team_meas_noise_, 3e-5);
    nh.param("detector/valid_target_odom_dist_thresh", valid_target_odom_dist_thresh_, 9.0);
    nh.param("detector/use_dpt_refine", use_dpt_refine_, false);

    if(use_dpt_refine_)
    {
        nh.param("detector/dpt_resolution", resolution_, 0.025);
        resolution_inv_ = 1.0 / resolution_;
        dpt_origin_ << 0.0, 0.0;
        dpt_angle_size_ << M_PI, 2 * M_PI;   
        dpt_grid_num_ = (dpt_angle_size_ * resolution_inv_).array().ceil().cast<int>();
        dpt_grid_size_ = dpt_grid_num_(1) * dpt_grid_num_(0);
        dpt_buffer_.resize(dpt_grid_size_, 0);
    }
    
    int unit_time_int = (int)(ros::Time::now().toSec()) % 100000;
    detector_log_ = ofstream(DEBUG_FILE_DIR("drone" + std::to_string(drone_id_) + "_detector_log_"  + std::to_string(unit_time_int) + ".txt"), ios::trunc);

   
    string local_ip;
    if(get_local_ip(local_ip)){
        //Set Drone ID
        uint8_t* ip_c = new uint8_t[4];
        StringIp2CharIp(local_ip, ip_c);
        drone_id_ =  (int)(ip_c[3] - 100);
    }


    ekf_tracker_.setMeasData(&meas_data_);

    scan_count_ = 0;
    ekf_count_ = 0;
    pcl_accumulate_.reset(new PointCloudXYZI());
    batch_last_stamp_ = ros::Time::now().toSec();

    has_valid_tracker_ = false;
    has_target_odom_ = false;
    has_trigger_ = true; //set triggered initially
    exist_self_meas_ = false;

    accumu_pcl_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/detector_accumu_pcl_vis", 10);
    cluster_pub_ = nh.advertise<visualization_msgs::Marker>("/detector_cluster_vis", 10);
    tracker_pub_ = nh.advertise<visualization_msgs::Marker>("/detector_tracker_vis",10);
    

}


bool uavDetector::runEKF(ros::Time& time_T, 
                         Eigen::Vector3d& target_pos,
                         Eigen::Vector3d& target_vel)
{   
    if(!has_trigger_)
    {
        ROS_WARN("[Detector] RunEKF: Have not TRIGGERED yet.");
        detector_log_ << "[ " << setiosflags(ios::fixed) << setprecision(4) << ros::Time::now().toSec() << " ] " 
        << " [Detector] RunEKF: Have not TRIGGERED yet." << std::endl;
        return false;
    }

    if(!has_valid_tracker_)
    {
        ROS_WARN("[Detector] RunEKF: Tracker not init yet.");
        detector_log_ << "[ " << setiosflags(ios::fixed) << setprecision(4) << ros::Time::now().toSec() << " ] " 
        << " [Detector] RunEKF: Tracker not init yet." << std::endl;
        return false;
    }
    
    time_T = ros::Time::now();
    double time_t = time_T.toSec(); 
    ekf_tracker_.predict(time_t);

    ekf_count_++;

    if(!(ekf_count_ % pred_update_rate_))
    {
        std::lock_guard<std::mutex> lock(mtx_);
        if(!meas_data_.empty())
        {
            ROS_WARN_STREAM("\033[42;1m TRACKER Updated with " << meas_data_.size() << " measurement. \033[0m");
        }

        if(time_t - ekf_tracker_.last_update_time_ > 1.0 && (!meas_data_.empty()))
        {
            ekf_tracker_.reset(meas_data_[0].pos_, meas_data_[0].t_stamp_);
            auto hd = meas_data_.begin();
            meas_data_.erase(hd);
        }

        ekf_tracker_.updateMeas(time_t);
        meas_data_.clear();
    }
    
    if(time_t - ekf_tracker_.last_update_time_ > 1.0)
    {   
        ROS_WARN_STREAM("[Detector] "<< setiosflags(ios::fixed) << setprecision(4) << "time_t: " <<  time_t << ","
        << " last_update_time_: " << ekf_tracker_.last_update_time_ <<
        " walltime: " << ros::WallTime::now().toSec());
        
        ROS_WARN_STREAM("\033[41;1m [Detector] RunEKF: No update over 1 seconds ! \033[0m");
        detector_log_ << "[ " << setiosflags(ios::fixed) << setprecision(4) << ros::Time::now().toSec() << " ] " 
        << " [Detector] RunEKF: No update over 1 seconds." << std::endl;
        return false;
    }
    
    target_pos = ekf_tracker_.get_state_pos();
    target_vel = ekf_tracker_.get_state_vel();
    return true;
}


bool uavDetector::setSelfMeas(const sensor_msgs::PointCloud2ConstPtr& pcl_msg,
                              Vector3d& target_observation)
{   
    
    if(!has_trigger_) return false;
    double pcl_stamp;

    pcl_stamp = pcl_msg->header.stamp.toSec(); 
    
    if(pcl_stamp < batch_last_stamp_) {
        ROS_WARN("[Detector] Input loop back, clear.");
        detector_log_ << "[ " << setiosflags(ios::fixed) << setprecision(4) << ros::Time::now().toSec() << " ] " 
        << " [Detector] Input loop back, clear." << std::endl;
        return false;
    }

    batch_last_stamp_ = pcl_stamp;

    PointCloudXYZI::Ptr cur_pcl(new PointCloudXYZI());
    pcl::fromROSMsg(*pcl_msg, *cur_pcl);

    *pcl_accumulate_ += *cur_pcl;
    scan_count_++;

    if(scan_count_ >= pcl_frame_num_ && !(pcl_accumulate_->empty()))
    {
        odom_ = OdomData(odom_recv_);
        visualizeAccumuPC(pcl_accumulate_); 

        bool is_obsrv_valid;

        // //Extract the predicted region
        ClusterExtractPredictRegion(pcl_accumulate_);

        // //Extract the high-inten region
        ClusterExtractHighIntensity(pcl_accumulate_);
        
        // //Check if the cluster matches the prediction
        is_obsrv_valid = CheckClusterValidation(pcl_stamp, target_observation);
            
        //set the tracker if first observation
        if(!has_valid_tracker_)
        {   
            initTrackerBySelfMeas(pcl_stamp);
        }

        scan_count_ = 0;
        pcl_accumulate_->clear();
        
        return is_obsrv_valid;
    }else{
        return false;
    }
    
}

void uavDetector::setTeamMeas(const swarm_msgs::TargetObsrvPtr& obsrv_msg)
{   
    if(!has_trigger_) return;

    int recv_id = obsrv_msg->drone_id;
    double obsrv_stamp;

    obsrv_stamp = obsrv_msg->header.stamp.toSec(); //for real exp
   
    if(team_obsrvs_.size() > recv_id &&
       team_obsrvs_[recv_id].drone_id == recv_id &&
       (obsrv_stamp <= team_obsrvs_[recv_id].update_time ||
        obsrv_stamp <= ekf_tracker_.last_update_time_ - 0.5))
    {
        if(obsrv_stamp <= team_obsrvs_[recv_id].update_time)
        {
            ROS_ERROR("[Detector] Duplicated Teammate Observation. No need to Update.");
             detector_log_ << "[ " << setiosflags(ios::fixed) << setprecision(4) << ros::Time::now().toSec() << " ] " 
             << " [Detector] Duplicated Teammate Observation. No need to Update." << std::endl;
        }

        if(obsrv_stamp <= ekf_tracker_.last_update_time_ - 0.5)
        {
            ROS_ERROR("[Detector] ekf_tracker_ Outdated Teammate Observation. No need to Update.");
            detector_log_ << "[ " << setiosflags(ios::fixed) << setprecision(4) << ros::Time::now().toSec() << " ] " 
             << " [Detector] ekf_tracker_ Outdated Teammate Observation. No need to Update." << std::endl;
        }
        
        return;
    }
    
    if(team_obsrvs_.size() <= recv_id)//fill up buffer
    {
        for (size_t i = team_obsrvs_.size(); i <= recv_id; i++)
        {
            TeamObsrv blank;
            blank.update_time = 0.0;
            blank.drone_id = -1;
            team_obsrvs_.emplace_back(blank);
        }
    }

    team_obsrvs_[recv_id].set(obsrv_msg);
    Eigen::Vector3d target_pos(obsrv_msg->target_pos[0],
                               obsrv_msg->target_pos[1],
                               obsrv_msg->target_pos[2]);
    //
    if(!has_valid_tracker_) 
    {
        initTrackerByTeamMeas(obsrv_stamp, target_pos);
        return;
    }

    //TODO reset mechanism
    std::lock_guard<std::mutex> lock(mtx_);
    meas_data_.emplace_back(target_pos, obsrv_stamp, team_meas_noise_);
}

void uavDetector::setOdom(const nav_msgs::OdometryConstPtr& odom_msg)
{   
    odom_recv_ = OdomData(odom_msg);
}

void uavDetector::setTargetOdom(const nav_msgs::OdometryConstPtr& target_odom_msg)
{   
    target_odom_ = OdomData(target_odom_msg);
    has_target_odom_ = true;
}


void uavDetector::setTrigger()
{
    has_trigger_ = true;
    ROS_INFO("\033[32m[Detector] Trigger Set.\033[0m");
}

void uavDetector::visualizeAccumuPC(const PointCloudXYZI::Ptr& accumu_pcl) 
{
    PointCloudXYZI accumu_pcl_in_world;
    pcl::PointXYZI pt;
    Eigen::Vector3d p_g;
    accumu_pcl_in_world.clear();

    if(has_valid_tracker_)
    {
        Vector3d predict_pos = ekf_tracker_.get_state_pos(); //;All in world frame

        for (auto it_pcl = accumu_pcl->points.begin(); it_pcl != accumu_pcl->points.end(); it_pcl++) {
            Vector3d cur_point(it_pcl->x, it_pcl->y, it_pcl->z);
            bool select_bar = (cur_point - predict_pos).norm() < predict_region_radius_;
            if (select_bar) {
                pcl::PointXYZI pt_add;
                pt_add.x = it_pcl->x;
                pt_add.y = it_pcl->y;
                pt_add.z = it_pcl->z;
                pt_add.intensity = it_pcl->intensity;
                accumu_pcl_in_world.push_back(pt_add);
            }    

        }

    }else{
        for(int i = 0; i < accumu_pcl->size(); i++)
        {
            // if(accumu_pcl->points[i].intensity > intensity_thresh_)
            // {
                p_g << accumu_pcl->points[i].x, accumu_pcl->points[i].y, accumu_pcl->points[i].z;
                pt.x = p_g.x(); pt.y = p_g.y(); pt.z = p_g.z();
                pt.intensity = accumu_pcl->points[i].intensity;
                accumu_pcl_in_world.points.push_back(pt);
            // }
        }
    }
    
    accumu_pcl_in_world.width = accumu_pcl_in_world.size();
    accumu_pcl_in_world.height = 1;
    accumu_pcl_in_world.is_dense = true;
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(accumu_pcl_in_world, cloud_msg);
    cloud_msg.header.stamp = ros::Time::now();
    cloud_msg.header.frame_id = "world";
    accumu_pcl_pub_.publish(cloud_msg);
}

void uavDetector::visualizeCluster(const double &lidar_end_time, const Eigen::Vector3d &pos, const int &cluster_index, bool is_high_inten) {

        visualization_msgs::Marker vis_cluster;
        vis_cluster.header.stamp = ros::Time().fromSec(lidar_end_time);
        vis_cluster.header.frame_id = "world";
        vis_cluster.action = visualization_msgs::Marker::ADD;
        vis_cluster.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        vis_cluster.id = cluster_index;
        vis_cluster.color.a = 1.0; // Don't forget to set the alpha!
        vis_cluster.color.r = 1.0;
        vis_cluster.color.g = 1.0;
        vis_cluster.color.b = 0.0;
        vis_cluster.scale.x = 0.3;
        vis_cluster.scale.y = 0.3;
        vis_cluster.scale.z = 0.3;

        Eigen::Vector3d cluster_pos_world =  pos;

        vis_cluster.pose.position.x = cluster_pos_world(0);
        vis_cluster.pose.position.y = cluster_pos_world(1);
        vis_cluster.pose.position.z = cluster_pos_world(2);
        vis_cluster.pose.orientation.x = 0.0;
        vis_cluster.pose.orientation.y = 0.0;
        vis_cluster.pose.orientation.z = 0.0;
        vis_cluster.pose.orientation.w = 1.0;

        vis_cluster.text = is_high_inten ? "High" : "Low";
        cluster_pub_.publish(vis_cluster);

        visualization_msgs::Marker line_strip_cluster;
        line_strip_cluster.header.stamp = ros::Time().fromSec(lidar_end_time);
        line_strip_cluster.header.frame_id = "world";
        line_strip_cluster.action = visualization_msgs::Marker::ADD;
        line_strip_cluster.pose.orientation.w = 1.0;
        line_strip_cluster.id = cluster_index + 100; //unique id, useful when multiple markers exist.
        line_strip_cluster.type = visualization_msgs::Marker::LINE_STRIP; //marker type
        line_strip_cluster.scale.x = 0.1;
        line_strip_cluster.color.r = 1.0;
        line_strip_cluster.color.g = 1.0;
        line_strip_cluster.color.b = 0.0;
        line_strip_cluster.color.a = 0.6; 
        geometry_msgs::Point p[8];
        double length = 0.3, width = 0.2, hight = 0.2;
        p[0].x = cluster_pos_world(0) - width;
        p[0].y = cluster_pos_world(1) + length;
        p[0].z = cluster_pos_world(2) + hight;
        p[1].x = cluster_pos_world(0) - width;
        p[1].y = cluster_pos_world(1) - length;
        p[1].z = cluster_pos_world(2) + hight;
        p[2].x = cluster_pos_world(0) - width;
        p[2].y = cluster_pos_world(1) - length;
        p[2].z = cluster_pos_world(2) - hight;
        p[3].x = cluster_pos_world(0) - width;
        p[3].y = cluster_pos_world(1) + length;
        p[3].z = cluster_pos_world(2) - hight;
        p[4].x = cluster_pos_world(0) + width;
        p[4].y = cluster_pos_world(1) + length;
        p[4].z = cluster_pos_world(2) - hight;
        p[5].x = cluster_pos_world(0) + width;
        p[5].y = cluster_pos_world(1) - length;
        p[5].z = cluster_pos_world(2) - hight;
        p[6].x = cluster_pos_world(0) + width;
        p[6].y = cluster_pos_world(1) - length;
        p[6].z = cluster_pos_world(2) + hight;
        p[7].x = cluster_pos_world(0) + width;
        p[7].y = cluster_pos_world(1) + length;
        p[7].z = cluster_pos_world(2) + hight;
        for (int i = 0; i < 8; i++) {
            line_strip_cluster.points.push_back(p[i]);
        }
        line_strip_cluster.points.push_back(p[0]);
        line_strip_cluster.points.push_back(p[3]);
        line_strip_cluster.points.push_back(p[2]);
        line_strip_cluster.points.push_back(p[5]);
        line_strip_cluster.points.push_back(p[6]);
        line_strip_cluster.points.push_back(p[1]);
        line_strip_cluster.points.push_back(p[0]);
        line_strip_cluster.points.push_back(p[7]);
        line_strip_cluster.points.push_back(p[4]);
        cluster_pub_.publish(line_strip_cluster);
    }

    void uavDetector::visualizeTracker(const double &lidar_end_time) {
      
        Vector3d vis_pos_world = ekf_tracker_.get_state_pos();

        visualization_msgs::Marker line_strip;
        line_strip.header.stamp = ros::Time().fromSec(lidar_end_time);
        line_strip.header.frame_id = "world";
        line_strip.action = visualization_msgs::Marker::ADD;
        line_strip.pose.orientation.w = 1.0;
        line_strip.id = 101;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;
        line_strip.scale.x = 0.1;
        line_strip.color.r = 0.0;
        line_strip.color.g = 1.0;
        line_strip.color.b = 0.5;
        line_strip.color.a = 1.0;
        geometry_msgs::Point p[8];
        double length = predict_region_radius_ - 0.05;
        double width = predict_region_radius_ - 0.05;
        double hight = predict_region_radius_ - 0.05;
        p[0].x = vis_pos_world(0) - width;
        p[0].y = vis_pos_world(1) + length;
        p[0].z = vis_pos_world(2) + hight;
        p[1].x = vis_pos_world(0) - width;
        p[1].y = vis_pos_world(1) - length;
        p[1].z = vis_pos_world(2) + hight;
        p[2].x = vis_pos_world(0) - width;
        p[2].y = vis_pos_world(1) - length;
        p[2].z = vis_pos_world(2) - hight;
        p[3].x = vis_pos_world(0) - width;
        p[3].y = vis_pos_world(1) + length;
        p[3].z = vis_pos_world(2) - hight;
        p[4].x = vis_pos_world(0) + width;
        p[4].y = vis_pos_world(1) + length;
        p[4].z = vis_pos_world(2) - hight;
        p[5].x = vis_pos_world(0) + width;
        p[5].y = vis_pos_world(1) - length;
        p[5].z = vis_pos_world(2) - hight;
        p[6].x = vis_pos_world(0) + width;
        p[6].y = vis_pos_world(1) - length;
        p[6].z = vis_pos_world(2) + hight;
        p[7].x = vis_pos_world(0) + width;
        p[7].y = vis_pos_world(1) + length;
        p[7].z = vis_pos_world(2) + hight;
        for (int i = 0; i < 8; i++) {
            line_strip.points.push_back(p[i]);
        }

        line_strip.points.push_back(p[0]);
        line_strip.points.push_back(p[3]);
        line_strip.points.push_back(p[2]);
        line_strip.points.push_back(p[5]);
        line_strip.points.push_back(p[6]);
        line_strip.points.push_back(p[1]);
        line_strip.points.push_back(p[0]);
        line_strip.points.push_back(p[7]);
        line_strip.points.push_back(p[4]);
        tracker_pub_.publish(line_strip);
    
    }

    void uavDetector::visualizeDeleteAllCluster(const double &time) {
        visualization_msgs::Marker vis_cluster_delete;
        vis_cluster_delete.header.stamp = ros::Time().fromSec(time);
        vis_cluster_delete.header.frame_id = "world";
        vis_cluster_delete.action = visualization_msgs::Marker::DELETEALL;
        cluster_pub_.publish(vis_cluster_delete);
    }