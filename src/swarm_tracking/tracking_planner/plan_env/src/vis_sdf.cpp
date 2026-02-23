#include "plan_env/vis_sdf.h"
#define MIN_INF     -9999999.9 

Eigen::Vector3d hslToRgb(double h, double s, double l) {
    h = std::fmod(h, 360.0);
    if (h < 0) h += 360.0;
    h /= 360.0; 

    double r, g, b;
    
    auto hue2rgb = [](double p, double q, double t) {
        if(t < 0.0) t += 1.0;
        if(t > 1.0) t -= 1.0;
        if(t < 1.0/6.0) return p + (q - p) * 6.0 * t;
        if(t < 0.5)     return q;
        if(t < 2.0/3.0) return p + (q - p) * (2.0/3.0 - t) * 6.0;
        return p;
    };

    if(s == 0) {
        r = g = b = l; 
    } else {
        double q = l < 0.5 ? l * (1.0 + s) : l + s - l * s;
        double p = 2.0 * l - q;
        
        r = hue2rgb(p, q, h + 1.0/3.0);
        g = hue2rgb(p, q, h);
        b = hue2rgb(p, q, h - 1.0/3.0);
    }

    auto clamp = [](double x) { 
        return (x < 0.0) ? 0.0 : ((x > 1.0) ? 1.0 : x); 
    };

    return Eigen::Vector3d(clamp(r), clamp(g), clamp(b));
}


Eigen::Vector3d sdfToColor(double value) {
    const double min_val = -M_PI / 9.3; 
    const double max_val = 0.0;

    double t = (value - max_val) / (min_val - max_val); // t ∈ [0,1]
  
    double hue = 270.0 * (1.0 - t); 
    double saturation = 0.8;        
    double lightness = 0.5;         

    return hslToRgb(hue, saturation, lightness);
}


template <typename F_get_itrsec, typename F_set_val>
void VisSDF::L2Scan(F_get_itrsec f_get_itrsec, F_set_val f_set_val, int start, int end, int vnum, bool can_print)
{
  bool need_print = true;

  int *v = new int[vnum];
  double *z = new double[vnum + 1];


  int k = start;
  v[start] = start;

  z[start] = -std::numeric_limits<double>::max();
  z[start + 1] = std::numeric_limits<double>::max();

  for (int q = start + 1; q <= end; q++) {
    k++;
    double s;

    do {
      k--;
      s = f_get_itrsec(q, v[k]);
    }while (s <= z[k]);

    k++;
    v[k] = q;

    z[k] = s;
    z[k + 1] = std::numeric_limits<double>::max();
  }

  k = start;

  for (int q = start; q <= end; q++) {
    while (z[k + 1] < q * resolution_ + theta_lb_) k++;
    f_set_val(q, v[k]); 
  }

  delete [] v;
  delete [] z;

}

template <typename F_get_val, typename F_set_val>
void VisSDF::L1Scan(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int vnum)
{
  int *v = new int[vnum];
  v[start] = f_get_val(start) ? start : std::numeric_limits<int>::max();
  for(int q = start + 1; q <= end; q++)
  {
    int qmod = q % vnum;
    v[qmod] = f_get_val(qmod) ? (qmod) : v[(q - 1) % vnum];
  }
    
  int emod = end % vnum;
  int s = v[emod]; 
  f_set_val(emod, s);                                                                  
  for(int q = end - 1; q >= start; q--)
  {
    int qmod = q % vnum;

    if(v[qmod] != std::numeric_limits<int>::max() &&
       std::min(abs(qmod - s), vnum - abs(qmod - s)) > 
       std::min(abs(qmod - v[qmod]), vnum - abs(qmod - v[qmod])))
      s = v[qmod];
      
    f_set_val(qmod, s);
  }
  delete [] v;
}

void VisSDF::updateVSDFbase(){

  int z = sdf_voxel_num_(2) - 1;

  for (int x = theta_idx_lb_[z]; x <= theta_idx_ub_[z]; x++) {
    L1Scan([&](int &y) {
      return (occupancy_buffer_[toOccAddress(x, y, z)] == 0); },
      [&](int &y, int &val) { tmp_phi_buffer_[toLayerAddress(x, y)] = val; }, 
      0, 3 * (sdf_voxel_num_[1] - 1) / 2, sdf_voxel_num_[1]);
  }

  for (int y = 0; y <= sdf_voxel_num_[1] - 1; y++) {

    L2Scan(
      [&](int &x1, int &x2) {
        int phi1_idx = tmp_phi_buffer_[toLayerAddress(x1, y)];
        if(phi1_idx == std::numeric_limits<int>::max()) 
          return std::numeric_limits<double>::max();
        
        int phi2_idx = tmp_phi_buffer_[toLayerAddress(x2, y)];
        if(phi2_idx == std::numeric_limits<int>::max()) 
        {
          return MIN_INF;
        }
        
        return calcItrsecTheta(x1, phi1_idx, x2, phi2_idx, y);
      },
      [&](int &x1, int &x2) {
        int phi2_idx = tmp_phi_buffer_[toLayerAddress(x2, y)];
        double dist = calcSphDist(x1, y, x2, phi2_idx);
        distance_buffer_[toDistAddress(x1, y, z)] = dist;
        tmp_distance_buffer_[toLayerAddress(x1, y)] = dist;
        closest_obstacle_buffer_[toLayerAddress(x1, y)] << x2, phi2_idx;
      },
      theta_idx_lb_[z], theta_idx_ub_[z], sdf_voxel_num_[0], (y == 0));
  }

}

void VisSDF::increPropVSDF() 
{
  int tmp_buffer_size = tmp_distance_buffer_.size();
  
  for(int z = sdf_voxel_num_(2) - 2; z >= 0; z--)
  {
    if(!incre_list_[z].empty())
    {
      for(auto &pt_idx : incre_list_[z])
      { 
        int addr = toLayerAddress(pt_idx);
        closest_obstacle_buffer_[addr] = pt_idx;
        tmp_distance_buffer_[addr] = 0.0;
        update_queue_.emplace(pt_idx);
      }
      updateOccQueue(z, false);
    }

    if(!expand_list_[z].empty())
    {
      for(auto &pt_idx : expand_list_[z])
      {
        int addr = toLayerAddress(pt_idx);
        if(getOcc(Eigen::Vector3i(pt_idx(0), pt_idx(1), z)))
        {
          tmp_distance_buffer_[addr] = DBL_MAX;
        }else{
          closest_obstacle_buffer_[addr] = pt_idx;
          tmp_distance_buffer_[addr] = 0.0;
        }
        update_queue_.emplace(pt_idx);
      }
      updateOccQueue(z, true);
    }
    
    incre_list_[z].clear();
    expand_list_[z].clear();
    memcpy(&distance_buffer_[toDistAddress(0, 0, z)], &tmp_distance_buffer_[0], 
           sizeof(double) * tmp_buffer_size);
  }
}

void VisSDF::updateOccQueue(int &layer_id, bool with_expansion)
{
  int hf_phi = (sdf_voxel_num_(1) - 1) / 2;
  while(!update_queue_.empty())
  {
    auto pt_idx = update_queue_.front();
    int addr = toLayerAddress(pt_idx);
    update_queue_.pop();

    if(with_expansion && tmp_distance_buffer_[addr] != 0.0)
    {
      bool change = false;
      for (const auto &dir : dirs0_)
      {
        Eigen::Vector2i nbr_idx = pt_idx + dir;
        if(isInLayer(nbr_idx, layer_id))
        {
          int nbr_addr = toLayerAddress(nbr_idx);
          if(closest_obstacle_buffer_[nbr_addr](0) == undefined_)
          {
            continue;
          }
          double tmp_dist = calcSphDist(pt_idx.x(), pt_idx.y(), 
                            closest_obstacle_buffer_[nbr_addr](0), closest_obstacle_buffer_[nbr_addr](1));
          if (tmp_distance_buffer_[addr] > tmp_dist)
          {
            tmp_distance_buffer_[addr] = tmp_dist;
            change = true;
            closest_obstacle_buffer_[addr] = closest_obstacle_buffer_[nbr_addr];
          }
        }
      }

      if (change) {
        update_queue_.emplace(pt_idx);
        continue;
      }
    }

    Eigen::Vector2i coc_idx = closest_obstacle_buffer_[addr];
    int pt_x = pt_idx(0);
    int pt_y = pt_idx(1);
    int coc_x = coc_idx(0);
    int coc_y = coc_idx(1);

    const std::vector<Eigen::Vector2i> *nbr_dirs;
    if(coc_y == pt_y)
    {
      if(coc_x == pt_x){
        nbr_dirs = &dirs0_;
      }else if(coc_x < pt_x){
        nbr_dirs = &dirs3_;
      }else{
        nbr_dirs = &dirs6_;
      }
    }else{
      bool ind;
      int df = coc_y - pt_y;
      if(df > hf_phi){
        ind = false;
      }else if(df < -hf_phi){
        ind = true;
      }else{
        ind = (df > 0);
      }

      if(ind){
        if(coc_x == pt_x){
          nbr_dirs = &dirs2_;
        }else if(coc_x < pt_x){
          nbr_dirs = &dirs5_;
        }else{
          nbr_dirs = &dirs8_;
        }
      }else{
        if(coc_x == pt_x){
          nbr_dirs = &dirs1_;
        }else if(coc_x < pt_x){
          nbr_dirs = &dirs4_;
        }else{
          nbr_dirs = &dirs7_;
        }
      }
    }
    
    for(const auto &dir : *nbr_dirs)
    {
      Eigen::Vector2i nbr_idx = pt_idx + dir;
      
      if(!isInLayer(nbr_idx, layer_id)) continue; 
      int nbr_addr = toLayerAddress(nbr_idx);
      double tmp_dist = calcSphDist(nbr_idx(0), nbr_idx(1), coc_x, coc_y);
      if(tmp_distance_buffer_[nbr_addr] > tmp_dist)
      {
        tmp_distance_buffer_[nbr_addr] = tmp_dist;
        closest_obstacle_buffer_[nbr_addr] = coc_idx;
        update_queue_.emplace(nbr_idx);
      }
    }
  }
}


void VisSDF::publishSDF(){

  Eigen::Vector3i local_box_min, local_box_max;
  Eigen::Vector3i sph_idx;

  double slice_z = std::max(height_lb_ + 0.2, ctr_pos_(2) );
  map_ptr_->posToGlobalIndex(Eigen::Vector3d(ctr_pos_(0) - rd_ub_, ctr_pos_(1) - rd_ub_, slice_z), local_box_min);
  map_ptr_->posToGlobalIndex(Eigen::Vector3d(ctr_pos_(0) + rd_ub_, ctr_pos_(1) + rd_ub_, slice_z), local_box_max);

  const double min_dist = -M_PI;
  const double max_dist = 0;
  double dist;
  pcl::PointCloud<pcl::PointXYZI> cloud;
  pcl::PointXYZI pt;
  Eigen::Vector3i tmp_p;
  Eigen::Vector3d crt;
  Eigen::Vector3d grad;
  tmp_p.z() = local_box_min(2);
  for (tmp_p.x() = local_box_min.x(); tmp_p.x() <= local_box_max.x(); tmp_p.x()++) {
        for (tmp_p.y() = local_box_min.y(); tmp_p.y() <= local_box_max.y(); tmp_p.y()++) {
              map_ptr_->globalIndexToPos(tmp_p, crt);
              evaluateSDTwithGradTrilinear(crt, dist, grad);
              if(dist <= 1e-4) continue;
              dist = -dist;
              dist = min(dist, max_dist);
              dist = max(dist, min_dist);
              pt.x = crt(0);
              pt.y = crt(1);
              pt.z = crt(2);
              pt.intensity = (dist - min_dist) / (max_dist - min_dist);
              cloud.push_back(pt);
      }
  }
  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = "world";
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);

  vsdf_pub_.publish(cloud_msg);
}



void VisSDF::publishSpherewithColorTF()
{
  Eigen::Vector3i min_vsdf(0, 0, 0);
  Eigen::Vector3i max_vsdf = sdf_voxel_num_ - Eigen::Vector3i::Ones();

  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  pcl::PointXYZRGB pt;

  double radius = rd_ub_ - 0.3;

  Eigen::Vector3d tmp_tpr;
  Eigen::Vector3d crt;
  Eigen::Vector3d grad;
  Eigen::Vector3d rgb_val;


  double tmp_tpr_step = 0.015;
  double dist;
  tmp_tpr.z() = radius;
  double phi_lb = 0.0;
  double phi_ub = M_PI * 2 - tmp_tpr_step;
  double theta_lb = 0.0;
  double theta_ub = M_PI;

  for (tmp_tpr.x() = theta_lb; tmp_tpr.x() <= theta_ub; tmp_tpr.x() += tmp_tpr_step) {
    for (tmp_tpr.y() = phi_lb; tmp_tpr.y() <= phi_ub; tmp_tpr.y() += tmp_tpr_step) {

      s2c(tmp_tpr, crt);
      crt += ctr_pos_;

      evaluateSDTwithGradTrilinear(crt, dist, grad);
      if(dist <= 5e-5) 
      {
        continue;
      }
        dist = -dist;
        pt.x = crt(0);
        pt.y = crt(1);
        pt.z = crt(2);
        rgb_val = sdfToColor(dist);

        pt.r = static_cast<uint8_t>((rgb_val[0] * 255));
        pt.g = static_cast<uint8_t>((rgb_val[1] * 255));
        pt.b = static_cast<uint8_t>((rgb_val[2] * 255));
        cloud.push_back(pt);
    }
  }
  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = "world";
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);

  sph_colored_pub_.publish(cloud_msg);
}



void VisSDF::publishSDFwithColorTF(){

  Eigen::Vector3i local_box_min, local_box_max;
  Eigen::Vector3i sph_idx;

  double slice_z = std::max(height_lb_ + 0.2, ctr_pos_(2) );

  double dist;
  Eigen::Vector3d tmp_p;
  Eigen::Vector3d grad;
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  pcl::PointXYZRGB pt;
  Eigen::Vector3d rgb_val;

  double min_sdf_val{0.0};
  double x_lb = ctr_pos_(0) - rd_ub_;
  double x_ub = ctr_pos_(0) + rd_ub_;
  double y_lb = ctr_pos_(1) - rd_ub_;
  double y_ub = ctr_pos_(1) + rd_ub_;
  double tmp_p_step = 0.05;

  tmp_p.z() = slice_z;
  for (tmp_p.x() = x_lb; tmp_p.x() <= x_ub; tmp_p.x() += tmp_p_step) {
      for (tmp_p.y() = y_lb; tmp_p.y() <= y_ub; tmp_p.y() += tmp_p_step) {
    
          if( (tmp_p - ctr_pos_).norm() > rd_ub_ ) continue;

          evaluateSDTwithGradTrilinear(tmp_p, dist, grad);
          if(dist <= 5e-5) continue;
          
          dist = -dist;
          
          pt.x = tmp_p(0);
          pt.y = tmp_p(1);
          pt.z = tmp_p(2);
          rgb_val = sdfToColor(dist);

          if(dist < min_sdf_val) min_sdf_val = dist;

          pt.r = static_cast<uint8_t>((rgb_val[0] * 255));
          pt.g = static_cast<uint8_t>((rgb_val[1] * 255));
          pt.b = static_cast<uint8_t>((rgb_val[2] * 255));

          cloud.push_back(pt);
      }
  }
  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = "world";
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  vsdf_colored_pub_.publish(cloud_msg);
}



void VisSDF::initSDF(ros::NodeHandle &nh, std::shared_ptr<rog_map::ROGMap> &mapPtr)
{
  map_ptr_ = mapPtr;

  nh.param("VSDF/theta_lb", theta_lb_, 0.0);
  nh.param("VSDF/theta_ub", theta_ub_, M_PI);
  nh.param("VSDF/rd_lb", rd_lb_, 0.5);
  nh.param("VSDF/rd_ub", rd_ub_, 6.0);
  nh.param("VSDF/height_lb", height_lb_, 0.01);
  nh.param("VSDF/height_ub", height_ub_, 5.5);
  nh.param("VSDF/resolution", resolution_, 0.1);
  nh.param("VSDF/inflation_step", inflation_step_, 1);
  nh.param("VSDF/have_floor", floor_as_occ_, true);
  nh.param("VSDF/have_ceiling", ceiling_as_occ_, false);

  ROS_WARN_STREAM("VSDF/theta_lb: " << theta_lb_);
  ROS_WARN_STREAM("VSDF/theta_ub: " << theta_ub_);
  ROS_WARN_STREAM("VSDF/rd_lb: " << rd_lb_);
  ROS_WARN_STREAM("VSDF/rd_ub: " << rd_ub_);
  ROS_WARN_STREAM("VSDF/height_lb: " << height_lb_);
  ROS_WARN_STREAM("VSDF/height_ub: " << height_ub_);
  ROS_WARN_STREAM("VSDF/resolution: " << resolution_);
  ROS_WARN_STREAM("VSDF/inflation_step: " << inflation_step_);

  resolution_inv_ = 1.0 / resolution_;
  sdf_origin_ << theta_lb_, 0.0, rd_lb_;
  sdf_size_ << theta_ub_ - theta_lb_, 2 * M_PI, rd_ub_ - rd_lb_;
  sdf_voxel_num_ = (sdf_size_ * resolution_inv_).array().ceil().cast<int>();

  int layer_size = sdf_voxel_num_(0) * sdf_voxel_num_(1);
  int buffer_size = layer_size * sdf_voxel_num_(2);

  std::vector<Eigen::Vector2i> blank; blank.clear();
  incre_list_.resize(sdf_voxel_num_(2), blank);
  expand_list_.resize(sdf_voxel_num_(2), blank);
  theta_idx_lb_.resize(sdf_voxel_num_(2), 0);
  theta_idx_ub_.resize(sdf_voxel_num_(2), sdf_voxel_num_(0) - 1);

  occupancy_buffer_.resize(buffer_size, 0);
  tmp_phi_buffer_.resize(layer_size, 0);
  distance_buffer_.resize(buffer_size, 0.0);
  tmp_distance_buffer_.resize(layer_size, 0.0);
  depth_buffer_.resize(layer_size, numeric_limits<int>::max());
  closest_obstacle_buffer_.resize(sdf_voxel_num_(0) * sdf_voxel_num_(1), 
                           Eigen::Vector2i(undefined_, undefined_));  

  if(inflation_step_ > 0)
    for (int dx = -inflation_step_; dx <= inflation_step_; dx++)
      for (int dy = -inflation_step_; dy <= inflation_step_; dy++)
        if(dx || dy) inflation_dirs_.emplace_back(dx, dy);
         
  vsdf_pub_ = nh.advertise<sensor_msgs::PointCloud2>("vsdf/sdf", 10);
  vsdf_colored_pub_ = nh.advertise<sensor_msgs::PointCloud2>("vsdf/sdf_colored", 10);
  sph_colored_pub_ = nh.advertise<sensor_msgs::PointCloud2>("vsdf/sph_colored", 10);
}


void VisSDF::resetSDF(const Eigen::Vector3d &center_pos)
{
  incre_list_.clear();
  expand_list_.clear();
  std::fill(occupancy_buffer_.begin(), occupancy_buffer_.end(), 0);
  std::fill(depth_buffer_.begin(), depth_buffer_.end(), numeric_limits<int>::max());
  std::fill(closest_obstacle_buffer_.begin(), closest_obstacle_buffer_.end(), 
            Eigen::Vector2i(undefined_, undefined_));
  std::fill(tmp_distance_buffer_.begin(), tmp_distance_buffer_.end(), 0.0);
  std::fill(theta_idx_lb_.begin(), theta_idx_lb_.end(), 0);
  std::fill(theta_idx_ub_.begin(), theta_idx_ub_.end(), sdf_voxel_num_(0) - 1);
  ctr_pos_ = center_pos;

  double upper_height = height_ub_ - ctr_pos_(2);
  double lower_height = ctr_pos_(2) - height_lb_;

  for(int i = sdf_voxel_num_[2] - 1; i >= 0; i--)
  { 
    double rad = (i + 0.5) * resolution_ + rd_lb_;
    if(ceiling_as_occ_) theta_idx_lb_[i] = (int)floor((aCos(upper_height / rad) - theta_lb_) * resolution_inv_);
    if(floor_as_occ_) theta_idx_ub_[i] = (int)floor((M_PI - aCos(lower_height / rad) - theta_lb_) * resolution_inv_);
    if(i < sdf_voxel_num_[2] - 1)
    { 
      if(ceiling_as_occ_){
        for(int theta_idx = theta_idx_lb_[i + 1] - 1; theta_idx >= theta_idx_lb_[i]; theta_idx--)
          for(int phi_idx = 0; phi_idx < sdf_voxel_num_(1); phi_idx++)
            expand_list_[i].emplace_back(theta_idx, phi_idx);
      }
      if(floor_as_occ_){
        for(int theta_idx = theta_idx_ub_[i + 1] + 1; theta_idx <= theta_idx_ub_[i]; theta_idx++)
          for(int phi_idx = 0; phi_idx < sdf_voxel_num_(1); phi_idx++)
            expand_list_[i].emplace_back(theta_idx, phi_idx);
      }
    }
  }
}

void VisSDF::generateSDF(const Eigen::Vector3d &center_pos)
{
    //processing
    resetSDF(center_pos);
    boxHit();         //visibility map
    updateVSDFbase(); //2D ssdf grid update
    increPropVSDF();  //incremental update
}

void VisSDF::boxHit()
{
  Eigen::Vector3i local_box_min, local_box_max;
  map_ptr_->posToGlobalIndex(Eigen::Vector3d(ctr_pos_(0) - rd_ub_, ctr_pos_(1) - rd_ub_, height_lb_), local_box_min);
  map_ptr_->posToGlobalIndex(Eigen::Vector3d(ctr_pos_(0) + rd_ub_, ctr_pos_(1) + rd_ub_, height_ub_), local_box_max);

  Eigen::Vector3d crt;
  Eigen::Vector3i crt_idx;
  Eigen::Vector3i sph_idx;

  int tmp_id;
  int stride = 1;
  for (crt_idx.x() = local_box_min.x(); crt_idx.x() <= local_box_max.x(); crt_idx.x() += stride + 1) {
      for (crt_idx.y() = local_box_min.y(); crt_idx.y() <= local_box_max.y(); crt_idx.y() += stride + 1) {
          for (crt_idx.z() = local_box_min.z(); crt_idx.z() <= local_box_max.z(); crt_idx.z() += stride + 1) {
              if (map_ptr_->isOccupiedInflate(crt_idx)) {
                map_ptr_->globalIndexToPos(crt_idx, crt);
                crt2idx(crt, sph_idx);
                if(isInSDF(sph_idx))
                {
                  Eigen::Vector2i pt_id(sph_idx(0), sph_idx(1));
                  tmp_id = toLayerAddress(pt_id);
                  if (depth_buffer_[tmp_id] > sph_idx.z())
                    depth_buffer_[tmp_id] = sph_idx.z();

                  if(inflation_step_ > 0)
                  { 
                    for (const auto &dir : inflation_dirs_)
                    {
                      Eigen::Vector2i nbr_id = pt_id + dir;
                      if(!isInLayer(nbr_id, sph_idx(2))) continue;
                      tmp_id = toLayerAddress(nbr_id);
                      if(depth_buffer_[tmp_id] > sph_idx.z())
                        depth_buffer_[tmp_id] = sph_idx.z();
                    }
                  }
                }
              }   
          }
      }
  }

  for(sph_idx.x() = 0; sph_idx.x() < sdf_voxel_num_(0); sph_idx.x()++)
    for(sph_idx.y() = 0; sph_idx.y() < sdf_voxel_num_(1); sph_idx.y()++)
    {
      tmp_id = depth_buffer_[toLayerAddress(sph_idx.x(), sph_idx.y())];
      if(tmp_id < numeric_limits<int>::max()){
        if(tmp_id) incre_list_[tmp_id - 1].emplace_back(sph_idx.x(), sph_idx.y());
        memset(&(occupancy_buffer_[toOccAddress(sph_idx.x(), sph_idx.y(), tmp_id)]), 
               1, sdf_voxel_num_(2) - tmp_id);
      }
    }
}



void VisSDF::interpolateSDTwithGrad(double values[2][2],
                                    const Eigen::Vector3d& diff,
                                    double& value,
                                    Eigen::Vector3d& grad){

  double v0 = (1 - diff(0)) * values[0][0] + diff(0) * values[1][0];  
  double v1 = (1 - diff(0)) * values[0][1] + diff(0) * values[1][1];  
  value = (1 - diff(1)) * v0 + diff(1) * v1;

  grad(2) = 0;
  grad(1) = (v1 - v0) * resolution_inv_;
  grad(0) = ((values[1][0] - values[0][0]) * (1 - diff(1)) + 
             (values[1][1] - values[0][1]) * diff(1)) * resolution_inv_;
}


void VisSDF::evaluateSDTwithGrad(const Eigen::Vector3d& crt,
                                 double& dist,
                                 Eigen::Vector3d& grad){
  Eigen::Vector3d sph, crt_p, grad_sph;
  crt_p = crt - ctr_pos_;
  c2s(crt_p, sph);

  if(!isInSDF(sph))
  {
    dist = -M_PI;
    grad.setZero();
    return;
  }

  Eigen::Vector3d diff_ratio;
  Eigen::Vector3d sur_pts[2][2];
  double sur_dist, sur_dists[2][2];
  Eigen::Vector3d sph_m = sph - 0.5 * resolution_ * Eigen::Vector3d::Ones();
  Eigen::Vector3i idx;
  Eigen::Vector3d idx_sph;

  sph2idx(sph_m, idx);
  if(!isInLayer(idx))
  {
    dist = -M_PI;
    grad.setZero();
    return;
  }
  idx2sph(idx, idx_sph);
  diff_ratio(0) = (sph(0) - idx_sph(0)) * resolution_inv_;
  diff_ratio(2) = (sph(2) - idx_sph(2)) * resolution_inv_;
  diff_ratio(1) = idx_sph(1) > sph(1) ? (2 * M_PI - idx_sph(1) + sph(1)) * resolution_inv_ : 
                                        (sph(1) - idx_sph(1)) * resolution_inv_;

  for (int x = 0; x < 2; x++) {
    for (int y = 0; y < 2; y++) {
        Eigen::Vector3i current_idx = idx + Eigen::Vector3i(x, y, 0);
        sur_dist = getDist(current_idx);
        if(sur_dist < 0)
        {
          dist = sur_dist;
          grad.setZero();
          return;
        }else{
          sur_dists[x][y] = sur_dist;
        }
        idx2sph(current_idx, sur_pts[x][y]);
    }
  }

  interpolateSDTwithGrad(sur_dists, diff_ratio, dist, grad_sph);

  double x{crt_p(0)}, y{crt_p(1)}, z{crt_p(2)}, r{sph(2)};
  double r2 = sqrt(x * x + y * y);
  grad(0) = grad_sph(0) * x * z / (r * r * r2) - grad_sph(1) * y / (r2 * r2) + grad_sph(2) * x / r;
  grad(1) = grad_sph(0) * y * z / (r * r * r2) + grad_sph(1) * x / (r2 * r2) + grad_sph(2) * y / r;
  grad(2) = grad_sph(2) * z / r - grad_sph(0) * r2 / (r * r);
}


void VisSDF::evaluateSDTwithGradTrilinear(const Eigen::Vector3d& crt,
                                          double& dist,
                                          Eigen::Vector3d& grad){
  Eigen::Vector3d sph, crt_p, grad_sph;
  crt_p = crt - ctr_pos_;
  c2s(crt_p, sph);

  if(!isInSDF(sph, 1))
  {
    dist = -M_PI;
    grad.setZero();
    return;
  }

  Eigen::Vector3d diff_ratio;
  double sur_dist, sur_dists[2][2][2];
  Eigen::Vector3d sph_m = sph - 0.5 * resolution_ * Eigen::Vector3d::Ones();
  Eigen::Vector3i idx;
  Eigen::Vector3d idx_sph;

  sph2idx(sph_m, idx);
  if(!isInLayer(idx))
  {
    dist = -M_PI;
    grad.setZero();
    return;
  }
  idx2sph(idx, idx_sph);
  diff_ratio(0) = (sph(0) - idx_sph(0)) * resolution_inv_;
  diff_ratio(2) = (sph(2) - idx_sph(2)) * resolution_inv_;
  diff_ratio(1) = idx_sph(1) > sph(1) ? (2 * M_PI - idx_sph(1) + sph(1)) * resolution_inv_ : 
                                        (sph(1) - idx_sph(1)) * resolution_inv_;

  for (int x = 0; x < 2; x++) {
    for (int y = 0; y < 2; y++) {
      for (int z = 0; z < 2; z++) {
        Eigen::Vector3i current_idx = idx + Eigen::Vector3i(x, y, z);
        if(current_idx(1) >= sdf_voxel_num_(1)) current_idx(1) = 0;
        sur_dist = getDist(current_idx);
        if(sur_dist < 0)
        {
          dist = sur_dist;
          grad.setZero();
          return;
        }else{
          sur_dists[x][y][z] = sur_dist;
        }
      }
    }
  }

  interpolateSDTwithGradTrilinear(sur_dists, diff_ratio, dist, grad_sph);

  double x{crt_p(0)}, y{crt_p(1)}, z{crt_p(2)}, r{sph(2)};
  double r2 = sqrt(x * x + y * y);
  grad(0) = grad_sph(0) * x * z / (r * r * r2) - grad_sph(1) * y / (r2 * r2) + grad_sph(2) * x / r;
  grad(1) = grad_sph(0) * y * z / (r * r * r2) + grad_sph(1) * x / (r2 * r2) + grad_sph(2) * y / r;
  grad(2) = grad_sph(2) * z / r - grad_sph(0) * r2 / (r * r);
}


void VisSDF::interpolateSDTwithGradTrilinear(double values[2][2][2], const Eigen::Vector3d& diff, 
                                             double& value, Eigen::Vector3d& grad)
{
  double v00 = (1 - diff(0)) * values[0][0][0] + diff(0) * values[1][0][0]; 
  double v01 = (1 - diff(0)) * values[0][0][1] + diff(0) * values[1][0][1]; 
  double v10 = (1 - diff(0)) * values[0][1][0] + diff(0) * values[1][1][0]; 
  double v11 = (1 - diff(0)) * values[0][1][1] + diff(0) * values[1][1][1]; 
  double v0 = (1 - diff(1)) * v00 + diff(1) * v10; 
  double v1 = (1 - diff(1)) * v01 + diff(1) * v11;  

  value = (1 - diff(2)) * v0 + diff(2) * v1;

  grad[2] = (v1 - v0) * resolution_inv_;
  grad[1] = ((1 - diff[2]) * (v10 - v00) + diff[2] * (v11 - v01)) * resolution_inv_;
  grad[0] = (1 - diff[2]) * (1 - diff[1]) * (values[1][0][0] - values[0][0][0]);
  grad[0] += (1 - diff[2]) * diff[1] * (values[1][1][0] - values[0][1][0]);
  grad[0] += diff[2] * (1 - diff[1]) * (values[1][0][1] - values[0][0][1]);
  grad[0] += diff[2] * diff[1] * (values[1][1][1] - values[0][1][1]);
  grad[0] *= resolution_inv_;
}


void VisSDF::evaluateSDTTrilinear(const Eigen::Vector3d& crt,
                                  double& dist){
  Eigen::Vector3d sph, crt_p, grad_sph;
  crt_p = crt - ctr_pos_;
  c2s(crt_p, sph);

  if(!isInSDF(sph))
  {
    dist = -M_PI;
    return;
  }

  Eigen::Vector3d diff_ratio;
  Eigen::Vector3d sur_pts[2][2][2];
  double sur_dist, sur_dists[2][2][2];
  Eigen::Vector3d sph_m = sph - 0.5 * resolution_ * Eigen::Vector3d::Ones();
  Eigen::Vector3i idx;
  Eigen::Vector3d idx_sph;

  sph2idx(sph_m, idx);
  if(!isInLayer(idx))
  {
    dist = -M_PI;
    return;
  }
  idx2sph(idx, idx_sph);
  diff_ratio(0) = (sph(0) - idx_sph(0)) * resolution_inv_;
  diff_ratio(2) = (sph(2) - idx_sph(2)) * resolution_inv_;
  diff_ratio(1) = idx_sph(1) > sph(1) ? (2 * M_PI - idx_sph(1) + sph(1)) * resolution_inv_ : 
                                        (sph(1) - idx_sph(1)) * resolution_inv_;

  for (int x = 0; x < 2; x++) {
    for (int y = 0; y < 2; y++) {
      for (int z = 0; z < 2; z++) {
        Eigen::Vector3i current_idx = idx + Eigen::Vector3i(x, y, z);
        sur_dist = getDist(current_idx);
        if(sur_dist < 0)
        {
          dist = sur_dist;
          return;
        }else{
          sur_dists[x][y][z] = sur_dist;
        }
        idx2sph(current_idx, sur_pts[x][y][z]);
      }
    }
  }

  interpolateSDTTrilinear(sur_dists, diff_ratio, dist);
}

void VisSDF::interpolateSDTTrilinear(double values[2][2][2], const Eigen::Vector3d& diff, 
                                     double& value)
{
  double v00 = (1 - diff(0)) * values[0][0][0] + diff(0) * values[1][0][0]; 
  double v01 = (1 - diff(0)) * values[0][0][1] + diff(0) * values[1][0][1]; 
  double v10 = (1 - diff(0)) * values[0][1][0] + diff(0) * values[1][1][0]; 
  double v11 = (1 - diff(0)) * values[0][1][1] + diff(0) * values[1][1][1]; 
  double v0 = (1 - diff(1)) * v00 + diff(1) * v10; 
  double v1 = (1 - diff(1)) * v01 + diff(1) * v11;  

  value = (1 - diff(2)) * v0 + diff(2) * v1;
}