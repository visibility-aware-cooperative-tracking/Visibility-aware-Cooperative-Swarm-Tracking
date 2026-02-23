#include <multi_map_manage/multi_map_manager.h>

class HilbertCurve2D;

namespace rog_map {
    MultiMapManager::MultiMapManager() {

    }

    MultiMapManager::~MultiMapManager() {
      if(is_real_experiment_)
      {
        transmit_log_.close();
        recv_log_.close();
        timediff_log_.close();
        timediff_offset_log_.close();
        pack_send_diff_log_.close();
        recv_insert_diff_log_.close();
      }
       
    }
    
    void MultiMapManager::setMap(ROGMap* map) {
        this->map_ = map;
    }  

    void MultiMapManager::init()
    {
        node_.param("multi_map_manager/drone_id", drone_id_, -1);
        node_.param("multi_map_manager/drone_num", map_num_, 4);
        node_.param("multi_map_manager/voxel_num_thresh", voxel_num_threshold_, 10000);
        node_.param("multi_map_manager/voxel_time_thresh", time_threshold_, 1.0);
        node_.param("multi_map_manager/is_real_experiment", is_real_experiment_, false);
        node_.param("multi_map_manager/remedy_idx_num_threshold", remedy_idx_num_threshold_, 15);

        //get drone_id from IP
        if(is_real_experiment_){
          string local_ip;
          if(get_local_ip(local_ip)){
              //Set Drone ID
              uint8_t* ip_c = new uint8_t[4];
              StringIp2CharIp(local_ip, ip_c);
              drone_id_ =  (int)(ip_c[3] - 100);
          }
        }

        ROS_WARN_STREAM("multi_map_manager/drone_id: " << drone_id_);
        ROS_WARN_STREAM("multi_map_manager/drone_num: " << map_num_);
        ROS_WARN_STREAM("multi_map_manager/voxel_num_thresh: " << voxel_num_threshold_);
        ROS_WARN_STREAM("multi_map_manager/voxel_time_thresh " << time_threshold_);

        localmaplist_pub_ = node_.advertise<swarm_msgs::LocalMapList>("/multi_map_manager/localmap_data_send", 5000);
        Eigen::Vector3i map_size = map_->getMapSize();
        buffer_size_ = map_size(0) * map_size(1) * map_size(2);
        is_voxel_update_buffer_ = vector<int>(buffer_size_, 0);
        localupdate_min_ << INT_MAX, INT_MAX, INT_MAX;
        localupdate_max_ << INT_MIN, INT_MIN, INT_MIN;
        updated_voxel_ids_buffer_.clear();
        
        self_map_chunks_.clear();
        last_send_idx_.resize(map_num_);
        swarm_chunk_idx_list_.resize(map_num_);
        chunk_to_insert_buffer_.resize(map_num_);
        remedy_upto_idx_.resize(map_num_);
        for(int i = 0; i < map_num_; i++)
        {
          last_send_idx_[i] = make_pair(0 , 0);
          swarm_chunk_idx_list_[i] = {};
          chunk_to_insert_buffer_[i].clear();
          remedy_upto_idx_[i] = -99;
        }

        stamp_timer_ = node_.createTimer(ros::Duration(0.05), &MultiMapManager::stampTimerCallback, this);
        pack_chunk_timer_ = node_.createTimer(ros::Duration(time_threshold_ / 2.5), &MultiMapManager::packChunkTimerCallback, this);
       
        stamp_pub_ = node_.advertise<swarm_msgs::ChunkStamps>("/multi_map_manager/chunk_stamps_send", 500);
        stamp_sub_ = node_.subscribe(
            "/multi_map_manager/chunk_stamps_recv", 500, &MultiMapManager::stampMsgCallback, this);
        chunk_sub_ = node_.subscribe("/multi_map_manager/localmap_data_recv", 5000,
            &MultiMapManager::chunkMsgCallback, this, ros::TransportHints().tcpNoDelay());
        ST_offset_sub_ = node_.subscribe("/spatial_temporal_offset", 1000, 
            &MultiMapManager::STOffsetCallback, this, ros::TransportHints().tcpNoDelay());
        
        last_packup_time_ = ros::Time::now();
        total_bw_ = 0;

        if(is_real_experiment_)
        {
          string pkg_path1 = ros::package::getPath("rog_map");
          pkg_path1.append("/doc/log_" + std::to_string(drone_id_) + "_localmap_transmit.txt");
          transmit_log_.open(pkg_path1.c_str(), std::ios_base::out);

          string pkg_path2 = ros::package::getPath("rog_map");
          pkg_path2.append("/doc/log_" + std::to_string(drone_id_) + "_localmap_recv.txt");
          recv_log_.open(pkg_path2.c_str(), std::ios_base::out);

          string pkg_path3 = ros::package::getPath("rog_map");
          pkg_path3.append("/doc/log_" + std::to_string(drone_id_) + "_pack_insert_diff.txt");
          timediff_log_.open(pkg_path3.c_str(), std::ios_base::out);
          
          string pkg_path4 = ros::package::getPath("rog_map");
          pkg_path4.append("/doc/log_" + std::to_string(drone_id_) + "_pack_insert_offset_diff.txt");
          timediff_offset_log_.open(pkg_path4.c_str(), std::ios_base::out);
          
          string pkg_path5 = ros::package::getPath("rog_map");
          pkg_path5.append("/doc/log_" + std::to_string(drone_id_) + "_pack_send_diff.txt");
          pack_send_diff_log_.open(pkg_path5.c_str(), std::ios_base::out);

          string pkg_path6 = ros::package::getPath("rog_map");
          pkg_path6.append("/doc/log_" + std::to_string(drone_id_) + "_recv_insert_diff.txt");
          recv_insert_diff_log_.open(pkg_path6.c_str(), std::ios_base::out);
        }
      
    }



/* ---- Callbacks / Pipeline ---- */
//Send stamp as the request
void MultiMapManager::stampTimerCallback(const ros::TimerEvent& e) { 

  // Send stamp of chunks to other drones
  swarm_msgs::ChunkStamps msg;
  msg.from_drone_id = drone_id_;
  msg.time = ros::Time::now().toSec();
  auto swarm_idx_intervals = swarm_chunk_idx_list_;
  for (auto &idx_list : swarm_idx_intervals) {
    swarm_msgs::IdxList idx_list_msg;
    idx_list_msg.ids = idx_list; 
    msg.idx_lists.emplace_back(idx_list_msg);
  }
  stamp_pub_.publish(msg);
  return;
}


//Send teammate with missed chunks as the response
void MultiMapManager::stampMsgCallback(const swarm_msgs::ChunkStampsConstPtr& msg) {

  int from_drone_id = msg->from_drone_id;
  if (from_drone_id == drone_id_) return;
  if (drone_id_ >= map_num_) return;  // Ground node does not send chunk

  // Check others' stamp info and send chunks unknown by them
    vector<int> missed;

    findMissedChunkIds(swarm_chunk_idx_list_[drone_id_], msg->idx_lists[drone_id_].ids, missed);
 
    if(missed.empty()) return; 

 

    bool have_new_chunk_missed;
    int left_idx = missed.size() - 2;
    int right_idx = missed.size() - 1;
    if(last_send_idx_[from_drone_id].second >= missed[left_idx]){
      if(last_send_idx_[from_drone_id].second + 1 > missed[right_idx]){
          have_new_chunk_missed = false;
        }else{
          have_new_chunk_missed = true;
          missed[left_idx] = last_send_idx_[from_drone_id].second + 1;
        }
    }else{
      have_new_chunk_missed = true;
    }

    

    if(getIdsListNum(missed) > remedy_idx_num_threshold_){
      
      if(swarm_chunk_idx_list_[drone_id_].back() - remedy_upto_idx_[from_drone_id] > 
        (int)(3.0 / time_threshold_)) 
      {

  
        packAndSendRemedyChunk(missed, from_drone_id);

        remedy_upto_idx_[from_drone_id] = missed.back();
        last_send_idx_[from_drone_id] = make_pair(missed.front(), missed.back());

      }else{ 
        if(!have_new_chunk_missed) { 
          return; 
          }
        trimChunkIds(missed, remedy_upto_idx_[from_drone_id]);
        if(missed.empty()) {
          return;
        }
        sendChunks(msg->from_drone_id, missed);
        last_send_idx_[from_drone_id] = make_pair(missed[missed.size() - 2], missed[missed.size() - 1]); 
        
      }
    }else{
      if(!have_new_chunk_missed) 
      {
        return;
      }
      sendChunks(msg->from_drone_id, missed);
      last_send_idx_[from_drone_id] = make_pair(missed[left_idx], missed[right_idx]); 
     
    }
}

//Send Chunk
void MultiMapManager::sendChunks(const int& to_drone_id, const vector<int>& idx_list) {

  swarm_msgs::LocalMapList msg_list;
  msg_list.from_drone_id = drone_id_; 
  msg_list.to_drone_id = to_drone_id;
  msg_list.is_remedy_chunk = false;

  for (int i = 0; i < idx_list.size(); i += 2) {
    for (int j = idx_list[i]; j <= idx_list[i + 1]; ++j) {
      swarm_msgs::LocalMapData msg;
      msg = self_map_chunks_[j - 1].map_msg_data_;

      msg_list.LocalMapDataList.emplace_back(msg);

      if(pack_send_diff_log_.is_open())
      pack_send_diff_log_ << 1000 * (ros::Time::now().toSec() - msg.pack_time.toSec()) << " " <<  msg.idx << ", to drone_" <<  to_drone_id << ", " << ros::Time::now() << std::endl; 
    }
  }
  uint32_t serial_size = ros::serialization::serializationLength(msg_list);
  double volume = (double)(serial_size) / 1024;
  if(transmit_log_.is_open()){
    transmit_log_ << volume << " " << ros::Time::now() << " Normal" << std::endl;
  }
  localmaplist_pub_.publish(msg_list);
}


//Receive and insert the received response(chunk)
void MultiMapManager::chunkMsgCallback(const swarm_msgs::LocalMapListConstPtr& msg_list) {

  if(msg_list->to_drone_id != drone_id_) 
  {
    if(is_real_experiment_) ROS_WARN_STREAM("Chunk to_id & local_id not match!!");
    return;
  }
  
  bool is_remedy_chunk = msg_list->is_remedy_chunk;
  int from_drone_id = msg_list->from_drone_id;
  if (from_drone_id == drone_id_) return;
  ros::Time recv_time = ros::Time::now();
  uint32_t serial_size = ros::serialization::serializationLength(*msg_list);
  double volume = (double)(serial_size) / 1024;

  if (recv_log_.is_open()) {
      if(is_remedy_chunk) recv_log_ << volume << " " << ros::Time::now() << " Remedy" << std::endl;
      else                recv_log_ << volume << " " << ros::Time::now() << " Normal" << std::endl;
  } else {

  }

  //-----------------------------------------
  //restore in tmp buffer
  for(auto &LocalMapDatamsg : msg_list->LocalMapDataList)
  {
    //check if this chunk_idx has been received once
    if(!is_remedy_chunk && isIdsInclude(swarm_chunk_idx_list_[from_drone_id], LocalMapDatamsg.idx)) 
      continue;
    
    chunk_to_insert_buffer_[from_drone_id].emplace_back(LocalMapDatamsg);
    chunk_to_insert_buffer_[from_drone_id].back().recv_time = recv_time;
  }

  vector<int> recv_idx_list;
  if(is_remedy_chunk){
    recv_idx_list = { 1, (int)msg_list->LocalMapDataList[0].idx };
  }else{
    //To add the received ids
    auto& recv_chunk_buffer = chunk_to_insert_buffer_[from_drone_id];
    sort(recv_chunk_buffer.begin(), recv_chunk_buffer.end(),
          [](const swarm_msgs::LocalMapData& chunk1, const swarm_msgs::LocalMapData& chunk2) {
            return chunk1.idx < chunk2.idx;
          });

    recv_idx_list = { int(recv_chunk_buffer.front().idx) };
    int recv_last_idx = recv_idx_list[0];
    for (int j = 1; j < recv_chunk_buffer.size(); ++j) {
      if (recv_chunk_buffer[j].idx - recv_last_idx > 1) {
        recv_idx_list.emplace_back(recv_last_idx);
        recv_idx_list.emplace_back(recv_chunk_buffer[j].idx);
      }
      recv_last_idx = recv_chunk_buffer[j].idx;
    }
    recv_idx_list.emplace_back(recv_last_idx); 
  }
  vector<int> union_list;
  mergeChunkIds(recv_idx_list, swarm_chunk_idx_list_[from_drone_id], union_list); 
  swarm_chunk_idx_list_[from_drone_id] = union_list;

  // insert the chunks in tmp buffer into local map, some may delayed by ST_offset
  for (int i = 0; i < map_num_; ++i) {
    if(chunk_to_insert_buffer_[i].empty()) continue;
    if(is_real_experiment_ && (ST_offsets_.count(i) == 0)) continue;
    for (auto msg : chunk_to_insert_buffer_[i]) {

      insertLocalmap(msg, i);
    }
    chunk_to_insert_buffer_[i].clear();
  }
  return;
}


/* ---- Insertion ---- */

// Insert the chunk content into localmap
void MultiMapManager::insertLocalmap(const swarm_msgs::LocalMapData& msg, const int& from_id){ 
    double pack_time = msg.pack_time.toSec();
    if(is_real_experiment_){
      if(ST_offsets_.count(from_id) > 0){
        pack_time -= (ST_offsets_.find(from_id)->second.time_offset);
      }else return; 
    }

    std::vector<unsigned char> decompressed_data;
    decompressed_data = decompressData(msg.highres_map_attr, msg.originalSize);
    std::vector<int> unpacked_data;
    unpacked_data = unpackData(decompressed_data, msg.originalSize);
    Eigen::Vector3i min_highres_id, max_highres_id;
    map_->hashIdToGlobalIndex(msg.highres_adr_min, min_highres_id);
    map_->hashIdToGlobalIndex(msg.highres_adr_max, max_highres_id);

    insertLocalMap_hilbert(min_highres_id, max_highres_id, from_id, unpacked_data, pack_time);

    double unoffset_duration = ros::Time::now().toSec() - msg.pack_time.toSec();
    
    if (timediff_log_.is_open()) {
        timediff_log_ << 1000 * unoffset_duration << " " <<  msg.idx << " " << ros::Time::now() << std::endl;
    } else {
    }

    double recv_insert_duration = ros::Time::now().toSec() - msg.recv_time.toSec();
  
    
    if (recv_insert_diff_log_.is_open()) {
        recv_insert_diff_log_ << 1000 * recv_insert_duration << " " <<  msg.idx << " " << ros::Time::now() << std::endl;
    } else {
    }

    double offset_duration = ros::Time::now().toSec() - pack_time;
  
    if (timediff_offset_log_.is_open()) {
        timediff_offset_log_ << "from_id: " << from_id << ", " << 1000 * offset_duration << ", " << setiosflags(ios::fixed) << setprecision(4) << ros::Time::now().toSec() << std::endl;
    } else {

    }
}

//FOR INSERT: restore the hilbert template and update accordingly to the map
void MultiMapManager::insertLocalMap_hilbert(const Eigen::Vector3i& box_min, const Eigen::Vector3i& box_max, 
                                             const int& from_id, std::vector<int> attr, const double& pack_time){

  if(is_real_experiment_ && ST_offsets_.count(from_id) == 0) return;

  Eigen::Vector3i min_highres_id, max_highres_id;
  Eigen::Vector3i unit_3d = Eigen::Vector3i::Ones();
  min_highres_id = box_min;
  max_highres_id = box_max + unit_3d;

  uint32_t order = 10;
  HilbertCurve2D hc(order);
  
  // uint32_t encoded;
  pair<uint32_t, uint32_t> decoded;
  uint32_t temp_size = (max_highres_id[0] - min_highres_id[0]) * (max_highres_id[1] - min_highres_id[1]);
  vector<uint64_t> hilbert_2d_index;
  
  sort_2D_hilbert(min_highres_id[0], max_highres_id[0], min_highres_id[1], max_highres_id[1], hilbert_2d_index);

  map_->update_mutex_.lock();
  for(int i = min_highres_id[2]; i < max_highres_id[2] ; i++)
  {
    int outloop_idx = (i - min_highres_id[2]) * temp_size;
    for (int w = 0; w < temp_size; w++) {
      if(attr[outloop_idx + w] == SharedType::UNDEFINED_S) continue;
     
      decoded = hc.decode(hilbert_2d_index[w]);
      Eigen::Vector3i id_g_tmt(decoded.first + min_highres_id[0], decoded.second + min_highres_id[1], i);
      //id_g_tmt in the teammate frame
      Eigen::Vector3d pos = map_->globalIndexToPos(id_g_tmt);
      //Offset pos_tmt to ego frame
      OffsetTmtPt(pos, from_id);
     
      //pos in ego state
      if((map_->getRobotPos() - pos).norm() < 0.6) continue;
      Eigen::Vector3i id_g_ego = map_->posToGlobalIndex(pos);
      int temp_adr = map_->getHashIndexFromGlobalIndex(id_g_ego);
      if(pack_time < map_->stamp_buffer_[temp_adr]) continue;
      map_->sharedPointUpdate(id_g_ego, temp_adr, attr[outloop_idx + w], pack_time);
    }
  }
  map_->update_mutex_.unlock();

}

//FOR INSERT: sort hilbert
void MultiMapManager::sort_2D_hilbert(int x_min, int x_max, int y_min, int y_max, 
                                      std::vector<uint64_t>& hilbert_2d_index){
  uint32_t order = 10;
  HilbertCurve2D hc(order);

  uint32_t encoded;
  hilbert_2d_index.clear();
  for (int j = y_min; j < y_max; j++)
  {
    for (int k = x_min; k < x_max; k++)
    {
      encoded = hc.encode((k - x_min), (j - y_min));
      hilbert_2d_index.emplace_back(encoded);
    }
  }
  sort(hilbert_2d_index.begin(), hilbert_2d_index.end());
}

/* ---- ROGMap Update & Wrap ---- */
void MultiMapManager::addUpdatedVoxels(const Vec3f& pos,  const int& hash_id)
{
  updated_voxel_ids_buffer_.emplace_back(hash_id);
  Eigen::Vector3i id_g;
  map_->posToGlobalIndex(pos, id_g);
  localupdate_min_ = localupdate_min_.cwiseMin(id_g);
  localupdate_max_ = localupdate_max_.cwiseMax(id_g);
}

void MultiMapManager::packChunkTimerCallback(const ros::TimerEvent& e){
  
  if(updated_voxel_ids_buffer_.empty()) return;
  if((ros::Time::now() - last_packup_time_).toSec() < time_threshold_) return;

  map_->update_mutex_.lock();

  auto updated_voxel_ids = updated_voxel_ids_buffer_;
  updated_voxel_ids_buffer_.clear();
  Eigen::Vector3i min_bound(localupdate_min_);
  Eigen::Vector3i max_bound(localupdate_max_);
  localupdate_min_ << INT_MAX, INT_MAX, INT_MAX;
  localupdate_max_ << INT_MIN, INT_MIN, INT_MIN;

  map_->update_mutex_.unlock();



  for(auto hash_id : updated_voxel_ids)
    is_voxel_update_buffer_[hash_id] = 1;
  
  swarm_msgs::LocalMapData localmap_data_msg;

  localmap_data_msg.pack_time = last_packup_time_ = ros::Time::now(); //record the stamp starting compression

  std::vector<int> OriginalVoxeldata_highres;
  MapChunk chunk;

  OriginalVoxeldata_highres = getTwobitsStatus_highreshilbert_update(min_bound, max_bound, false);

  std::vector<unsigned char> bits_data_highres;
  bits_data_highres = packData(OriginalVoxeldata_highres);   
  std::vector<unsigned char> compressed_data_highres;
  compressed_data_highres = compressData(bits_data_highres);  
  
  localmap_data_msg.highres_adr_min = map_->getHashIndexFromGlobalIndex(min_bound);
  localmap_data_msg.highres_adr_max = map_->getHashIndexFromGlobalIndex(max_bound);
  localmap_data_msg.originalSize = OriginalVoxeldata_highres.size();
  localmap_data_msg.highres_map_attr = compressed_data_highres;
  localmap_data_msg.idx = self_map_chunks_.size() + 1;

  chunk.map_msg_data_ = localmap_data_msg;
  chunk.empty_ = false;

  self_map_chunks_.emplace_back(chunk);

  if (swarm_chunk_idx_list_[drone_id_].empty()) {
    swarm_chunk_idx_list_[drone_id_] = { 1, 1 };
  }
  swarm_chunk_idx_list_[drone_id_].back() =
  self_map_chunks_.back().map_msg_data_.idx;

}

void MultiMapManager::packAndSendRemedyChunk(const vector<int>& missed_idx_list, int to_drone_id)
{


  Eigen::Vector3i min_missed_bound(INT_MAX, INT_MAX, INT_MAX);
  Eigen::Vector3i max_missed_bound(INT_MIN, INT_MIN, INT_MIN);
  for (int i = 0; i < missed_idx_list.size(); i += 2) {
    for (int j = missed_idx_list[i]; j <= missed_idx_list[i + 1]; ++j) {
      Eigen::Vector3i min_highres_id, max_highres_id;

      auto chunk = &self_map_chunks_[j - 1].map_msg_data_;
      map_->hashIdToGlobalIndex(chunk->highres_adr_min, min_highres_id);
      map_->hashIdToGlobalIndex(chunk->highres_adr_max, max_highres_id);

      min_missed_bound = min_missed_bound.cwiseMin(min_highres_id);
      max_missed_bound = max_missed_bound.cwiseMax(max_highres_id);
    }
  }


  swarm_msgs::LocalMapData localmap_data_msg;
  localmap_data_msg.pack_time = ros::Time::now();
  std::vector<int> Voxeldata_highres;
  Voxeldata_highres = getTwobitsStatus_highreshilbert_update(min_missed_bound, max_missed_bound, true);
  std::vector<unsigned char> bits_data_highres;
  bits_data_highres = packData(Voxeldata_highres);   
  std::vector<unsigned char> compressed_data_highres;
  compressed_data_highres = compressData(bits_data_highres); 

  localmap_data_msg.highres_adr_min = map_->getHashIndexFromGlobalIndex(min_missed_bound);
  localmap_data_msg.highres_adr_max = map_->getHashIndexFromGlobalIndex(max_missed_bound);
  localmap_data_msg.originalSize = Voxeldata_highres.size();
  localmap_data_msg.highres_map_attr = compressed_data_highres;
  localmap_data_msg.idx = missed_idx_list.back();

  swarm_msgs::LocalMapList msg_list;
  msg_list.from_drone_id = drone_id_; 
  msg_list.to_drone_id = to_drone_id;
  msg_list.is_remedy_chunk = true;
  msg_list.LocalMapDataList.emplace_back(localmap_data_msg);

  uint32_t serial_size = ros::serialization::serializationLength(msg_list);
  double volume = (double)(serial_size) / 1024;

  if(transmit_log_.is_open()){
    transmit_log_ << volume << " " << ros::Time::now() << " Remedy" << std::endl;
  }
  localmaplist_pub_.publish(msg_list);
}

std::vector<int> MultiMapManager::getTwobitsStatus_highreshilbert_update(const Eigen::Vector3i& box_min, const Eigen::Vector3i& box_max, bool flag_update_all) {

  ros::Time test_start_time = ros::Time::now();
 
  Eigen::Vector3i unit_3d = Eigen::Vector3i::Ones();
  Eigen::Vector3i min_highres_id = box_min, max_highres_id = box_max + unit_3d;

  std::vector<int> OriginalVoxeldata;
  uint32_t order = 10;
  HilbertCurve2D hc(order);

  uint32_t localmap_voxel_num_ = (max_highres_id[2] - min_highres_id[2]) * (max_highres_id[1] - min_highres_id[1]) * (max_highres_id[0] - min_highres_id[0]);
  OriginalVoxeldata.resize(localmap_voxel_num_);
  uint32_t localmap_voxel_num_per_layer = (max_highres_id[1] - min_highres_id[1]) * (max_highres_id[0] - min_highres_id[0]);
  uint32_t size_k = max_highres_id[0] - min_highres_id[0];
  uint32_t temp_size = localmap_voxel_num_per_layer;

  vector<uint32_t> hilbert_2d_index(localmap_voxel_num_per_layer);
  vector<Eigen::Vector2i> Index(localmap_voxel_num_per_layer);

  for (int j = min_highres_id[1]; j < max_highres_id[1]; j++) 
  {
    for (int k = min_highres_id[0]; k < max_highres_id[0]; k++)
    {
      uint32_t encoded = hc.encode((k - min_highres_id[0]), (j - min_highres_id[1]));
      uint32_t temp_idx_ = (j - min_highres_id[1]) * size_k + (k - min_highres_id[0]);
      hilbert_2d_index[temp_idx_] = encoded;
      Index[temp_idx_] = Eigen::Vector2i(k, j);
    }
  }

  sortHilbertAndReorderAdrs(hilbert_2d_index, Index);

  // #pragma omp parallel for
  for(int i = min_highres_id[2]; i < max_highres_id[2]; i++) {
      uint32_t temp_begin = i - min_highres_id[2];

      for (int w = temp_begin  * temp_size; w < (temp_begin + 1) * temp_size; w++) {
    
          int tmp_ = w - temp_begin * temp_size;
          Eigen::Vector3i index_tmp(Index[tmp_](0), Index[tmp_](1), i);
          uint32_t temp_adr = map_->getHashIndexFromGlobalIndex(index_tmp);
          int voxel_status;
          
          if(flag_update_all){
            voxel_status = map_->getGridType(index_tmp) == GridType::OCCUPIED ? SharedType::OCCUPIED_S : 
                           map_->getGridType(index_tmp) == GridType::KNOWN_FREE ? SharedType::KNOWN_FREE_S : SharedType::UNDEFINED_S;
          }else{
            if(is_voxel_update_buffer_[temp_adr] == 0) {
              voxel_status = SharedType::UNDEFINED_S;
            }else{
              // 0 : Known_free | 1 : Occ | 2 : Unknown | 3 : Undefined
              voxel_status = map_->getGridType(index_tmp) == GridType::OCCUPIED ? SharedType::OCCUPIED_S : 
                             map_->getGridType(index_tmp) == GridType::KNOWN_FREE ? SharedType::KNOWN_FREE_S : SharedType::UNDEFINED_S;
              is_voxel_update_buffer_[temp_adr] = 0;
            }
          }
          OriginalVoxeldata[w] = voxel_status;
      }
  }

  ros::Time test_end_time = ros::Time::now();
  return OriginalVoxeldata;
}

//FOR WRAP: sort hilbert
void MultiMapManager::sortHilbertAndReorderAdrs(std::vector<uint32_t>& hilbert_idx, std::vector<Eigen::Vector2i>& voxel_adrs)
{
    size_t size = hilbert_idx.size();
    if (size != voxel_adrs.size()) {
        std::cerr << "Error: The two vectors must have the same size." << std::endl;
        return;
    }
    std::vector<size_t> indices(size);
    std::iota(indices.begin(), indices.end(), 0);
    std::sort(indices.begin(), indices.end(),
              [&hilbert_idx](size_t i1, size_t i2) { return hilbert_idx[i1] < hilbert_idx[i2]; });

    std::vector<Eigen::Vector2i> sorted_voxel_adrs(size);
    std::vector<uint32_t> sorted_hilbert_idx_(size);
    for (size_t i = 0; i < size; i++) {
        sorted_voxel_adrs[i] = voxel_adrs[indices[i]];
    }
    voxel_adrs.swap(sorted_voxel_adrs);
}

/* ---- Pack/Unpack & Compress/Decompress ---- */

std::vector<unsigned char> MultiMapManager::packData(const std::vector<int>& data) {
    std::vector<unsigned char> packedData;

    size_t packedDataSize = (data.size() + 3) / 4;  
    packedData.resize(packedDataSize, 0); 
    for(size_t i = 0; i < data.size(); ++i) {
        if(data[i] < 0 || data[i] > 3) {
            throw std::out_of_range("Data values should be between 0 and 3.");
        }
        size_t dataIndex = i / 4;
        size_t bitIndex = (i % 4) * 2;
        packedData[dataIndex] |= data[i] << bitIndex;  
    }
    return packedData;
  }

  std::vector<int> MultiMapManager::unpackData(const std::vector<unsigned char>& packedData, size_t originalSize) {
      std::vector<int> data;
      data.resize(originalSize);
      for(size_t i = 0; i < originalSize; ++i) {
          size_t dataIndex = i / 4;
          size_t bitIndex = (i % 4) * 2;
          data[i] = (packedData[dataIndex] >> bitIndex) & 0b11;  
      }
      return data;
  }

  std::vector<unsigned char> MultiMapManager::compressData(const std::vector<unsigned char>& packedData) {
      uLongf compressedDataSize = compressBound(packedData.size());
      std::vector<unsigned char> compressedData(compressedDataSize);
      if(compress2(compressedData.data(), &compressedDataSize, packedData.data(), packedData.size(), Z_BEST_COMPRESSION) != Z_OK) {
          throw std::runtime_error("Failed to compress data.");
      }
      compressedData.resize(compressedDataSize); 
      return compressedData;
  }

  std::vector<unsigned char> MultiMapManager::decompressData(const std::vector<unsigned char>& compressedData, uLongf originalSize) {
      std::vector<unsigned char> decompressedData(originalSize);
      if(uncompress(decompressedData.data(), &originalSize, compressedData.data(), compressedData.size()) != Z_OK) {
          throw std::runtime_error("Failed to decompress data.");
      }
      return decompressedData; 
  }


/* ---- Operations On IdxList ---- */

int MultiMapManager::getIdsListNum(const vector<int>& idx_list)
{
  if(idx_list.empty())
  {
    return 0;
  }else{
    int sum{0};
    for (int i = 0; i < idx_list.size(); i += 2) {
      sum += (idx_list[i + 1] - idx_list[i] + 1);
    }
    return sum;
  }
}

void MultiMapManager::findMissedChunkIds(  
    const vector<int>& self_idx_list, const vector<int>& other_idx_list, vector<int>& miss_ids) {
  if (other_idx_list.empty()) {
    miss_ids = self_idx_list;
    return;
  }

  vector<int> not_in_other;
  if (other_idx_list[0] > 1) { 
    not_in_other.emplace_back(1);
    not_in_other.emplace_back(other_idx_list[0] - 1);
  }
  for (int i = 1; i < other_idx_list.size(); i += 2) {
    not_in_other.emplace_back(other_idx_list[i] + 1);
    if (i == other_idx_list.size() - 1) {
      int infinite = std::numeric_limits<int>::max();
      not_in_other.emplace_back(infinite);
    } else {
      not_in_other.emplace_back(other_idx_list[i + 1] - 1);
    }
  }

  for (int i = 0; i < self_idx_list.size(); i += 2) {
    for (int j = 0; j < not_in_other.size(); j += 2) {
      int minr, maxr;
      if (findIntersect(self_idx_list[i], self_idx_list[i + 1], not_in_other[j],
              not_in_other[j + 1], minr, maxr)) {
        miss_ids.emplace_back(minr);
        miss_ids.emplace_back(maxr);
      }
    }
  }
}

bool MultiMapManager::findIntersect(
    const int& min1, const int& max1, const int& min2, const int max2, int& minr, int& maxr) {
  minr = max(min1, min2);
  maxr = min(max1, max2);
  if (minr <= maxr) return true;
  return false;
}

bool MultiMapManager::isIdsInclude(const vector<int>& idx_list, const int& id)
{
  for (int i = 0; i < idx_list.size(); i += 2) {
    if(id <= idx_list[i + 1] && id >= idx_list[i]){
      return true;
    }
  }
  return false;
}

void MultiMapManager::trimChunkIds(vector<int>& idx_list, const int& trim_id)
{
  if(idx_list[0] > trim_id) return;
  
  for(int i = 0; i < idx_list.size() - 1; i++)
  {
    if(trim_id >= idx_list[i + 1]) continue;
    if( i % 2 == 0 )
    {
      idx_list.assign(idx_list.cbegin() + i, idx_list.cend());
      idx_list[0] = trim_id + 1;
      return;
    }else{ 
      idx_list.assign(idx_list.cbegin() + i + 1, idx_list.cend());
      return;
    }
  }
  idx_list.clear(); return;
}

void MultiMapManager::mergeChunkIds(
    const vector<int>& list1, const vector<int>& list2, vector<int>& output) {

  if (list1.empty()) {
    output = list2;
    return;
  }

  output = list1;
  int tmp1, tmp2;
  for (int i = 0; i < list2.size(); i += 2) {

    bool intersect = false;
    for (int j = 0; j < output.size(); j += 2) {
      if (findIntersect(output[j], output[j + 1], list2[i], list2[i + 1], tmp1, tmp2)) {
        output[j] = min(output[j], list2[i]);
        output[j + 1] = max(output[j + 1], list2[i + 1]);
        intersect = true;
      }
    }
    if (!intersect) {  
      vector<int> tmp = { list2[i], list2[i + 1] };
      if (list2[i + 1] < output.front()) {
        output.insert(output.begin(), tmp.begin(), tmp.end());
      } else if (list2[i] > output.back()) {
        output.insert(output.end(), tmp.begin(), tmp.end());
      } else {
        for (auto iter = output.begin() + 1; iter != output.end(); iter += 2) {
          if (*iter < list2[i] && *(iter + 1) > list2[i + 1]) {
            output.insert(iter + 1, tmp.begin(), tmp.end());
            break;
          }
        }
      }
    }

    for (auto iter = output.begin() + 1; iter != output.end() - 1;) {
      if (*iter >= *(iter + 1) - 1) {
        iter = output.erase(iter);
        iter = output.erase(iter);
      } else {
        iter += 2;
      }
    }
  }
}
  

  template<typename arr_3>
  void MultiMapManager::SE3offset(arr_3& v, const Eigen::Matrix3d& R, const Eigen::Vector3d& t)
  {
      Eigen::Vector3d vec1(v[0],v[1],v[2]);
      Eigen::Vector3d vec2 = R * vec1 + t;
      v[0] = vec2(0); v[1] = vec2(1); v[2] = vec2(2);
  }

  void MultiMapManager::STOffsetCallback(const swarm_msgs::SpatialTemporalOffsetStatus::ConstPtr &msg)
  {  
      if(drone_id_ != msg->drone_id)
      {
        if(is_real_experiment_) ROS_WARN("Offset & OgmShare ID Not Matched!");
        return;
      }
      if(msg->header.stamp.toSec() < last_offset_stamp_)
      {
          ROS_ERROR("[MMM_node] Outdated GEs.");
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
              ROS_WARN("\033[32;1m[MMM_node] New teammate offset INSERTED.\033[0m");
          }
      }
  }

  bool MultiMapManager::OffsetTmtPt(Eigen::Vector3d& pos, int tmt_id) 
  {
    auto offset_iter = ST_offsets_.find(tmt_id);
    if(offset_iter != ST_offsets_.end()){
      SE3offset(pos, offset_iter->second.rot, offset_iter->second.trans);
    }else{
      if(is_real_experiment_){
        return false;
      }
    }
    return true;
  }
}


