/*
 * FAR Planner
 * Copyright (C) 2021 Fan Yang - All rights reserved
 * fanyang2@andrew.cmu.edu,   
 */



#include "far_planner/map_handler.h"

/***************************************************************************************/

void MapHandler::Init(const MapHandlerParams& params) {
    map_params_ = params;
    const int row_num = std::ceil(map_params_.grid_max_length / map_params_.cell_length);
    const int col_num = row_num; // 地图水平面上的网格数目
    int level_num = std::ceil(map_params_.grid_max_height / map_params_.cell_height); // 地图垂直面上的网格数目
    // 根据传感器测量范围来计算附近的网格数量
    neighbor_Lnum_ = std::ceil(map_params_.sensor_range * 2.0f / map_params_.cell_length) + 1; 
    neighbor_Hnum_ = 5; 
    if (level_num % 2 == 0) level_num ++;           // force to odd number, robot will be at center
    if (neighbor_Lnum_ % 2 == 0) neighbor_Lnum_ ++; // force to odd number

    // inlitialize grid 
    Eigen::Vector3i pointcloud_grid_size(row_num, col_num, level_num); // 最终的点云网格(体素)的尺寸 长宽高
    Eigen::Vector3d pointcloud_grid_origin(0,0,0);
    Eigen::Vector3d pointcloud_grid_resolution(map_params_.cell_length, map_params_.cell_length, map_params_.cell_height); // 点云网格分辨率
    PointCloudPtr cloud_ptr_tmp;
    // 这里的世界障碍物点云网格或者空闲点云网格 其实就是一个个的体素(长宽相同，高度不同)
    // 每个网格里面存放的就是点云数据
    world_obs_cloud_grid_ = std::make_unique<grid_ns::Grid<PointCloudPtr>>(
        pointcloud_grid_size, cloud_ptr_tmp, pointcloud_grid_origin, pointcloud_grid_resolution, 3);

    world_free_cloud_grid_ = std::make_unique<grid_ns::Grid<PointCloudPtr>>(
        pointcloud_grid_size, cloud_ptr_tmp, pointcloud_grid_origin, pointcloud_grid_resolution, 3);

    const int n_cell  = world_obs_cloud_grid_->GetCellNumber();
    for (int i = 0; i < n_cell; i++) { // 初始化网格
        world_obs_cloud_grid_->GetCell(i) = PointCloudPtr(new PointCloud);
        world_free_cloud_grid_->GetCell(i) = PointCloudPtr(new PointCloud);
    }
    global_visited_induces_.resize(n_cell), util_remove_check_list_.resize(n_cell);
    util_obs_modified_list_.resize(n_cell), util_free_modified_list_.resize(n_cell);
    std::fill(global_visited_induces_.begin(), global_visited_induces_.end(), 0);
    std::fill(util_obs_modified_list_.begin(), util_obs_modified_list_.end(), 0);
    std::fill(util_free_modified_list_.begin(), util_free_modified_list_.end(), 0);
    std::fill(util_remove_check_list_.begin(), util_remove_check_list_.end(), 0);

    // init terrain height map 初始化地形高度地图
    // 先计算地形高度网格地图的尺寸(用传感器测量范围除以机器人的直径)，即得到多少个以机器人直径为长度的格子
    int height_dim = std::ceil((map_params_.sensor_range + map_params_.cell_length) * 2.0f / FARUtil::robot_dim);
    if (height_dim % 2 == 0) height_dim ++; // 保持为奇数
    Eigen::Vector3i height_grid_size(height_dim, height_dim, 1);
    Eigen::Vector3d height_grid_origin(0,0,0);
    Eigen::Vector3d height_grid_resolution(FARUtil::robot_dim, FARUtil::robot_dim, FARUtil::kLeafSize); // voxel dim
    std::vector<float> temp_vec;
    terrain_height_grid_ = std::make_unique<grid_ns::Grid<std::vector<float>>>(
        height_grid_size, temp_vec, height_grid_origin, height_grid_resolution, 3);
    
    const int n_terrain_cell = terrain_height_grid_->GetCellNumber();
    terrain_grid_occupy_list_.resize(n_terrain_cell), terrain_grid_traverse_list_.resize(n_terrain_cell);
    std::fill(terrain_grid_occupy_list_.begin(), terrain_grid_occupy_list_.end(), 0);
    std::fill(terrain_grid_traverse_list_.begin(), terrain_grid_traverse_list_.end(), 0);

    INFLATE_N = 1;
    flat_terrain_cloud_    = PointCloudPtr(new pcl::PointCloud<PCLPoint>()); // 平坦地形点云
    kdtree_terrain_clould_ = PointKdTreePtr(new pcl::KdTreeFLANN<PCLPoint>()); // kd树地形点云
    kdtree_terrain_clould_->setSortedResults(false); // 配置最近邻搜索结果不用进行排序
}

void MapHandler::ResetGripMapCloud() {
    const int n_cell = world_obs_cloud_grid_->GetCellNumber();
    for (int i=0; i<n_cell; i++) {
        world_obs_cloud_grid_->GetCell(i)->clear();
        world_free_cloud_grid_->GetCell(i)->clear();
    }
    std::fill(global_visited_induces_.begin(),     global_visited_induces_.end(),     0);
    std::fill(util_obs_modified_list_.begin(),     util_obs_modified_list_.end(),     0);
    std::fill(util_free_modified_list_.begin(),    util_free_modified_list_.end(),    0);
    std::fill(util_remove_check_list_.begin(),     util_remove_check_list_.end(),     0);
    std::fill(terrain_grid_occupy_list_.begin(),   terrain_grid_occupy_list_.end(),   0);
    std::fill(terrain_grid_traverse_list_.begin(), terrain_grid_traverse_list_.end(), 0);
}

void MapHandler::ClearObsCellThroughPosition(const Point3D& point) {
    const Eigen::Vector3i psub = world_obs_cloud_grid_->Pos2Sub(point.x, point.y, point.z);
    std::vector<Eigen::Vector3i> ray_subs;
    world_obs_cloud_grid_->RayTraceSubs(robot_cell_sub_, psub, ray_subs);
    const int H = neighbor_Hnum_ / 2;
    for (const auto& sub : ray_subs) {
        for (int k = -H; k <= H; k++) {
            Eigen::Vector3i csub = sub;
            csub.z() += k;
            const int ind = world_obs_cloud_grid_->Sub2Ind(csub);
            if (!world_obs_cloud_grid_->InRange(csub) || neighbor_obs_indices_.find(ind) == neighbor_obs_indices_.end()) continue; 
            world_obs_cloud_grid_->GetCell(ind)->clear();
            if (world_free_cloud_grid_->GetCell(ind)->empty()) {
                global_visited_induces_[ind] = 0;
            }
        }
    }
}

void MapHandler::GetCloudOfPoint(const Point3D& center, 
                                 const PointCloudPtr& cloudOut,
                                 const CloudType& type,
                                 const bool& is_large) 
{
    cloudOut->clear();
    const Eigen::Vector3i sub = world_obs_cloud_grid_->Pos2Sub(center.x, center.y, center.z);
    const int N = is_large ? 1 : 0;
    const int H = neighbor_Hnum_ / 2;
    for (int i = -N; i <= N; i++) {
        for (int j = -N; j <= N; j++) {
            for (int k = -H; k <= H; k++) {
                Eigen::Vector3i csub = sub;
                csub.x() += i, csub.y() += j, csub.z() += k;
                if (!world_obs_cloud_grid_->InRange(csub)) continue;
                if (type == CloudType::FREE_CLOUD) {
                    *cloudOut += *(world_free_cloud_grid_->GetCell(csub));
                } else if (type == CloudType::OBS_CLOUD) {
                    *cloudOut += *(world_obs_cloud_grid_->GetCell(csub));
                } else {
                    if (FARUtil::IsDebug) std::cout << "MH: Assigned cloud type invalid." << std::endl;
                    return;
                }
            }
        }
    }
}

// 以输入的位姿为地图网格的中心，设置两个世界地图网格的origin(立方体左下角所处世界坐标)
// 从这里可以看出来，网格坐标的x,y,z方向跟map坐标系是保持一致的
void MapHandler::SetMapOrigin(const Point3D& ori_robot_pos) {
    Point3D map_origin;
    const Eigen::Vector3i dim = world_obs_cloud_grid_->GetSize(); // 获取每个维度的网格数量
    // 计算左下角网格的物理位置，也就是origin
    map_origin.x = ori_robot_pos.x - (map_params_.cell_length * dim.x()) / 2.0f;
    map_origin.y = ori_robot_pos.y - (map_params_.cell_length * dim.y()) / 2.0f;
    map_origin.z = ori_robot_pos.z - (map_params_.cell_height * dim.z()) / 2.0f - FARUtil::vehicle_height; // From Ground Level
    // 这里z值减去vehicle_height是因为机器人是以从地面高度作为地图中心的，而不是雷达高度
    Eigen::Vector3d pointcloud_grid_origin(map_origin.x, map_origin.y, map_origin.z);
    world_obs_cloud_grid_->SetOrigin(pointcloud_grid_origin);
    world_free_cloud_grid_->SetOrigin(pointcloud_grid_origin);
    is_init_ = true;
    if (FARUtil::IsDebug) std::cout << "MH: Global Cloud Map Grid Initialized." << std::endl;
}

void MapHandler::UpdateRobotPosition(const Point3D& odom_pos) {
    if (!is_init_) this->SetMapOrigin(odom_pos); // 利用第一帧机器人位姿数据设置地图原点
    // 更新机器人当前网格位置
    robot_cell_sub_ = world_obs_cloud_grid_->Pos2Sub(Eigen::Vector3d(odom_pos.x, odom_pos.y, odom_pos.z));
    // Get neighbor indices
    neighbor_free_indices_.clear(), neighbor_obs_indices_.clear();
    const int N = neighbor_Lnum_ / 2;
    const int H = neighbor_Hnum_ / 2;
    Eigen::Vector3i neighbor_sub;
    // 根据机器人当前网格位置索引，计算其周围邻域的网格索引
    for (int i = -N; i <= N; i++) {
        neighbor_sub.x() = robot_cell_sub_.x() + i;
        for (int j = -N; j <= N; j++) {
            neighbor_sub.y() = robot_cell_sub_.y() + j;
            // additional terrain points -1 // TODO: WHY THIS？
            neighbor_sub.z() = robot_cell_sub_.z() - H - 1;
            if (world_obs_cloud_grid_->InRange(neighbor_sub)) {
                int ind = world_obs_cloud_grid_->Sub2Ind(neighbor_sub);
                neighbor_free_indices_.insert(ind);
            }
            for (int k =-H; k <= H; k++) {
                neighbor_sub.z() = robot_cell_sub_.z() + k;
                if (world_obs_cloud_grid_->InRange(neighbor_sub)) {
                    int ind = world_obs_cloud_grid_->Sub2Ind(neighbor_sub);
                    neighbor_obs_indices_.insert(ind), neighbor_free_indices_.insert(ind);
                }
            }
        }
    }
    this->SetTerrainHeightGridOrigin(odom_pos); // 实时更新地形高度地图的origin位置
}

// 设置地形高度网格地图的原点，使得当前机器人处于网格地图最中心
void MapHandler::SetTerrainHeightGridOrigin(const Point3D& robot_pos) {
    // update terrain height grid center
    const Eigen::Vector3d res = terrain_height_grid_->GetResolution();  // 网格分辨率 [robot_dim, robot_dim, leaf_size]
    const Eigen::Vector3i dim = terrain_height_grid_->GetSize();        // 每个维度的网格数量
    Eigen::Vector3d grid_origin;
    grid_origin.x() = robot_pos.x - (res.x() * dim.x()) / 2.0f;
    grid_origin.y() = robot_pos.y - (res.y() * dim.y()) / 2.0f;
    grid_origin.z() = 0.0f        - (res.z() * dim.z()) / 2.0f;
    terrain_height_grid_->SetOrigin(grid_origin);
}

void MapHandler::GetSurroundObsCloud(const PointCloudPtr& obsCloudOut) {
    if (!is_init_) return;
    obsCloudOut->clear();
    for (const auto& neighbor_ind : neighbor_obs_indices_) {
        if (world_obs_cloud_grid_->GetCell(neighbor_ind)->empty()) continue;
        *obsCloudOut += *(world_obs_cloud_grid_->GetCell(neighbor_ind));
    }
}

void MapHandler::GetSurroundFreeCloud(const PointCloudPtr& freeCloudOut) {
    if (!is_init_) return;
    freeCloudOut->clear();
    for (const auto& neighbor_ind : neighbor_free_indices_) {
        if (world_free_cloud_grid_->GetCell(neighbor_ind)->empty()) continue;
        *freeCloudOut += *(world_free_cloud_grid_->GetCell(neighbor_ind));
    }
}

// 更新障碍物点云网格(提取obsCloudInOut中处于机器人附近的点)
void MapHandler::UpdateObsCloudGrid(const PointCloudPtr& obsCloudInOut) {
    if (!is_init_ || obsCloudInOut->empty()) return;
    // 先将网格索引列表的元素都置零
    std::fill(util_obs_modified_list_.begin(), util_obs_modified_list_.end(), 0);
    PointCloudPtr obs_valid_ptr(new pcl::PointCloud<PCLPoint>());
    for (const auto& point : obsCloudInOut->points) {
        // 计算障碍物点的在world_obs_cloud_grid_中的网格索引
        Eigen::Vector3i sub = world_obs_cloud_grid_->Pos2Sub(Eigen::Vector3d(point.x, point.y, point.z));
        if (!world_obs_cloud_grid_->InRange(sub)) continue;
        const int ind = world_obs_cloud_grid_->Sub2Ind(sub); // 转换为一维索引
        // 如果该索引在机器人周围的索引列表范围内(这个索引列表是实时更新的)
        if (neighbor_obs_indices_.find(ind) != neighbor_obs_indices_.end()) {
            world_obs_cloud_grid_->GetCell(ind)->points.push_back(point); // 将该有效点添加进网格的障碍物点云中
            obs_valid_ptr->points.push_back(point);
            util_obs_modified_list_[ind] = 1;   // obs网格索引
            global_visited_induces_[ind] = 1;   // 更新全局访问索引
        }
    }
    *obsCloudInOut = *obs_valid_ptr;
    // Filter Modified Ceils 对刚添加的障碍物网格点云进行滤波
    for (int i = 0; i < world_obs_cloud_grid_->GetCellNumber(); ++i) {
      if (util_obs_modified_list_[i] == 1) FARUtil::FilterCloud(world_obs_cloud_grid_->GetCell(i), FARUtil::kLeafSize);
    }
}

// 更新free点云网格 步骤同上
void MapHandler::UpdateFreeCloudGrid(const PointCloudPtr& freeCloudIn){
    if (!is_init_ || freeCloudIn->empty()) return;
    std::fill(util_free_modified_list_.begin(), util_free_modified_list_.end(), 0);
    for (const auto& point : freeCloudIn->points) {
        Eigen::Vector3i sub = world_free_cloud_grid_->Pos2Sub(Eigen::Vector3d(point.x, point.y, point.z));
        if (!world_free_cloud_grid_->InRange(sub)) continue;
        const int ind = world_free_cloud_grid_->Sub2Ind(sub);
        world_free_cloud_grid_->GetCell(ind)->points.push_back(point);
        util_free_modified_list_[ind] = 1;
        global_visited_induces_[ind]  = 1;  // 更新全局访问索引
    }
    // Filter Modified Ceils
    for (int i = 0; i < world_free_cloud_grid_->GetCellNumber(); ++i) {
      if (util_free_modified_list_[i] == 1) FARUtil::FilterCloud(world_free_cloud_grid_->GetCell(i), FARUtil::kLeafSize);
    }
}

/**
 * @brief 查询给定点所处的地形高度信息
 * 
 * @param p 给定三维点
 * @param is_matched 查询点是否处于可通行网格中
 * @param is_search 是否通过搜索查找附近的最近邻网格地形高度
 * @return float 返回查询到的地形高度值，未查询到则返回原z值
 */
float MapHandler::TerrainHeightOfPoint(const Point3D& p, bool& is_matched, const bool& is_search) {
    is_matched = false;
    const Eigen::Vector3i sub = terrain_height_grid_->Pos2Sub(Eigen::Vector3d(p.x, p.y, 0.0f));
    if (terrain_height_grid_->InRange(sub)) {
        const int ind = terrain_height_grid_->Sub2Ind(sub);
        if (terrain_grid_traverse_list_[ind] != 0) { // 查询点处于地形网格可通行列表中，直接返回地形网格的高度值
            is_matched = true;
            return terrain_height_grid_->GetCell(ind)[0];
        }
    }
    if (is_search) {
        float matched_dist_squre;
        const float terrain_h = NearestHeightOfPoint(p, matched_dist_squre); // 搜索最近邻的地形高度数据
        return terrain_h;
    }
    return p.z; 
}

float MapHandler::NearestTerrainHeightofNavPoint(const Point3D& point, bool& is_associated) {
    const float p_th = point.z-FARUtil::vehicle_height;
    const Eigen::Vector3i ori_sub = world_free_cloud_grid_->Pos2Sub(Eigen::Vector3d(point.x, point.y, p_th));
    is_associated = false;
    if (world_free_cloud_grid_->InRange(ori_sub)) {
        // downward seach
        bool is_dw_associated = false;
        Eigen::Vector3i dw_near_sub = ori_sub;
        float dw_terrain_h = p_th;
        while (world_free_cloud_grid_->InRange(dw_near_sub)) {
            if (!world_free_cloud_grid_->GetCell(dw_near_sub)->empty()) {
                int counter = 0;
                dw_terrain_h = 0.0f;
                for (const auto& pcl_p : world_free_cloud_grid_->GetCell(dw_near_sub)->points) {
                    dw_terrain_h += pcl_p.z, counter ++;
                }
                dw_terrain_h /= (float)counter;
                is_dw_associated = true;
                break;
            }
            dw_near_sub.z() --;
        }
        // upward search
        bool is_up_associated = false;
        Eigen::Vector3i up_near_sub = ori_sub;
        float up_terrain_h = p_th;
        while (world_free_cloud_grid_->InRange(up_near_sub)) {
            if (!world_free_cloud_grid_->GetCell(up_near_sub)->empty()) {
                int counter = 0;
                up_terrain_h = 0.0f;
                for (const auto& pcl_p : world_free_cloud_grid_->GetCell(up_near_sub)->points) {
                    up_terrain_h += pcl_p.z, counter ++;
                }
                up_terrain_h /= (float)counter;
                is_up_associated = true;
                break;
            }
            up_near_sub.z() ++;
            
        }
        is_associated = (is_up_associated || is_dw_associated) ? true : false;
        if (is_up_associated && is_dw_associated) { // compare nearest
            if (up_near_sub.z() - ori_sub.z() < ori_sub.z() - dw_near_sub.z()) return up_terrain_h;
            else return dw_terrain_h;
        } else if (is_up_associated) return up_terrain_h;
        else return dw_terrain_h;
    }
    return p_th;
}


bool MapHandler::IsNavPointOnTerrainNeighbor(const Point3D& point, const bool& is_extend) {
    const float h = point.z - FARUtil::vehicle_height; 
    const Eigen::Vector3i sub = world_obs_cloud_grid_->Pos2Sub(Eigen::Vector3d(point.x, point.y, h));
    if (!world_obs_cloud_grid_->InRange(sub)) return false;
    const int ind = world_obs_cloud_grid_->Sub2Ind(sub);
    if (is_extend && extend_obs_indices_.find(ind) != extend_obs_indices_.end()) {
        return true;
    }
    if (!is_extend && neighbor_obs_indices_.find(ind) != neighbor_obs_indices_.end()) {
        return true;
    }
    return false;
}


void MapHandler::AdjustNodesHeight(const NodePtrStack& nodes) {
    if (nodes.empty()) return;
    for (const auto& node_ptr : nodes) {
        if (!node_ptr->is_active || node_ptr->is_boundary || FARUtil::IsFreeNavNode(node_ptr) || FARUtil::IsOutsideGoal(node_ptr) || !FARUtil::IsPointInLocalRange(node_ptr->position, true)) {
            continue;
        } 
        bool is_match = false;
        float terrain_h = TerrainHeightOfPoint(node_ptr->position, is_match, false);
        if (is_match) {
            terrain_h += FARUtil::vehicle_height;
            if (node_ptr->pos_filter_vec.empty()) {
                node_ptr->position.z = terrain_h;
            } else {
                node_ptr->pos_filter_vec.back().z = terrain_h; // assign to position filter
                node_ptr->position.z = FARUtil::AveragePoints(node_ptr->pos_filter_vec).z;
            }
        }
    }
}

void MapHandler::AdjustCTNodeHeight(const CTNodeStack& ctnodes) {
    if (ctnodes.empty()) return;
    const float H_MAX = FARUtil::robot_pos.z + FARUtil::kTolerZ;
    const float H_MIN = FARUtil::robot_pos.z - FARUtil::kTolerZ;
    for (auto& ctnode_ptr : ctnodes) {
        float min_th, max_th;
        // 查询节点位置附近范围内的最近邻点(地形点)
        NearestHeightOfRadius(ctnode_ptr->position, FARUtil::kMatchDist, min_th, max_th, ctnode_ptr->is_ground_associate);
        if (ctnode_ptr->is_ground_associate) { // 在该范围内搜索到了最近邻点，节点与地面有关
            ctnode_ptr->position.z = min_th + FARUtil::vehicle_height;
            ctnode_ptr->position.z = std::max(std::min(ctnode_ptr->position.z, H_MAX), H_MIN); // 限制范围
        } else { // 未搜索到最近邻点
            ctnode_ptr->position.z = TerrainHeightOfPoint(ctnode_ptr->position, ctnode_ptr->is_ground_associate, true);
            ctnode_ptr->position.z += FARUtil::vehicle_height;
            ctnode_ptr->position.z = std::max(std::min(ctnode_ptr->position.z, H_MAX), H_MIN);
        }
    }
}

void MapHandler::ObsNeighborCloudWithTerrain(std::unordered_set<int>& neighbor_obs, std::unordered_set<int>& extend_terrain_obs) {
    std::unordered_set<int> neighbor_copy = neighbor_obs;
    neighbor_obs.clear();
    const float R = map_params_.cell_length * 0.7071f; // sqrt(2)/2
    for (const auto& idx : neighbor_copy) {
        const Point3D pos = Point3D(world_obs_cloud_grid_->Ind2Pos(idx)); // 计算该索引物理位置
        bool inRange = false;
        float minH, maxH;
        NearestHeightOfRadius(pos, R, minH, maxH, inRange);
        if (inRange && pos.z + map_params_.cell_height > minH &&
                       pos.z - map_params_.cell_height < maxH + FARUtil::kTolerZ) // use map_params_.cell_height/2.0 as a tolerance margin
        {
            neighbor_obs.insert(idx);
        }
    }
    extend_terrain_obs.clear(); // assign extended terrain obs indices
    const std::vector<int> inflate_vec{-1, 0};
    for (const int& idx : neighbor_obs) {
        const Eigen::Vector3i csub = world_obs_cloud_grid_->Ind2Sub(idx);
        for (const int& plus : inflate_vec) {
            Eigen::Vector3i sub = csub; 
            sub.z() += plus;
            if (!world_obs_cloud_grid_->InRange(sub)) continue;
            const int plus_idx = world_obs_cloud_grid_->Sub2Ind(sub);
            extend_terrain_obs.insert(plus_idx);
        }
    }
}

void MapHandler::UpdateTerrainHeightGrid(const PointCloudPtr& freeCloudIn, const PointCloudPtr& terrainHeightOut) {
    if (freeCloudIn->empty()) return;
    PointCloudPtr copy_free_ptr(new pcl::PointCloud<PCLPoint>());
    pcl::copyPointCloud(*freeCloudIn, *copy_free_ptr); // 拷贝点云
    FARUtil::FilterCloud(copy_free_ptr, terrain_height_grid_->GetResolution()); // [robot_dim,robot_dim, leaf_size]
    std::fill(terrain_grid_occupy_list_.begin(), terrain_grid_occupy_list_.end(), 0); // 先置0，每轮都重新计算
    // 这里的方法类似于terrian analysis中的高程计算，在一个网格中放入多个点的z值
    for (const auto& point : copy_free_ptr->points) {
        // 先计算该free点在地形高度网格中的对应索引
        Eigen::Vector3i csub = terrain_height_grid_->Pos2Sub(Eigen::Vector3d(point.x, point.y, 0.0f));
        std::vector<Eigen::Vector3i> subs;
        this->Expansion2D(csub, subs, INFLATE_N); // 扩展当前索引(膨胀)
        for (const auto& sub : subs) {
            if (!terrain_height_grid_->InRange(sub)) continue;
            const int ind = terrain_height_grid_->Sub2Ind(sub);
            if (terrain_grid_occupy_list_[ind] == 0) { // 首次访问该网格
                terrain_height_grid_->GetCell(ind).resize(1);
                terrain_height_grid_->GetCell(ind)[0] = point.z; // 高程值，并非传统的z坐标值
            } else {
                terrain_height_grid_->GetCell(ind).push_back(point.z);
            }
            terrain_grid_occupy_list_[ind] = 1; // 只要赋值为1，就表示该index在本轮更新地形高程网格过程中有free point
        }
    }
    this->TraversableAnalysis(terrainHeightOut); // 可通行性分析
    if (terrainHeightOut->empty()) { // set terrain height kdtree
        FARUtil::ClearKdTree(flat_terrain_cloud_, kdtree_terrain_clould_);
    } else {
        this->AssignFlatTerrainCloud(terrainHeightOut, flat_terrain_cloud_);
        kdtree_terrain_clould_->setInputCloud(flat_terrain_cloud_);
    }
    // update surrounding obs cloud grid indices based on terrain
    this->ObsNeighborCloudWithTerrain(neighbor_obs_indices_, extend_obs_indices_); // neighbor_obs_indices_ 在这里清空了
}

// 可通行性分析，生成可通行稀疏点云
void MapHandler::TraversableAnalysis(const PointCloudPtr& terrainHeightOut) {
    // 获取机器人当前位置在地形高程网格中索引
    const Eigen::Vector3i robot_sub = terrain_height_grid_->Pos2Sub(Eigen::Vector3d(FARUtil::robot_pos.x, 
                                                                                    FARUtil::robot_pos.y, 0.0f));
    terrainHeightOut->clear();
    if (!terrain_height_grid_->InRange(robot_sub)) { // 机器人超出网格边界
        std::cout << "MH: terrain height analysis error: robot position is not in range" << std::endl;
        return;
    }
    const float H_THRED = map_params_.height_voxel_dim; // voxel_dim * 2.0 高度变化阈值
    std::fill(terrain_grid_traverse_list_.begin(), terrain_grid_traverse_list_.end(), 0); // 可通行网格列表，先置0
    // Lambda Function
    auto IsTraversableNeighbor = [&] (const int& cur_id, const int& ref_id) {
        if (terrain_grid_occupy_list_[ref_id] == 0) return false; // 已经添加过了
        const float cur_h = terrain_height_grid_->GetCell(cur_id)[0];
        float ref_h = 0.0f;
        int counter = 0;
        for (const auto& e : terrain_height_grid_->GetCell(ref_id)) {
            if (abs(e - cur_h) > H_THRED) continue;
            ref_h += e, counter ++;
        }
        if (counter > 0) {
            terrain_height_grid_->GetCell(ref_id).resize(1);
            terrain_height_grid_->GetCell(ref_id)[0] = ref_h / (float)counter;
            return true;
        }
        return false;
    };
    // 添加索引到可通行网格列表中
    auto AddTraversePoint = [&] (const int& idx) {
        Eigen::Vector3d cpos = terrain_height_grid_->Ind2Pos(idx);
        cpos.z() = terrain_height_grid_->GetCell(idx)[0]; // 为什么是添加网格第一个高程值？？
        const PCLPoint p = FARUtil::Point3DToPCLPoint(Point3D(cpos));
        terrainHeightOut->points.push_back(p);
        terrain_grid_traverse_list_[idx] = 1;
    };

    const int robot_idx = terrain_height_grid_->Sub2Ind(robot_sub); // 当前机器人位置所在地形高度网格中的索引
    const std::array<int, 4> dx = {-1, 0, 1, 0};
    const std::array<int, 4> dy = { 0, 1, 0,-1};
    std::deque<int> q;
    bool is_robot_terrain_init = false;
    std::unordered_set<int> visited_set;
    q.push_back(robot_idx), visited_set.insert(robot_idx);
    // BFS
    while (!q.empty()) {
        const int cur_id = q.front();
        q.pop_front();
        // 一般机器人位置所在的网格以及周围一些网格都是没有地形数据的(由于diablo激光雷达的安装方式，如果能打到地面的话会有一米的盲区)
        if (terrain_grid_occupy_list_[cur_id] != 0) { // 有地形free数据(本轮free point 所在的网格)
            if (!is_robot_terrain_init) {
                float avg_h = 0.0f;
                int counter = 0;
                for (const auto& e : terrain_height_grid_->GetCell(cur_id)) { // 一个网格里面可能有很多高程值
                    if (abs(e - FARUtil::robot_pos.z + FARUtil::vehicle_height) > H_THRED) continue;
                    avg_h += e, counter ++;
                }
                if (counter > 0) {
                    avg_h /= (float)counter; // 没有对该值进行检查，这里默认是机器人周围就是地面！！！ //TODO
                    terrain_height_grid_->GetCell(cur_id).resize(1);
                    terrain_height_grid_->GetCell(cur_id)[0] = avg_h;
                    AddTraversePoint(cur_id);
                    is_robot_terrain_init = true; // init terrain height map current robot height
                    q.clear(); // 初始化成功，清除队列，以该索引为中心重新进行传播
                }
            } else { // 初始化成功直接添加
                AddTraversePoint(cur_id);
            }
        } else if (is_robot_terrain_init) { // 没有free地形数据，且地形已经初始化了，那么就不需要往该网格的四个邻域方向扩展了
            continue;
        }
        // 两种情况会继续扩展: 1. 地形高度没有初始化; 2. 有free 地形数据
        const Eigen::Vector3i csub = terrain_height_grid_->Ind2Sub(cur_id);
        for (int i=0; i<4; i++) { // 四邻域
            Eigen::Vector3i ref_sub = csub;
            ref_sub.x() += dx[i], ref_sub.y() += dy[i];
            if (!terrain_height_grid_->InRange(ref_sub)) continue;
            const int ref_id = terrain_height_grid_->Sub2Ind(ref_sub);
            // 两种情况下扩展搜索网格节点: 1. 网格没有被遍历过; 2. 地形高度还未被初始化 或者 下个节点是可通行的
            // 地形高度还未被初始化(!is_robot_terrain_init) 这种情况是刚开始搜索时，机器人网格附近都没有数据
            // 地形高度初始化后，则根据后面IsTraversableNeighbor 来进行判断是否要将邻域节点加入到队列中进行传播
            if (!visited_set.count(ref_id) && (!is_robot_terrain_init || IsTraversableNeighbor(cur_id, ref_id))) {
                q.push_back(ref_id);
                visited_set.insert(ref_id);
            }
        }
    }
}

// 提取相邻障碍物点云网格的中心点
void MapHandler::GetNeighborCeilsCenters(PointStack& neighbor_centers) {
    if (!is_init_) return;
    neighbor_centers.clear();
    for (const auto& ind : neighbor_obs_indices_) {
        if (global_visited_induces_[ind] == 0) continue;
        Point3D center_p(world_obs_cloud_grid_->Ind2Pos(ind));
        neighbor_centers.push_back(center_p);
    }
}

// 提取所有障碍物点云网格的中心点
void MapHandler::GetOccupancyCeilsCenters(PointStack& occupancy_centers) {
    if (!is_init_) return;
    occupancy_centers.clear();
    const int N = world_obs_cloud_grid_->GetCellNumber();
    for (int ind=0; ind<N; ind++) {
        if (global_visited_induces_[ind] == 0) continue;
        Point3D center_p(world_obs_cloud_grid_->Ind2Pos(ind));
        occupancy_centers.push_back(center_p);
    }
}

void MapHandler::RemoveObsCloudFromGrid(const PointCloudPtr& obsCloud) {
    std::fill(util_remove_check_list_.begin(), util_remove_check_list_.end(), 0);
    for (const auto& point : obsCloud->points) {
        Eigen::Vector3i sub = world_obs_cloud_grid_->Pos2Sub(Eigen::Vector3d(point.x, point.y, point.z));
        if (!world_free_cloud_grid_->InRange(sub)) continue;
        const int ind = world_free_cloud_grid_->Sub2Ind(sub);
        util_remove_check_list_[ind] = 1;
    }
    for (const auto& ind : neighbor_obs_indices_) {
        if (util_remove_check_list_[ind] == 1 && global_visited_induces_[ind] == 1) {
            FARUtil::RemoveOverlapCloud(world_obs_cloud_grid_->GetCell(ind), obsCloud);
        }
    }
}

