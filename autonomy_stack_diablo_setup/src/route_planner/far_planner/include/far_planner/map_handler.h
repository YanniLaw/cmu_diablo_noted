#ifndef MAP_HANDLER_H
#define MAP_HANDLER_H

#include "utility.h"

enum CloudType {
    FREE_CLOUD = 0,
    OBS_CLOUD  = 1
};

struct MapHandlerParams {
    MapHandlerParams() = default;
    float sensor_range;         // 传感器测量范围
    float floor_height;         // 楼层高度???
    float cell_length;          // 每个格子的长度 m
    float cell_height;          // 每个格子的高度 m
    float grid_max_length;      // 地图网格最大长度 m
    float grid_max_height;      // 地图网格最大高度 m
    // local terrain height map
    float height_voxel_dim;     // 
};

class MapHandler {

public:
    MapHandler() = default;
    ~MapHandler() = default;

    void Init(const MapHandlerParams& params);
    void SetMapOrigin(const Point3D& robot_pos);

    void UpdateRobotPosition(const Point3D& odom_pos);

    void AdjustNodesHeight(const NodePtrStack& nodes);

    void AdjustCTNodeHeight(const CTNodeStack& ctnodes);

    static float TerrainHeightOfPoint(const Point3D& p, 
                                      bool& is_matched, 
                                      const bool& is_search);

    static bool IsNavPointOnTerrainNeighbor(const Point3D& p, const bool& is_extend);

    static float NearestTerrainHeightofNavPoint(const Point3D& point, bool& is_associated);

    /**
     * @brief Calculate the terrain height of a given point and radius around it
     * 利用 KD-Tree 进行快速邻域搜索，计算一个点及其周围区域的最小高程值、最大高程值和平均高程值，并返回平均值。如果找不到有效地形点，则返回输入点的高度值
     * @param p A given position 给定查询点
     * @param radius The radius distance around the given posiitn p 搜索范围，领域大小
     * @param minH[out] The mininal terrain height in the radius 邻域中最小高程值
     * @param maxH[out] The maximal terrain height in the radius 邻域中最大高程值
     * @param is_match[out] Whether or not find terrain association in radius 是否在邻域中搜索到了有效点
     * @return The average terrain height 返回邻域范围的平均高程值
     */
    template <typename Position>
    static inline float NearestHeightOfRadius(const Position& p, const float& radius, float& minH, float& maxH, bool& is_matched) {
        std::vector<int> pIdxK;
        std::vector<float> pdDistK; // 近邻点到查询点的平方距离
        PCLPoint pcl_p;
        pcl_p.x = p.x, pcl_p.y = p.y, pcl_p.z = 0.0f, pcl_p.intensity = 0.0f;
        minH = maxH = p.z;
        is_matched = false;
        if (kdtree_terrain_clould_->radiusSearch(pcl_p, radius, pIdxK, pdDistK) > 0) { // radiusSearch返回值为找到的近邻点数量
            float avgH = kdtree_terrain_clould_->getInputCloud()->points[pIdxK[0]].intensity; // 这里的intensity被赋值为高程值
            minH = maxH = avgH;
            for (std::size_t i=1; i<pIdxK.size(); i++) {
                const float temp = kdtree_terrain_clould_->getInputCloud()->points[pIdxK[i]].intensity;
                if (temp < minH) minH = temp;
                if (temp > maxH) maxH = temp;
                avgH += temp;
            }
            avgH /= (float)pIdxK.size();
            is_matched = true;
            return avgH; // 如果查询到了最近邻点，就返回这些点的平均高程值
        }
        return p.z; // 没有查询到符合要求的最近邻点，直接返回查询点的z坐标
    }

    /** Update global cloud grid with incoming clouds 
     * @param CloudInOut incoming cloud ptr and output valid in range points
    */
    void UpdateObsCloudGrid(const PointCloudPtr& obsCloudInOut);
    void UpdateFreeCloudGrid(const PointCloudPtr& freeCloudIn);
    void UpdateTerrainHeightGrid(const PointCloudPtr& freeCloudIn, const PointCloudPtr& terrainHeightOut);

    /** Extract Surrounding Free & Obs clouds 
     * @param SurroundCloudOut output surrounding cloud ptr
    */
    void GetSurroundObsCloud(const PointCloudPtr& obsCloudOut);
    void GetSurroundFreeCloud(const PointCloudPtr& freeCloudOut);

    /** Extract Surrounding Free & Obs clouds 
     * @param center the position of the grid that want to extract
     * @param cloudOut output cloud ptr
     * @param type choose free or obstacle cloud for extraction
     * @param is_large whether or not using the surrounding cells
    */
    void GetCloudOfPoint(const Point3D& center, 
                         const PointCloudPtr& CloudOut, 
                         const CloudType& type,
                         const bool& is_large);

    /**
     * Get neihbor cells center positions
     * @param neighbor_centers[out] neighbor centers stack
    */
    void GetNeighborCeilsCenters(PointStack& neighbor_centers);

    /**
     * Get neihbor cells center positions
     * @param occupancy_centers[out] occupanied cells center stack
    */
    void GetOccupancyCeilsCenters(PointStack& occupancy_centers);

    /**
     * Remove pointcloud from grid map
     * @param obsCloud obstacle cloud points that need to be removed
    */ 
    void RemoveObsCloudFromGrid(const PointCloudPtr& obsCloud);

    /**
     * @brief Reset Current Grip Map Clouds
     */
    void ResetGripMapCloud();

    /**
     * @brief Clear the cells that from the robot position to the given position
     * @param point Give point location
     */
    void ClearObsCellThroughPosition(const Point3D& point);

private:
    MapHandlerParams map_params_;
    int neighbor_Lnum_, neighbor_Hnum_; // 根据传感器测量范围计算的附近的网格数量(水平与垂直的网格数，不是一半)
    Eigen::Vector3i robot_cell_sub_; // 机器人实时位置对应的网格索引
    int INFLATE_N;
    bool is_init_ = false;
    PointCloudPtr flat_terrain_cloud_;  // 平坦地形点云
    static PointKdTreePtr kdtree_terrain_clould_; // kd树地形点云

    // 查询给定点最近的地形高度
    template <typename Position>
    static inline float NearestHeightOfPoint(const Position& p, float& dist_square) {
        // Find the nearest node in graph
        std::vector<int> pIdxK(1);
        std::vector<float> pdDistK(1);
        PCLPoint pcl_p;
        dist_square = FARUtil::kINF;
        pcl_p.x = p.x, pcl_p.y = p.y, pcl_p.z = 0.0f, pcl_p.intensity = 0.0f;
        if (kdtree_terrain_clould_->nearestKSearch(pcl_p, 1, pIdxK, pdDistK) > 0) {
            pcl_p = kdtree_terrain_clould_->getInputCloud()->points[pIdxK[0]];
            dist_square = pdDistK[0];
            return pcl_p.intensity;
        }
        return p.z; // 查询不到返回原值
    }

    void SetTerrainHeightGridOrigin(const Point3D& robot_pos);

    void TraversableAnalysis(const PointCloudPtr& terrainHeightOut);

    // 分配平面地形点云，将地形高度赋值给intensity，z值置0
    inline void AssignFlatTerrainCloud(const PointCloudPtr& terrainRef, PointCloudPtr& terrainFlatOut) {
        const int N = terrainRef->size();
        terrainFlatOut->resize(N);
        for (int i = 0; i<N; i++) {
            PCLPoint pcl_p = terrainRef->points[i];
            pcl_p.intensity = pcl_p.z, pcl_p.z = 0.0f;
            terrainFlatOut->points[i] = pcl_p;
        }
    }

    // 计算以 csub 为中心、扩展n个单位的所有二维网格索引
    inline void Expansion2D(const Eigen::Vector3i& csub, std::vector<Eigen::Vector3i>& subs, const int& n) {
        subs.clear();
        for (int ix=-n; ix<=n; ix++) {
            for (int iy=-n; iy<=n; iy++) {
                Eigen::Vector3i sub = csub;
                sub.x() += ix, sub.y() += iy;
                subs.push_back(sub); 
            }
        }
    }

    void ObsNeighborCloudWithTerrain(std::unordered_set<int>& neighbor_obs,
                                     std::unordered_set<int>& extend_terrain_obs);

    std::unordered_set<int> neighbor_free_indices_;        // surrounding free cloud grid indices stack
    static std::unordered_set<int> neighbor_obs_indices_;  // surrounding obs cloud grid indices stack
    static std::unordered_set<int> extend_obs_indices_;    // extended surrounding obs cloud grid indices stack

    std::vector<int> global_visited_induces_;   // 地形点云的回调中所有点所在的地图网格的索引列表
    std::vector<int> util_obs_modified_list_;   // 地形点云的回调中障碍物点所在的地图网格的索引列表
    std::vector<int> util_free_modified_list_;  // 地形点云的回调中free点所在的地图网格的索引列表
    std::vector<int> util_remove_check_list_;
    static std::vector<int> terrain_grid_occupy_list_;   // 地形网格的free点列表，实时更新
    static std::vector<int> terrain_grid_traverse_list_; // 地形网格的可通行列表，实时更新

    
    static std::unique_ptr<grid_ns::Grid<PointCloudPtr>> world_free_cloud_grid_;    // free点云网格，以第一帧机器人定位为中心
    static std::unique_ptr<grid_ns::Grid<PointCloudPtr>> world_obs_cloud_grid_;     // obs点云网格，以第一帧机器人定位为中心
    static std::unique_ptr<grid_ns::Grid<std::vector<float>>> terrain_height_grid_; // 地形高度网格，以机器人当前位置为中心，实时更新
 
};

#endif