#ifndef SCAN_HANDLER_H
#define SCAN_HANDLER_H

#include "utility.h"

struct ScanHandlerParams {
    ScanHandlerParams() = default;
    float terrain_range;  // 地形范围
    float voxel_size;     // 体素尺寸 
    float ceil_height;    // 什么高度?? floor height
};

enum GridStatus {
    EMPTY = 0,
    SCAN  = 1,
    OBS   = 2,
    RAY   = 3,
};

class ScanHandler {

public:
    ScanHandler() = default;
    ~ScanHandler() = default;

    void Init(const ScanHandlerParams& params);

    void UpdateRobotPosition(const Point3D& odom_pos);
    
    void SetCurrentScanCloud(const PointCloudPtr& scanCloudIn, const PointCloudPtr& freeCloudIn);

    void SetSurroundObsCloud(const PointCloudPtr& obsCloudIn, 
                             const bool& is_fiWlter_cloud=false);

    void ExtractDyObsCloud(const PointCloudPtr& cloudIn, const PointCloudPtr& dyObsCloudOut);

    void GridVisualCloud(const PointCloudPtr& cloudOut, const GridStatus& type);

    void ReInitGrids();

    inline PCLPoint Ind2PCLPoint(const int& ind) {
        PCLPoint pcl_p;
        Eigen::Vector3d pos = voxel_grids_->Ind2Pos(ind);
        pcl_p.x = pos.x(), pcl_p.y = pos.y(), pcl_p.z = pos.z();
        return pcl_p;
    }

private:
    ScanHandlerParams scan_params_;
    Eigen::Vector3i center_sub_; // 机器人当前位置所在的网格索引
    int row_num_, col_num_, level_num_;
    bool is_grids_init_ = false;
    PCLPoint center_p_; // scan中心位置，也就是机器人当前位置 (pcl格式，方便与pcl函数进行运算)
    // Set resolution for Velodyne LiDAR PUCK: https://www.amtechs.co.jp/product/VLP-16-Puck.pdf
    const float ANG_RES_Y = 2.0f/180.0f * M_PI; // vertical resolution 2 degree
    const float ANG_RES_X = 0.5f/180.0f * M_PI; // horizontal resolution 0.5 degree
    std::unique_ptr<grid_ns::Grid<char>> voxel_grids_; // 体素网格，三倍的分辨率，这个网格也是以机器人的实时位置为中心的

    void SetMapOrigin(const Point3D& ori_robot_pos);
    void SetRayCloud(const Eigen::Vector3i& point_sub);

};


#endif