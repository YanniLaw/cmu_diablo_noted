#ifndef NODE_STRUCT_H
#define NODE_STRUCT_H

#include "point_struct.h"

enum NodeType {
    NOT_DEFINED = 0,  // 未定义
    GROUND      = 1,  // 地面节点
    AIR         = 2   // 空中节点
};

enum NodeFreeDirect {
  UNKNOW  =  0,   // 未知
  CONVEX  =  1,   // 凸
  CONCAVE =  2,   // 凹
  PILLAR  =  3    // 柱
};

typedef std::pair<Point3D, Point3D> PointPair;

// 激光雷达模型数据
namespace LiDARMODEL {
    /* array resolution: 1 degree */
    static const int kHorizontalFOV = 360;  
    static const int kVerticalFOV = 31; 
    static const float kAngleResX = 0.2;
    static const float kAngleResY = 2.0;    
} // NODEPARAMS

struct Polygon
{
  Polygon() = default;
  std::size_t N;  // 多边形顶点数
  std::vector<Point3D> vertices;  // 各个顶点坐标
  bool is_robot_inside; // 机器人是否在多边形内部
  bool is_pillar;       // 是否是柱形
  float perimeter;      // 周长
};

typedef std::shared_ptr<Polygon> PolygonPtr;
typedef std::vector<PolygonPtr> PolygonStack;

struct CTNode
{
    CTNode() = default;
    Point3D position; // 轮廓节点所在位置
    bool is_global_match;
    bool is_contour_necessary;
    bool is_ground_associate;
    std::size_t nav_node_id;
    NodeFreeDirect free_direct; // 节点free方向

    PointPair surf_dirs;
    PolygonPtr poly_ptr;  // 多边形轮廓指针
    std::shared_ptr<CTNode> front;  // 前驱节点
    std::shared_ptr<CTNode> back;   // 后继节点

    std::vector<std::shared_ptr<CTNode>> connect_nodes;
};

typedef std::shared_ptr<CTNode> CTNodePtr;
typedef std::vector<CTNodePtr> CTNodeStack;

struct NavNode
{
    NavNode() = default;
    std::size_t id;     // 节点唯一ID
    Point3D position;   // 节点位置
    PointPair surf_dirs;
    std::deque<Point3D> pos_filter_vec;
    std::deque<PointPair> surf_dirs_vec;
    CTNodePtr ctnode;
    bool is_active;           // 是否激活
    bool is_block_frontier;   // 是否
    bool is_contour_match;    // 是否
    bool is_odom;             // 是否为odom 节点
    bool is_goal;             // 是否为目标点
    bool is_near_nodes;       // 是否靠近节点
    bool is_wide_near;        // 是否
    bool is_merged;           // 是否合并
    bool is_covered;          // 是否覆盖
    bool is_frontier;         // 是否边界点
    bool is_finalized;        // 是否为终点
    bool is_navpoint;         // 是否为导航点
    bool is_boundary;         // 是否为边界点
    int  clear_dumper_count;  // 清除
    std::deque<int> frontier_votes;
    std::unordered_set<std::size_t> invalid_boundary;
    std::vector<std::shared_ptr<NavNode>> connect_nodes;
    std::vector<std::shared_ptr<NavNode>> poly_connects;
    std::vector<std::shared_ptr<NavNode>> contour_connects;
    std::unordered_map<std::size_t, std::deque<int>> contour_votes;
    std::unordered_map<std::size_t, std::deque<int>> edge_votes;
    std::vector<std::shared_ptr<NavNode>> potential_contours;
    std::vector<std::shared_ptr<NavNode>> potential_edges;
    std::vector<std::shared_ptr<NavNode>> trajectory_connects;
    std::unordered_map<std::size_t, std::size_t> trajectory_votes;
    std::unordered_map<std::size_t, std::size_t> terrain_votes;
    NodeType node_type; // 节点类型
    NodeFreeDirect free_direct;   // 自由方向???
    // planner members
    bool is_block_to_goal;
    bool is_traversable;        // 是否可通行
    bool is_free_traversable;   // 是否可以自由通行
    float gscore, fgscore;
    std::shared_ptr<NavNode> parent;
    std::shared_ptr<NavNode> free_parent;
    
};

typedef std::shared_ptr<NavNode> NavNodePtr;
typedef std::pair<NavNodePtr, NavNodePtr> NavEdge;

struct nodeptr_equal
{
  bool operator()(const NavNodePtr& n1, const NavNodePtr& n2) const
  {
    return n1->id == n2->id;
  }
};

struct navedge_hash
{
  std::size_t operator() (const NavEdge& nav_edge) const
  {
    return boost::hash<std::pair<std::size_t, std::size_t>>()({nav_edge.first->id, nav_edge.second->id});
  }
};

struct nodeptr_hash
{
  std::size_t operator() (const NavNodePtr& n_ptr) const
  {
    return std::hash<std::size_t>()(n_ptr->id);
  }
};

struct nodeptr_gcomp
{
  bool operator()(const NavNodePtr& n1, const NavNodePtr& n2) const
  {
    return n1->gscore > n2->gscore;
  }
};

struct nodeptr_fgcomp
{
  bool operator()(const NavNodePtr& n1, const NavNodePtr& n2) const
  {
    return n1->fgscore > n2->fgscore;
  }
};

struct nodeptr_icomp
{
  bool operator()(const NavNodePtr& n1, const NavNodePtr& n2) const
  {
    return n1->position.intensity < n2->position.intensity;
  }
};

#endif
