/*
 * FAR Planner
 * Copyright (C) 2021 Fan Yang - All rights reserved
 * fanyang2@andrew.cmu.edu,   
 */



#include "far_planner/contour_graph.h"
#include "far_planner/intersection.h"

/***************************************************************************************/

void ContourGraph::Init(const rclcpp::Node::SharedPtr nh, const ContourGraphParams& params) {
    nh_ = nh;
    ctgraph_params_ = params;
    ContourGraph::contour_graph_.clear();
    ContourGraph::contour_polygons_.clear();
    ALIGN_ANGLE_COS = cos(M_PI - FARUtil::kAcceptAlign / 2.0f);
    is_robot_inside_poly_ = false;
    ContourGraph::global_contour_set_.clear();
    ContourGraph::boundary_contour_set_.clear();
}

// 更新轮廓图
void ContourGraph::UpdateContourGraph(const NavNodePtr& odom_node_ptr,
                                      const std::vector<std::vector<Point3D>>& filtered_contours) {
    odom_node_ptr_ = odom_node_ptr;
    this->ClearContourGraph(); // 清除轮廓图相关变量
    for (const auto& poly : filtered_contours) { // 遍历提取出的轮廓
        PolygonPtr new_poly_ptr = NULL;
        this->CreatePolygon(poly, new_poly_ptr); // 根据当前轮廓点创建多边形结构指针
        this->AddPolyToContourPolygon(new_poly_ptr); // 将该多边形加入到 contour_polygons_ 中，该变量会在上一步ClearContourGraph清空
    }
    // 更新可通行点free_odom_p
    ContourGraph::UpdateOdomFreePosition(odom_node_ptr_, FARUtil::free_odom_p);
    for (const auto& poly_ptr : ContourGraph::contour_polygons_) { // 遍历上一步加入的多边形轮廓数组
        // 根据上一步更新的可通行点重新计算 is_robot_inside 参数，即判断是否还处于轮廓内
        poly_ptr->is_robot_inside = FARUtil::PointInsideAPoly(poly_ptr->vertices, FARUtil::free_odom_p);
        CTNodePtr new_ctnode_ptr = NULL;
        if (poly_ptr->is_pillar) { // 较小的柱状轮廓
            Point3D mean_p = FARUtil::AveragePoints(poly_ptr->vertices); // 计算轮廓中心
            this->CreateCTNode(mean_p, new_ctnode_ptr, poly_ptr, true);  // 以轮廓中心坐标创建轮廓节点 ctnode
            this->AddCTNodeToGraph(new_ctnode_ptr); // 将该轮廓节点加入到轮廓节点图graph中
        } else { // 对于较大的其他轮廓，就以每个轮廓点坐标创建轮廓节点node
            CTNodeStack ctnode_stack;
            ctnode_stack.clear();
            const int N = poly_ptr->vertices.size(); // 该多边形轮廓点数
            for (int idx=0; idx<N; idx++) {
                this->CreateCTNode(poly_ptr->vertices[idx], new_ctnode_ptr, poly_ptr, false); // 以每个轮廓点坐标创建轮廓节点node
                ctnode_stack.push_back(new_ctnode_ptr); // temp stack
            }
            // add connections to contour nodes 设置当前node的前驱以及后继节点 构成闭环
            for (int idx=0; idx<N; idx++) {
                int ref_idx = FARUtil::Mod(idx-1, N); // [0,N-1]
                ctnode_stack[idx]->front = ctnode_stack[ref_idx];
                ref_idx = FARUtil::Mod(idx+1, N);
                ctnode_stack[idx]->back = ctnode_stack[ref_idx];
                this->AddCTNodeToGraph(ctnode_stack[idx]);
            }
            // add first ctnode of each polygon to poly ctnodes stack，将第一个轮廓节点ctnode放入到多边形轮廓节点数组中
            // polys_ctnodes_ 存储的是大轮廓的第一个轮廓节点
            if (!ctnode_stack.empty()) ContourGraph::polys_ctnodes_.push_back(ctnode_stack.front());
        }
    }
    this->AnalysisSurfAngleAndConvexity(ContourGraph::contour_graph_); // 平面角以及凹凸性分析
}

/* Match current contour with global navigation nodes */
void ContourGraph::MatchContourWithNavGraph(const NodePtrStack& global_nodes, const NodePtrStack& near_nodes, CTNodeStack& new_convex_vertices) {
    for (const auto& node_ptr : global_nodes) {
        node_ptr->is_contour_match = false;
        node_ptr->ctnode = NULL;
    }
    for (const auto& ctnode_ptr : ContourGraph::contour_graph_) { // distance match
        ctnode_ptr->is_global_match = false;
        ctnode_ptr->nav_node_id = 0;
        if (ctnode_ptr->free_direct != NodeFreeDirect::UNKNOW) {
            const NavNodePtr matched_node = this->NearestNavNodeForCTNode(ctnode_ptr, near_nodes);
            if (matched_node != NULL && IsCTMatchLineFreePolygon(ctnode_ptr, matched_node, false)) {
                this->MatchCTNodeWithNavNode(ctnode_ptr, matched_node);
            }   
        }
    }
    this->EnclosePolygonsCheck();
    new_convex_vertices.clear();
    for (const auto& ctnode_ptr : ContourGraph::contour_graph_) { // Get new vertices
        if (!ctnode_ptr->is_global_match && ctnode_ptr->free_direct != NodeFreeDirect::UNKNOW) {
            if (ctnode_ptr->free_direct != NodeFreeDirect::PILLAR) { // check wall contour
                const float dot_value = ctnode_ptr->surf_dirs.first * ctnode_ptr->surf_dirs.second;
                if (dot_value < ALIGN_ANGLE_COS) continue; // wall detected
            }
            new_convex_vertices.push_back(ctnode_ptr);
        }
    }
}

bool ContourGraph::IsNavNodesConnectFreePolygon(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2) {
    if (node_ptr1->is_navpoint || node_ptr2->is_navpoint) {
        if ((node_ptr1->position - node_ptr2->position).norm() < FARUtil::kNavClearDist) { // connect to internav node
            return true;
        }
    }
    const bool is_global_check = ContourGraph::IsNeedGlobalCheck(node_ptr1->position, node_ptr2->position);
    ConnectPair cedge = ContourGraph::ReprojectEdge(node_ptr1, node_ptr2, FARUtil::kProjectDist, is_global_check);
    if (node_ptr1->is_odom) {
        cedge.start_p = cv::Point2f(FARUtil::free_odom_p.x, FARUtil::free_odom_p.y);
    } else if (node_ptr2->is_odom) {
        cedge.end_p = cv::Point2f(FARUtil::free_odom_p.x, FARUtil::free_odom_p.y);
    }
    ConnectPair bd_cedge = cedge;
    const HeightPair h_pair(node_ptr1->position, node_ptr2->position);
    if (!node_ptr1->is_boundary) bd_cedge.start_p = cv::Point2f(node_ptr1->position.x, node_ptr1->position.y);
    if (!node_ptr2->is_boundary) bd_cedge.end_p = cv::Point2f(node_ptr2->position.x, node_ptr2->position.y);
    return ContourGraph::IsPointsConnectFreePolygon(cedge, bd_cedge, h_pair, is_global_check);
}

bool ContourGraph::IsPoint3DConnectFreePolygon(const Point3D& p1, const Point3D& p2) {
    const bool is_global_check = ContourGraph::IsNeedGlobalCheck(p1, p2);
    const ConnectPair ori_cedge(p1, p2);
    const ConnectPair cedge = ori_cedge;
    const HeightPair h_pair(p1, p2);
    return ContourGraph::IsPointsConnectFreePolygon(cedge, ori_cedge, h_pair, is_global_check);
}

bool ContourGraph::IsEdgeCollideBoundary(const Point3D& p1, const Point3D& p2) {
    if (ContourGraph::boundary_contour_.empty()) return false;
    const ConnectPair edge = ConnectPair(p1, p2);
    for (const auto& contour : ContourGraph::boundary_contour_) {
        if (ContourGraph::IsEdgeCollideSegment(contour, edge)) {return true;}
    }
    return false;
}

bool ContourGraph::IsNavToGoalConnectFreePolygon(const NavNodePtr& node_ptr, const NavNodePtr& goal_ptr) {
    if ((node_ptr->position - goal_ptr->position).norm() < FARUtil::kNavClearDist) return true;
    HeightPair h_pair(node_ptr->position, goal_ptr->position);
    bool is_global_check = ContourGraph::IsNeedGlobalCheck(node_ptr->position, goal_ptr->position);
    if (FARUtil::IsMultiLayer) {
        if (!FARUtil::IsAtSameLayer(node_ptr, goal_ptr) && !node_ptr->is_frontier) return false;
        h_pair = HeightPair(goal_ptr->position.z, goal_ptr->position.z);
        is_global_check = true;
    }
    const ConnectPair cedge = ContourGraph::ReprojectEdge(node_ptr, goal_ptr, FARUtil::kProjectDist, is_global_check);
    ConnectPair bd_cedge = cedge;
    if (!node_ptr->is_boundary) bd_cedge.start_p = cv::Point2f(node_ptr->position.x, node_ptr->position.y);
    if (!goal_ptr->is_boundary) bd_cedge.end_p = cv::Point2f(goal_ptr->position.x, goal_ptr->position.y);
    return ContourGraph::IsPointsConnectFreePolygon(cedge, bd_cedge, h_pair, is_global_check);
}


bool ContourGraph::IsCTMatchLineFreePolygon(const CTNodePtr& matched_ctnode, const NavNodePtr& matched_navnode, const bool& is_global_check) {
    if ((matched_ctnode->position - matched_navnode->position).norm() < FARUtil::kNavClearDist) return true;
    const HeightPair h_pair(matched_ctnode->position, matched_navnode->position);
    const ConnectPair bd_cedge = ConnectPair(matched_ctnode->position, matched_navnode->position);
    const ConnectPair cedge = ContourGraph::ReprojectEdge(matched_ctnode, matched_navnode, FARUtil::kProjectDist);
    return ContourGraph::IsPointsConnectFreePolygon(cedge, bd_cedge, h_pair, is_global_check);
}

bool ContourGraph::IsPointsConnectFreePolygon(const ConnectPair& cedge,
                                              const ConnectPair& bd_cedge,
                                              const HeightPair h_pair,
                                              const bool& is_global_check)
{
    // check for boundaries edges 
    for (const auto& contour : ContourGraph::boundary_contour_) {
        if (!ContourGraph::IsEdgeOverlapInHeight(h_pair, HeightPair(contour.first, contour.second))) continue;
        if (ContourGraph::IsEdgeCollideSegment(contour, bd_cedge)) {
            return false;
        }
    }
    if (!is_global_check) {
        // check for local range polygons
        const Point3D center_p = Point3D((cedge.start_p.x + cedge.end_p.x) / 2.0f,
                                         (cedge.start_p.y + cedge.end_p.y) / 2.0f,
                                         0.0f);
        for (const auto& poly_ptr : ContourGraph::contour_polygons_) {
            if (poly_ptr->is_pillar) continue;
            if ((poly_ptr->is_robot_inside != FARUtil::PointInsideAPoly(poly_ptr->vertices, center_p)) || 
                ContourGraph::IsEdgeCollidePoly(poly_ptr->vertices, cedge)) 
            {
                return false;
            }
        }
        // check for unmatched local contours
        for (const auto& contour : ContourGraph::unmatched_contour_) {
            if (ContourGraph::IsEdgeCollideSegment(contour, cedge)) {
                return false;
            }
        }
        // check for any inactive local contours
        for (const auto& contour : ContourGraph::inactive_contour_) {
            if (ContourGraph::IsEdgeCollideSegment(contour, cedge)) {
                return false;
            }
        }
    } else {
        for (const auto& contour : ContourGraph::global_contour_) {
            if (!ContourGraph::IsEdgeOverlapInHeight(h_pair, HeightPair(contour.first, contour.second))) continue;
            if (ContourGraph::IsEdgeCollideSegment(contour, cedge)) {
                return false;
            }
        }
        for (const auto& poly_ptr : ContourGraph::contour_polygons_) {
            if (poly_ptr->is_pillar) continue;
            if (ContourGraph::IsEdgeCollidePoly(poly_ptr->vertices, cedge)) {
                return false;
            }
        }
    }
    return true;
}

bool ContourGraph::IsNavNodesConnectFromContour(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2) {
    if (node_ptr1->is_odom || node_ptr2->is_odom) return false;
    const CTNodePtr ctnode1 = node_ptr1->ctnode;
    const CTNodePtr ctnode2 = node_ptr2->ctnode;
    if (ctnode1 == NULL || ctnode2 == NULL || ctnode1 == ctnode2) return false;
    return ContourGraph::IsCTNodesConnectFromContour(ctnode1, ctnode2);
}

bool ContourGraph::IsCTNodesConnectFromContour(const CTNodePtr& ctnode1, const CTNodePtr& ctnode2) {
    if (ctnode1 == ctnode2 || ctnode1->poly_ptr != ctnode2->poly_ptr) return false;
    // check for boundary collision
    const ConnectPair cedge = ConnectPair(ctnode1->position, ctnode2->position);
    for (const auto& contour : ContourGraph::boundary_contour_) {
        if (ContourGraph::IsEdgeCollideSegment(contour, cedge)) {
            return false;
        }
    }
    // forward search
    CTNodePtr next_ctnode = ctnode1->front; 
    while (next_ctnode != NULL && next_ctnode != ctnode1) {
        if (next_ctnode == ctnode2) {
            return true;
        }
        if (next_ctnode->is_global_match || !FARUtil::IsInCylinder(ctnode1->position, ctnode2->position, next_ctnode->position, FARUtil::kNearDist, true)) 
        {
            break;
        } else {
            next_ctnode = next_ctnode->front;
        }
    }
    // backward search
    next_ctnode = ctnode1->back;
    while (next_ctnode != NULL && next_ctnode != ctnode1) {
        if (next_ctnode == ctnode2) {
            return true;
        }
        if (next_ctnode->is_global_match || !FARUtil::IsInCylinder(ctnode1->position, ctnode2->position, next_ctnode->position, FARUtil::kNearDist, true)) 
        {
            break;
        } else {
            next_ctnode = next_ctnode->back;
        }
    }
    return false;
}

CTNodePtr ContourGraph::FirstMatchedCTNode(const CTNodePtr& ctnode_ptr) {
    if (ctnode_ptr->is_global_match) return ctnode_ptr;
    CTNodePtr cur_ctnode_ptr = ctnode_ptr->front;
    while (cur_ctnode_ptr != ctnode_ptr) {
        if (cur_ctnode_ptr->is_global_match) return cur_ctnode_ptr;
        cur_ctnode_ptr = cur_ctnode_ptr->front;
    }
    return NULL;
}

NavNodePtr ContourGraph::MatchOutrangeNodeWithCTNode(const NavNodePtr& out_node_ptr, const NodePtrStack& near_nodes) {
    if (near_nodes.empty()) return NULL;
    float min_dist = FARUtil::kINF;
    NavNodePtr min_matched_node = NULL;
    for (const auto& node_ptr : near_nodes) {
        if (!node_ptr->is_contour_match) continue;
        CTNodePtr matched_ctnode = NULL;
        if (IsContourLineMatch(node_ptr, out_node_ptr, matched_ctnode)) {
            const float dist = FARUtil::VerticalDistToLine2D(node_ptr->position, matched_ctnode->position, out_node_ptr->position);
            if (dist < min_dist) {
                min_dist = dist;
                min_matched_node = node_ptr;
            }
        }
    }
    if (min_dist < FARUtil::kNavClearDist) {
        return min_matched_node;
    }
    return NULL;
}

bool ContourGraph::IsContourLineMatch(const NavNodePtr& inNode_ptr, const NavNodePtr& outNode_ptr, CTNodePtr& matched_ctnode) {
    const CTNodePtr ctnode_ptr = inNode_ptr->ctnode;
    matched_ctnode = NULL;
    if (ctnode_ptr == NULL || ctnode_ptr->poly_ptr->is_pillar) return false;
    // check forward
    const PointPair line1(inNode_ptr->position, outNode_ptr->position);
    CTNodePtr next_ctnode = ctnode_ptr->front;
    CTNodePtr prev_ctnode = ctnode_ptr;
    while (!next_ctnode->is_global_match && next_ctnode != ctnode_ptr &&
           FARUtil::IsInCylinder(ctnode_ptr->position, next_ctnode->position, prev_ctnode->position, FARUtil::kNearDist, true)) 
    {
        if (!FARUtil::IsPointInMarginRange(next_ctnode->position) &&
            (ctnode_ptr->position - next_ctnode->position).norm_flat() > FARUtil::kMatchDist) 
        {
            const PointPair line2(ctnode_ptr->position, next_ctnode->position);
            if (FARUtil::LineMatchPercentage(line1, line2) > 0.99f) {
                if (IsCTMatchLineFreePolygon(next_ctnode, outNode_ptr, true)) {
                    matched_ctnode = next_ctnode;
                    return true;
                }
            }
        }
        prev_ctnode = next_ctnode;
        next_ctnode = next_ctnode->front;
    }
    // check backward
    next_ctnode = ctnode_ptr->back;
    prev_ctnode = ctnode_ptr;
    while (!next_ctnode->is_global_match && next_ctnode != ctnode_ptr &&
           FARUtil::IsInCylinder(ctnode_ptr->position, next_ctnode->position, prev_ctnode->position, FARUtil::kNearDist, true)) 
    {
        if (!FARUtil::IsPointInMarginRange(next_ctnode->position) && 
            (ctnode_ptr->position - next_ctnode->position).norm_flat() > FARUtil::kMatchDist) 
        {
            const PointPair line2(ctnode_ptr->position, next_ctnode->position);
            if (FARUtil::LineMatchPercentage(line1, line2) > 0.99f) {
                if (IsCTMatchLineFreePolygon(next_ctnode, outNode_ptr, true)) {
                    matched_ctnode = next_ctnode;
                    return true;
                }
            }
        }
        prev_ctnode = next_ctnode;
        next_ctnode = next_ctnode->back;
    }
    return false;
}

bool ContourGraph::IsCTNodesConnectWithinOrder(const CTNodePtr& ctnode1, const CTNodePtr& ctnode2, CTNodePtr& block_vertex) {
    block_vertex = NULL;
    if (ctnode1 == ctnode2 || ctnode1->poly_ptr != ctnode2->poly_ptr) return false;
    CTNodePtr next_ctnode = ctnode1->front; // forward search
    while (next_ctnode != NULL && next_ctnode != ctnode2) {
        if (!FARUtil::IsInCylinder(ctnode1->position, ctnode2->position, next_ctnode->position, FARUtil::kNearDist, true)) {
            block_vertex = next_ctnode;
            return false;
        }
        next_ctnode = next_ctnode->front;
    }
    return true;
}

void ContourGraph::EnclosePolygonsCheck() {
    for (const auto& ctnode_ptr : ContourGraph::polys_ctnodes_) { // loop each polygon
        if (ctnode_ptr->poly_ptr->is_pillar) continue;
        const CTNodePtr start_ctnode_ptr = FirstMatchedCTNode(ctnode_ptr);
        if (start_ctnode_ptr == NULL) continue;
        CTNodePtr pre_ctnode_ptr = start_ctnode_ptr;
        CTNodePtr cur_ctnode_ptr = start_ctnode_ptr->front;
        while (cur_ctnode_ptr != start_ctnode_ptr) {
            if (!cur_ctnode_ptr->is_global_match) {
                cur_ctnode_ptr = cur_ctnode_ptr->front;
                continue;
            }
            CTNodePtr block_vertex = NULL;
            if (!IsCTNodesConnectWithinOrder(pre_ctnode_ptr, cur_ctnode_ptr, block_vertex) && block_vertex != NULL) {
                if (block_vertex->is_ground_associate && FARUtil::IsPointInMarginRange(block_vertex->position)) {
                    block_vertex->is_contour_necessary = true;
                }
            }
            pre_ctnode_ptr = cur_ctnode_ptr;
            cur_ctnode_ptr = cur_ctnode_ptr->front;
        }
    }
}

void ContourGraph::CreateCTNode(const Point3D& pos, CTNodePtr& ctnode_ptr, const PolygonPtr& poly_ptr, const bool& is_pillar) {
    ctnode_ptr = std::make_shared<CTNode>();
    ctnode_ptr->position = pos;
    ctnode_ptr->front = NULL;
    ctnode_ptr->back  = NULL;
    ctnode_ptr->is_global_match = false;
    ctnode_ptr->is_contour_necessary = false;
    ctnode_ptr->is_ground_associate = false;
    ctnode_ptr->nav_node_id = 0;
    ctnode_ptr->poly_ptr = poly_ptr;
    ctnode_ptr->free_direct = is_pillar ? NodeFreeDirect::PILLAR : NodeFreeDirect::UNKNOW;
    ctnode_ptr->connect_nodes.clear();
}

void ContourGraph::CreatePolygon(const PointStack& poly_points, PolygonPtr& poly_ptr) {
    poly_ptr = std::make_shared<Polygon>();
    poly_ptr->N = poly_points.size();
    poly_ptr->vertices = poly_points;
    poly_ptr->is_robot_inside = FARUtil::PointInsideAPoly(poly_points, odom_node_ptr_->position);
    float perimeter = 0.0f;
    poly_ptr->is_pillar = this->IsAPillarPolygon(poly_points, perimeter);
    poly_ptr->perimeter = perimeter;
}

NavNodePtr ContourGraph::NearestNavNodeForCTNode(const CTNodePtr& ctnode_ptr, const NodePtrStack& near_nodes) {
    NavNodePtr nearest_node = NULL;
    float min_edist = FARUtil::kINF;
    const float dir_thred = 0.5f; //cos(pi/3);
    for (const auto& node_ptr : near_nodes) {
        if (node_ptr->is_odom || node_ptr->is_navpoint || FARUtil::IsOutsideGoal(node_ptr) || !IsInMatchHeight(ctnode_ptr, node_ptr)) continue;
        // no match with pillar to non-pillar local vertices
        if ((node_ptr->free_direct == NodeFreeDirect::PILLAR && ctnode_ptr->free_direct != NodeFreeDirect::PILLAR) ||
            (ctnode_ptr->free_direct == NodeFreeDirect::PILLAR && node_ptr->free_direct != NodeFreeDirect::PILLAR)) 
        {
            continue;
        }
        float dist_thred = FARUtil::kMatchDist;
        float dir_score = 0.0f;
        if (ctnode_ptr->free_direct != NodeFreeDirect::PILLAR && node_ptr->free_direct != NodeFreeDirect::UNKNOW && node_ptr->free_direct != NodeFreeDirect::PILLAR) {
            if (ctnode_ptr->free_direct == node_ptr->free_direct) {
                const Point3D topo_dir1 = FARUtil::SurfTopoDirect(node_ptr->surf_dirs);
                const Point3D topo_dir2 = FARUtil::SurfTopoDirect(ctnode_ptr->surf_dirs);
                dir_score = (topo_dir1 * topo_dir2 - dir_thred) / (1.0f - dir_thred);
            }
        } else if (node_ptr->free_direct == NodeFreeDirect::PILLAR && ctnode_ptr->free_direct == NodeFreeDirect::PILLAR) {
            dir_score = 0.5f;
        }
        dist_thred *= dir_score;
        const float edist = (node_ptr->position - ctnode_ptr->position).norm_flat();
        if (edist < dist_thred && edist < min_edist) {
            nearest_node = node_ptr;
            min_edist = edist;
        }
    }
    if (nearest_node != NULL && nearest_node->is_contour_match) {
        const float pre_dist = (nearest_node->position - nearest_node->ctnode->position).norm_flat();
        if (min_edist < pre_dist) {
            // reset matching for previous ctnode
            RemoveMatchWithNavNode(nearest_node);
        } else {
            nearest_node = NULL;
        }
    }
    return nearest_node;
}

// 分析所有轮廓节点的表面朝向以及凹凸性
void ContourGraph::AnalysisSurfAngleAndConvexity(const CTNodeStack& contour_graph) {
    for (const auto& ctnode_ptr : contour_graph) { // 遍历所有轮廓ctnode
        if (ctnode_ptr->free_direct == NodeFreeDirect::PILLAR || ctnode_ptr->poly_ptr->is_pillar) { // 对于柱状小轮廓直接标记
            ctnode_ptr->surf_dirs = {Point3D(0,0,-1), Point3D(0,0,-1)};
            ctnode_ptr->poly_ptr->is_pillar = true; // 这个参数在创建polynode的时候也设置了 
            ctnode_ptr->free_direct = NodeFreeDirect::PILLAR; // 这个参数在创建ctnode的时候已经设置了一遍
        } else { // 较大的轮廓需要其他处理  较大的轮廓每个点都构成了一个ctnode，而且每个ctnode都有前驱跟后继
            CTNodePtr next_ctnode; // 将要进行计算的下一个节点
            // front direction
            next_ctnode = ctnode_ptr->front;
            Point3D start_p = ctnode_ptr->position;
            Point3D end_p = next_ctnode->position;
            float edist = (end_p - ctnode_ptr->position).norm_flat(); // 与前驱节点的水平方向距离
            // 向前搜索，直到遇到足够远的节点(edist > kNavClearDist) 或者 搜索完成 (next_ctnode == ctnode_ptr) 就退出该循环
            // 一般来说 next_ctnode == NULL 这个条件不会触发，因为在构造轮廓节点ctnode的时候形成了闭环
            while (next_ctnode != NULL && next_ctnode != ctnode_ptr && edist < FARUtil::kNavClearDist) { // nav_clear_dist 参数
                next_ctnode = next_ctnode->front;
                start_p = end_p;
                end_p = next_ctnode->position;
                edist = (end_p - ctnode_ptr->position).norm_flat(); // 搜索节点到选定节点的水平距离
            }
            // 前向搜索该节点所在轮廓的所有节点都搜索完了，但是到该节点的距离还是很近，就说明这是一个小轮廓 对应搜索完成的退出情况 (next_ctnode == ctnode_ptr)
            // TODO  是否将该ctnode所在轮廓的所有节点都进行设置????
            if (edist < FARUtil::kNavClearDist) { // This Node should be a pillar.
                ctnode_ptr->surf_dirs = {Point3D(0,0,-1), Point3D(0,0,-1)};
                ctnode_ptr->poly_ptr->is_pillar = true;
                ctnode_ptr->free_direct = NodeFreeDirect::PILLAR;
                continue;
            } else { // 对应 edist > kNavClearDist 的退出情况 ， 计算该节点前向搜索的最佳表面朝向
                ctnode_ptr->surf_dirs.first = FARUtil::ContourSurfDirs(end_p, start_p, ctnode_ptr->position, FARUtil::kNavClearDist);
            }
            // back direction，同理，计算后向搜索的相关参数
            next_ctnode = ctnode_ptr->back;
            start_p = ctnode_ptr->position;
            end_p   = next_ctnode->position;
            edist = (end_p - ctnode_ptr->position).norm_flat();
            while (next_ctnode != NULL && next_ctnode != ctnode_ptr && edist < FARUtil::kNavClearDist) {
                next_ctnode = next_ctnode->back;
                start_p = end_p;
                end_p = next_ctnode->position;
                edist = (end_p - ctnode_ptr->position).norm_flat();
            }
            if (edist < FARUtil::kNavClearDist) { // This Node should be a pillar.
                ctnode_ptr->surf_dirs = {Point3D(0,0,-1), Point3D(0,0,-1)}; // TODO!
                ctnode_ptr->poly_ptr->is_pillar = true;
                ctnode_ptr->free_direct = NodeFreeDirect::PILLAR;
                continue;
            } else {
                ctnode_ptr->surf_dirs.second = FARUtil::ContourSurfDirs(end_p, start_p, ctnode_ptr->position, FARUtil::kNavClearDist);
            }
        }
        // analysis convexity (except pillar)
        this->AnalysisConvexityOfCTNode(ctnode_ptr);
    }
}

// 基于周长阈值判断是否属于柱状轮廓，一般柱状物体小且紧凑，周长较短
bool ContourGraph::IsAPillarPolygon(const PointStack& vertex_points, float& perimeter) {
    perimeter = 0.0f;
    if (vertex_points.size() < 3) return true; // 点数太少，无法构成闭合多边形
    Point3D prev_p(vertex_points[0]);
    for (std::size_t i=1; i<vertex_points.size(); i++) {
        const Point3D cur_p(vertex_points[i]);
        const float dist = std::hypotf(cur_p.x - prev_p.x, cur_p.y - prev_p.y);
        perimeter += dist; // 计算周长
        prev_p = cur_p;
    }
    return perimeter > ctgraph_params_.kPillarPerimeter ? false : true; // robot_dim * 4.0f
}

bool ContourGraph::IsEdgeCollideSegment(const PointPair& line, const ConnectPair& edge) {
    const cv::Point2f start_p(line.first.x, line.first.y);
    const cv::Point2f end_p(line.second.x, line.second.y);
    if (POLYOPS::doIntersect(start_p, end_p, edge.start_p, edge.end_p)) {
        return true;
    }
    return false;
}

bool ContourGraph::IsEdgeCollidePoly(const PointStack& poly, const ConnectPair& edge) {
    const int N = poly.size();
    if (N < 3) cout<<"Poly vertex size less than 3."<<endl;
    for (int i=0; i<N; i++) {
        const PointPair line(poly[i], poly[FARUtil::Mod(i+1, N)]);
        if (ContourGraph::IsEdgeCollideSegment(line, edge)) {
            return true;
        }
    }
    return false;
}

// 分析ct节点的凹凸性
void ContourGraph::AnalysisConvexityOfCTNode(const CTNodePtr& ctnode_ptr) {
    // 还是要先做柱状轮廓判断
    if (ctnode_ptr->surf_dirs.first  == Point3D(0,0,-1) || ctnode_ptr->surf_dirs.second == Point3D(0,0,-1) || ctnode_ptr->poly_ptr->is_pillar) {
        ctnode_ptr->surf_dirs.first = Point3D(0,0,-1), ctnode_ptr->surf_dirs.second == Point3D(0,0,-1);
        ctnode_ptr->poly_ptr->is_pillar = true;
        ctnode_ptr->free_direct = NodeFreeDirect::PILLAR;
        return;
    }
    bool is_wall = false;
    const Point3D topo_dir = FARUtil::SurfTopoDirect(ctnode_ptr->surf_dirs, is_wall); // 计算表面拓扑方向
    if (is_wall) { // 墙壁点处理
        ctnode_ptr->free_direct = NodeFreeDirect::UNKNOW; // 无法计算出有效方向，两个方向对称抵消，标记为 未知类型
        return;
    }
    const Point3D ev_p = ctnode_ptr->position + topo_dir * FARUtil::kLeafSize;
    if (FARUtil::IsConvexPoint(ctnode_ptr->poly_ptr, ev_p)) {
        ctnode_ptr->free_direct = NodeFreeDirect::CONVEX;   // 凸角逃离
    } else {
        ctnode_ptr->free_direct = NodeFreeDirect::CONCAVE;  // 凹角卡住
    }
}

bool ContourGraph::ReprojectPointOutsidePolygons(Point3D& point, const float& free_radius) {
    PolygonPtr inside_poly_ptr = NULL;
    bool is_inside_poly = false;
    for (const auto& poly_ptr : ContourGraph::contour_polygons_) {
        if (poly_ptr->is_pillar) continue;
        if (FARUtil::PointInsideAPoly(poly_ptr->vertices, point) && !FARUtil::PointInsideAPoly(poly_ptr->vertices, FARUtil::free_odom_p)) {
            inside_poly_ptr = poly_ptr;
            is_inside_poly = true;
            break;
        }
    }
    if (is_inside_poly) {
        float near_dist = FARUtil::kINF;
        Point3D reproject_p = point;
        Point3D free_dir(0,0,-1);
        const int N = inside_poly_ptr->vertices.size();
        for (int idx=0; idx<N; idx++) {
            const Point3D vertex = inside_poly_ptr->vertices[idx];
            const float temp_dist = (vertex - point).norm_flat();
            if (temp_dist < near_dist) {
                const Point3D dir1 = (inside_poly_ptr->vertices[FARUtil::Mod(idx-1, N)] - vertex).normalize_flat();
                const Point3D dir2 = (inside_poly_ptr->vertices[FARUtil::Mod(idx+1, N)] - vertex).normalize_flat();
                const Point3D dir = (dir1 + dir2).normalize_flat();
                if (FARUtil::PointInsideAPoly(inside_poly_ptr->vertices, vertex + dir * FARUtil::kLeafSize)) { // convex 
                    reproject_p = vertex;
                    near_dist = temp_dist;
                    free_dir = dir;
                }
            }
        }
        const float origin_z = point.z;
        point = reproject_p - free_dir * free_radius;
        point.z = origin_z;
    }
    return is_inside_poly;
}

void ContourGraph::AddContourToSets(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2) {
    NavEdge edge(node_ptr1, node_ptr2);
    // force to form pair id1 < id2
    if (node_ptr1->id > node_ptr2->id) edge = NavEdge(node_ptr2, node_ptr1);

    ContourGraph::global_contour_set_.insert(edge);
    if (node_ptr1->is_boundary && node_ptr2->is_boundary) {
        ContourGraph::boundary_contour_set_.insert(edge);
    }
}

void ContourGraph::DeleteContourFromSets(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2) {
    NavEdge edge(node_ptr1, node_ptr2);
    // force to form pair id1 < id2
    if (node_ptr1->id > node_ptr2->id) edge = NavEdge(node_ptr2, node_ptr1);

    ContourGraph::global_contour_set_.erase(edge);
    if (node_ptr1->is_boundary && node_ptr2->is_boundary) {
        ContourGraph::boundary_contour_set_.erase(edge);
    }
}

void ContourGraph::ExtractGlobalContours() {
    ContourGraph::global_contour_.clear();
    ContourGraph::inactive_contour_.clear();
    ContourGraph::unmatched_contour_.clear();
    ContourGraph::boundary_contour_.clear();
    ContourGraph::local_boundary_.clear();
    for (const auto& edge : ContourGraph::global_contour_set_) {
        ContourGraph::global_contour_.push_back({edge.first->position, edge.second->position});
        if (IsEdgeInLocalRange(edge.first, edge.second)) {
            if (!this->IsActiveEdge(edge.first, edge.second)) {
                ContourGraph::inactive_contour_.push_back({edge.first->position, edge.second->position});
            } else if (!edge.first->is_near_nodes || !edge.second->is_near_nodes) {
                PointPair unmatched_pair = std::make_pair(edge.first->position, edge.second->position);
                if (edge.first->is_contour_match) {
                    unmatched_pair.first = edge.first->ctnode->position;
                } else if (edge.second->is_contour_match) {
                    unmatched_pair.second = edge.second->ctnode->position;
                } 
                ContourGraph::unmatched_contour_.push_back(unmatched_pair);
            }
        }
    }
    for (const auto& edge : ContourGraph::boundary_contour_set_) {
        ContourGraph::boundary_contour_.push_back({edge.first->position, edge.second->position});
        if (IsEdgeInLocalRange(edge.first, edge.second)) {
            ContourGraph::local_boundary_.push_back({edge.first->position, edge.second->position});
            bool is_new_invalid = false;
            if (!IsValidBoundary(edge.first, edge.second, is_new_invalid) && is_new_invalid) {
                edge.first->invalid_boundary.insert(edge.second->id);
                edge.second->invalid_boundary.insert(edge.first->id);
            }
        }
    }
}

bool ContourGraph::IsValidBoundary(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2, bool& is_new) {
    is_new = true;
    if (node_ptr1->invalid_boundary.find(node_ptr2->id) != node_ptr1->invalid_boundary.end()) { // already invalid
        is_new = false;
        return false;
    }
    // check against local polygon
    const ConnectPair cedge = ConnectPair(node_ptr1->position, node_ptr2->position);
    for (const auto& poly_ptr : ContourGraph::contour_polygons_) {
        if (poly_ptr->is_pillar) continue;
        if (ContourGraph::IsEdgeCollidePoly(poly_ptr->vertices, cedge)) {
            return false;
        }
    }
    return true;
}

/**
 * @brief 根据odom导航节点以及当前的多边形轮廓，找到一个可通行的点
 * 
 * @param odom_ptr odom导航节点
 * @param global_free_p 计算出来的可通行点，如果找不到，那么为odom节点坐标
 */
void ContourGraph::UpdateOdomFreePosition(const NavNodePtr& odom_ptr, Point3D& global_free_p) {
    Point3D free_p = odom_ptr->position;
    bool is_free_p = true;
    PointStack free_sample_points;
    for (const auto& poly_ptr : ContourGraph::contour_polygons_) { // 遍历所有多边形轮廓
        // is_robot_inside 为 true 则表明当前机器人位置处于某个轮廓内部，因为该变量是根据实时的odom位置计算出来的
        if (!poly_ptr->is_pillar && poly_ptr->is_robot_inside) { // 如果非柱状(轮廓比较大)且机器人处于该轮廓内部
            is_free_p = false;
            // 在机器人位置周围生成一系列栅格采样点集合
            FARUtil::CreatePointsAroundCenter(free_p, FARUtil::kNavClearDist, FARUtil::kLeafSize, free_sample_points); // nav_clear_dist 参数
            break;
        }
    }
    if (is_free_p) is_robot_inside_poly_ = false; // 机器人当前位置不在多边形轮廓内部
    global_free_p = free_p; // 先赋值，如果该点不是free的，后面再找一个free的
    if (!is_free_p && !is_robot_inside_poly_) { // 如果机器人在多边形轮廓内部 且 刚刚重置了环境(只有这样才会is_free_p为false的情况下is_robot_inside_poly_也为false)
        bool is_free_pos_found = false;
        // 遍历所有的采样点，与所有多边形轮廓进行比较，来找到一个可以通行的free点
        for (const auto& p : free_sample_points) {
            bool is_sample_free = true;
            for (const auto& poly_ptr : ContourGraph::contour_polygons_) {
                if (!poly_ptr->is_pillar && FARUtil::PointInsideAPoly(poly_ptr->vertices, p)) {
                    is_sample_free = false;
                    break; //TODO 有一个不满足就退出???? 应该是所有都不满足才退出吧！
                }
            }
            if (is_sample_free) { // 有一个free点就退出  不进行选择么？
                global_free_p = p;
                is_free_pos_found = true;
                break;
            }
        }
        if (!is_free_pos_found) is_robot_inside_poly_ = true; // 没有找到free的位置，标志位置true
    }
}

ConnectPair ContourGraph::ReprojectEdge(const CTNodePtr& ctnode_ptr1, const NavNodePtr& node_ptr2, const float& dist) {
    ConnectPair edgeOut;
    const float ndist = (ctnode_ptr1->position - node_ptr2->position).norm_flat();
    const float ref_dist = std::min(ndist*0.4f, dist);

    edgeOut.start_p = ProjectNode(ctnode_ptr1, ref_dist); // node 1
    edgeOut.end_p   = ProjectNode(node_ptr2, ref_dist);   // node 2

    return edgeOut;
}

ConnectPair ContourGraph::ReprojectEdge(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2, const float& dist, const bool& is_global_check) {
    ConnectPair edgeOut;
    const float ndist = (node_ptr1->position - node_ptr2->position).norm_flat();
    const float ref_dist = std::min(ndist*0.4f, dist);
    // node 1
    if (!is_global_check && node_ptr1->is_contour_match && node_ptr1->ctnode->free_direct == node_ptr1->free_direct) { // node 1
        const auto ctnode1 = node_ptr1->ctnode;
        edgeOut.start_p = ProjectNode(ctnode1, ref_dist); // node 1
    } else {
        edgeOut.start_p = ProjectNode(node_ptr1, ref_dist); // node 1
    }
    // node 2
    if (!is_global_check && node_ptr2->is_contour_match && node_ptr2->ctnode->free_direct == node_ptr2->free_direct) { // node 2
        const auto ctnode2 = node_ptr2->ctnode;
        edgeOut.end_p = ProjectNode(ctnode2, ref_dist); // node 1
    } else {
        edgeOut.end_p = ProjectNode(node_ptr2, ref_dist);
    }
    return edgeOut;
}

void ContourGraph::ResetCurrentContour() {
    this->ClearContourGraph();
    // clear contour sets
    ContourGraph::global_contour_set_.clear();
    ContourGraph::boundary_contour_set_.clear();

    odom_node_ptr_ = NULL;
    is_robot_inside_poly_ = false;
}   


