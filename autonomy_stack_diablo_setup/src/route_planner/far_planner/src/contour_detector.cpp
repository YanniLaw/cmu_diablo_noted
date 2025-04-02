/*
 * FAR Planner
 * Copyright (C) 2021 Fan Yang - All rights reserved
 * fanyang2@andrew.cmu.edu,   
 */

#include "far_planner/contour_detector.h"

// const static int BLUR_SIZE = 10;

/***************************************************************************************/

void ContourDetector::Init(const ContourDetectParams& params) {
    cd_params_ = params;
    /* Allocate Pointcloud pointer memory */
    new_corners_cloud_   = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
    // Init projection cv Mat
    // 以传感器测量范围为一半边长的图像
    MAT_SIZE = std::ceil(cd_params_.sensor_range * 2.0f / cd_params_.voxel_dim);
    if (MAT_SIZE % 2 == 0) MAT_SIZE ++;
    MAT_RESIZE = MAT_SIZE * (int)cd_params_.kRatio; // 3.0
    CMAT = MAT_SIZE / 2, CMAT_RESIZE = MAT_RESIZE / 2;
    img_mat_ = cv::Mat::zeros(MAT_SIZE, MAT_SIZE, CV_32FC1); // 以机器人当前位置为中心
    img_counter_ = 0;
    odom_node_ptr_ = NULL;
    refined_contours_.clear(), refined_hierarchy_.clear();
    DIST_LIMIT = cd_params_.kRatio * 1.2f;
    ALIGN_ANGLE_COS = cos(FARUtil::kAcceptAlign / 2.0f); // accept_max_align_angle 4/2 = 2 114 度
    VOXEL_DIM_INV = 1.0f / cd_params_.voxel_dim;
}

void ContourDetector::BuildTerrainImgAndExtractContour(const NavNodePtr& odom_node_ptr,
                                                       const PointCloudPtr& surround_cloud,
                                                       std::vector<PointStack>& realworl_contour) {
    CVPointStack cv_corners;
    PointStack corner_vec;
    this->UpdateOdom(odom_node_ptr);
    this->ResetImgMat(img_mat_);
    this->UpdateImgMatWithCloud(surround_cloud, img_mat_);
    this->ExtractContourFromImg(img_mat_, refined_contours_, realworl_contour);
}

void ContourDetector::UpdateImgMatWithCloud(const PointCloudPtr& pc, cv::Mat& img_mat) {
    int row_idx, col_idx, inf_row, inf_col;
    const std::vector<int> inflate_vec{-1, 0, 1};
    for (const auto& pcl_p : pc->points) {
        this->PointToImgSub(pcl_p, odom_pos_, row_idx, col_idx, false, false); // 因为下一步有判断，所以最后一个参数为false
        if (!this->IsIdxesInImg(row_idx, col_idx)) continue;
        // 以当前点所在index为中心，向附近八邻域膨胀
        for (const auto& dr : inflate_vec) {
            for (const auto& dc : inflate_vec) {
                inf_row = row_idx+dr, inf_col = col_idx+dc;
                if (this->IsIdxesInImg(inf_row, inf_col)) {
                    img_mat.at<float>(inf_row, inf_col) += 1.0; // TODO:是否可以加上权重值exp,这样可能会对窄通道有一定影响;z值是否限制
                }
            }
        }
    }
    // 动态环境下，对该图像进行二值化处理
    if (!FARUtil::IsStaticEnv) {
        cv::threshold(img_mat, img_mat, cd_params_.kThredValue, 1.0, cv::ThresholdTypes::THRESH_BINARY);
    }
    if (cd_params_.is_save_img) this->SaveCurrentImg(img_mat); // 保存二值化图像
}

void ContourDetector::ResizeAndBlurImg(const cv::Mat& img, cv::Mat& Rimg) {
    img.convertTo(Rimg, CV_8UC1, 255); // 二值化图像转换为CV_8UC1并调整像素
    cv::resize(Rimg, Rimg, cv::Size(), cd_params_.kRatio, cd_params_.kRatio, 
               cv::InterpolationFlags::INTER_LINEAR);
    //cv::morphologyEx(Rimg, Rimg, cv::MORPH_OPEN, getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));
    // 均值滤波，平滑去噪 (滤波器窗口大小为kBlurSize)
    cv::boxFilter(Rimg, Rimg, -1, cv::Size(cd_params_.kBlurSize, cd_params_.kBlurSize), cv::Point2i(-1, -1), false);
    //cv::morphologyEx(Rimg, Rimg, cv::MORPH_CLOSE, getStructuringElement(cv::MORPH_RECT, cv::Size(cd_params_.kBlurSize+2, cd_params_.kBlurSize+2)));
}

void ContourDetector::ExtractContourFromImg(const cv::Mat& img, // 二值化后的图像
                                            std::vector<CVPointStack>& img_contours, 
                                            std::vector<PointStack>& realworld_contour)
{
    cv::Mat Rimg;
    this->ResizeAndBlurImg(img, Rimg); // 图像预处理
    this->ExtractRefinedContours(Rimg, img_contours); // 提取精细轮廓
    this->ConvertContoursToRealWorld(img_contours, realworld_contour);
}

void ContourDetector::ConvertContoursToRealWorld(const std::vector<CVPointStack>& ori_contours,
                                                 std::vector<PointStack>& realWorld_contours)
{
    const std::size_t C_N = ori_contours.size();
    realWorld_contours.clear(), realWorld_contours.resize(C_N);
    for (std::size_t i=0; i<C_N; i++) {
        const CVPointStack cv_contour = ori_contours[i];
        this->ConvertCVToPoint3DVector(cv_contour, realWorld_contours[i], true);
    }
}


void ContourDetector::ShowCornerImage(const cv::Mat& img_mat, const PointCloudPtr& pc) {
    cv::Mat dst = cv::Mat::zeros(MAT_RESIZE, MAT_RESIZE, CV_8UC3);
    const int circle_size = (int)(cd_params_.kRatio*1.5);
    for (std::size_t i=0; i<pc->size(); i++) {
        cv::Point2f cv_p = this->ConvertPoint3DToCVPoint(pc->points[i], odom_pos_, true);
        cv::circle(dst, cv_p, circle_size, cv::Scalar(128,128,128), -1);

    }
    // show free odom point
    cv::circle(dst, free_odom_resized_, circle_size, cv::Scalar(0,0,255), -1);
    std::vector<std::vector<cv::Point2i>> round_contours;
    this->RoundContours(refined_contours_, round_contours);
    for(std::size_t idx=0; idx<round_contours.size(); idx++) {
        cv::Scalar color(rand()&255, rand()&255, rand()&255 );
        cv::drawContours(dst, round_contours, idx, color, cv::LineTypes::LINE_4);
    }
    cv::imshow("Obstacle Cloud Image", dst);
    cv::waitKey(30);
}

void ContourDetector::ExtractRefinedContours(const cv::Mat& imgIn,
                                            std::vector<CVPointStack>& refined_contours) 
{ 

    std::vector<std::vector<cv::Point2i>> raw_contours; // 存储所有检测到的轮廓点的集合
    refined_contours.clear(), refined_hierarchy_.clear(); // 存储轮廓之间的层次结构
    // 轮廓检测
    cv::findContours(imgIn, raw_contours, refined_hierarchy_, 
                     cv::RetrievalModes::RETR_TREE, // 检索模式: 检索所有轮廓，并创建一个包含所有轮廓的层次结构
                     cv::ContourApproximationModes::CHAIN_APPROX_TC89_L1);
    // 对检测到的轮廓进行多边形近似处简化处理                
    refined_contours.resize(raw_contours.size());
    for (std::size_t i=0; i<raw_contours.size(); i++) {
        // using Ramer–Douglas–Peucker algorithm url: https://en.wikipedia.org/wiki/Ramer%E2%80%93Douglas%E2%80%93Peucker_algorithm
        // DIST_LIMIT 表示轮廓点与多边形逼近的最大允许距离（即逼近的精度）。该值越大，简化后的多边形就越粗糙，保留的点越少；该值越小，简化后的多边形就越接近原始轮廓，保留的点越多。
        cv::approxPolyDP(raw_contours[i], refined_contours[i], DIST_LIMIT, true); // 闭合轮廓
    }
    this->TopoFilterContours(refined_contours); // 对轮廓进行拓扑过滤
    this->AdjecentDistanceFilter(refined_contours);
}

void ContourDetector::AdjecentDistanceFilter(std::vector<CVPointStack>& contoursInOut) {
    /* filter out vertices that are overlapped with neighbor */
    std::unordered_set<int> remove_idxs;
    for (std::size_t i=0; i<contoursInOut.size(); i++) { 
        const auto c = contoursInOut[i]; // 当前轮廓
        const std::size_t c_size = c.size();
        std::size_t refined_idx = 0;
        for (std::size_t j=0; j<c_size; j++) { // 该轮廓中所有点
            cv::Point2f p = c[j]; 
            if (refined_idx < 1 || FARUtil::PixelDistance(contoursInOut[i][refined_idx-1], p) > DIST_LIMIT) {
                /** Reduce wall nodes */
                RemoveWallConnection(contoursInOut[i], p, refined_idx); // refined_idx需要>2这个函数才有用
                contoursInOut[i][refined_idx] = p;
                refined_idx ++;
            }
        }
        /** Reduce wall nodes */
        RemoveWallConnection(contoursInOut[i], contoursInOut[i][0], refined_idx);
        contoursInOut[i].resize(refined_idx);
        if (refined_idx > 1 && FARUtil::PixelDistance(contoursInOut[i].front(), contoursInOut[i].back()) < DIST_LIMIT) {
            contoursInOut[i].pop_back();
        }
        if (contoursInOut[i].size() < 3) remove_idxs.insert(i);
    }
    if (!remove_idxs.empty()) { // clear contour with vertices size less that 3
        std::vector<CVPointStack> temp_contours = contoursInOut;
        contoursInOut.clear();
        for (std::size_t i=0; i<temp_contours.size(); i++) {
            if (remove_idxs.find(i) != remove_idxs.end()) continue;
            contoursInOut.push_back(temp_contours[i]);
        }
    }
}

// 对轮廓进行拓扑过滤，移除一些不符合要求的轮廓
void ContourDetector::TopoFilterContours(std::vector<CVPointStack>& contoursInOut) {
    std::unordered_set<int> remove_idxs;
    for (std::size_t i=0; i<contoursInOut.size(); i++) {
        if (remove_idxs.find(i) != remove_idxs.end()) continue;
        const auto poly = contoursInOut[i];
        if (poly.size() < 3) { // 轮廓点数太少
            remove_idxs.insert(i);
        } else if (!FARUtil::PointInsideAPoly(poly, free_odom_resized_)) { // free_odom_resized_ 点处于多边形轮廓外部
            InternalContoursIdxs(refined_hierarchy_, i, remove_idxs); // 去除所有子轮廓，只保留最外层轮廓
        } // TODO 如果free_odom_resized_点处于轮廓内部呢？
    }
    if (!remove_idxs.empty()) {
        std::vector<CVPointStack> temp_contours = contoursInOut;
        contoursInOut.clear();
        for (std::size_t i=0; i<temp_contours.size(); i++) {
            if (remove_idxs.find(i) != remove_idxs.end()) continue;
            contoursInOut.push_back(temp_contours[i]);
        }
    }
}



