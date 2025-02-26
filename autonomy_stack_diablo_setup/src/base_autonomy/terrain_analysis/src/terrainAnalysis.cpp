#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "builtin_interfaces/msg/time.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/float32.hpp>

#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "rmw/types.h"
#include "rmw/qos_profiles.h"

using namespace std;

const double PI = 3.1415926;

double scanVoxelSize = 0.05;
double decayTime = 2.0;
double noDecayDis = 4.0;
double clearingDis = 8.0; // 清除该距离内的所有点云
bool clearingCloud = false;
bool useSorting = true;
double quantileZ = 0.25; // 量化因子，从排序后的高程值中提取一定比例的高度
bool considerDrop = false;
bool limitGroundLift = false;
double maxGroundLift = 0.15;
bool clearDyObs = false;
double minDyObsDis = 0.3; // 清除动态障碍物的最小距离
double minDyObsAngle = 0;
double minDyObsRelZ = -0.5;
double absDyObsRelZThre = 0.2;
double minDyObsVFOV = -16.0;
double maxDyObsVFOV = 16.0;
int minDyObsPointNum = 1;
bool noDataObstacle = false;
int noDataBlockSkipNum = 0;
int minBlockPointNum = 10;
double vehicleHeight = 1.5;
int voxelPointUpdateThre = 100;
double voxelTimeUpdateThre = 2.0;
double minRelZ = -1.5;
double maxRelZ = 0.2;
double disRatioZ = 0.2;

// terrain voxel parameters
// 整个地形分析程序包含高低两种分辨率的体素，
// 其中低分辨率体素存储的是从slam端传过来的点云数据，存储的点云数据的距离范围比较大
// 高分辨率体素(平面体素)存储的是真正的地形分析数据，存储的点云的距离范围比较小

// 这两种分辨率的体素坐标以及一维表示其实都是类似的
// 下面以地形分析的平面体素为例，简要介绍下平面体素的摆放
// 每个地形体素的格子大小为1m，该地形一共有 21 * 21 = 441 个格子，换算成物理实际面积 441 m^2
// 同样，对于高分辨率平面体素来说，一共51 * 51 = 2601 个格子，换算成物理实际面积 10.2 * 10.2 = 104.04 m^2
/*
                  ^ y 
                  |
         —————————|—————————  （max_x，max_y）
        |         |         |
        |         |         |
        |         |         |
  - - - | - - - - | - - - - - - - - - - > x (map link)
        |         |         |
        |         |         |
        |         |         |
         —————————|—————————
      （0，0）     |
                  |
                  |
其体素分布类似于occmap，是一个正方形区域，
其中，正方形左下角为坐标原点，右上角为最大值terrainVoxelWidth
terrainVoxelCloud是一个一维数组，存储了里面的所有数据，类似于occmap，按列(y)优先的顺序进行排列
四个角的一维索引分别为: 左下角 0 , 左上角 max_y, 右下角 (max_x - 1) * max_y 右上角 max_x * max_y
*/

float terrainVoxelSize = 1.0;   // 地形体素尺寸大小，其实就是分辨率(固定值)
int terrainVoxelShiftX = 0;
int terrainVoxelShiftY = 0;
const int terrainVoxelWidth = 21; // 地形体素总宽度
int terrainVoxelHalfWidth = (terrainVoxelWidth - 1) / 2; // 一半宽度
const int terrainVoxelNum = terrainVoxelWidth * terrainVoxelWidth; // 地形体素的总数量 441

// planar voxel parameters
// 平面体素参数
float planarVoxelSize = 0.2; // 平面体素尺寸
const int planarVoxelWidth = 51; // 平面体素宽度
int planarVoxelHalfWidth = (planarVoxelWidth - 1) / 2; // 平面体素半宽
const int planarVoxelNum = planarVoxelWidth * planarVoxelWidth; // 平面体素总数量 2601

pcl::PointCloud<pcl::PointXYZI>::Ptr
    laserCloud(new pcl::PointCloud<pcl::PointXYZI>()); // ros转换过来的点云数据
pcl::PointCloud<pcl::PointXYZI>::Ptr
    laserCloudCrop(new pcl::PointCloud<pcl::PointXYZI>());  // 初步裁剪后的点云数据(map系下)
pcl::PointCloud<pcl::PointXYZI>::Ptr
    laserCloudDwz(new pcl::PointCloud<pcl::PointXYZI>()); // 一个中间变量，存储低分辨率voxel中降采样后的点云数据
pcl::PointCloud<pcl::PointXYZI>::Ptr
    terrainCloud(new pcl::PointCloud<pcl::PointXYZI>()); // 附近5m 范围的点云数据
pcl::PointCloud<pcl::PointXYZI>::Ptr
    terrainCloudElev(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloud[terrainVoxelNum]; // 低分辨率体素数组 存储点云数据

int terrainVoxelUpdateNum[terrainVoxelNum] = {0};
float terrainVoxelUpdateTime[terrainVoxelNum] = {0}; // 地形体素一维数组的更新时间

float planarVoxelElev[planarVoxelNum] = {0}; // 存储最终选取的高分辨率平面体素的高程值
int planarVoxelEdge[planarVoxelNum] = {0};
int planarVoxelDyObs[planarVoxelNum] = {0};
vector<float> planarPointElev[planarVoxelNum]; // 存储的是每个高分辨率平面体素中所有点的额高程值
// planarPointElev 是一个大小为 planarVoxelNum 的数组，数组的每个元素是一个 std::vector<float>

double laserCloudTime = 0; // 当前帧接收到的点云时间
bool newlaserCloud = false;

double systemInitTime = 0; // 系统初始化时间(用第一帧雷达数据)
bool systemInited = false; // 系统初始化的标志
int noDataInited = 0;
// 当前robot的位姿
float vehicleRoll = 0, vehiclePitch = 0, vehicleYaw = 0; // 机器人当前姿态
float vehicleX = 0, vehicleY = 0, vehicleZ = 0; // 机器人当前位置
float vehicleXRec = 0, vehicleYRec = 0; // 新接收数据时机器人的位置

float sinVehicleRoll = 0, cosVehicleRoll = 0;
float sinVehiclePitch = 0, cosVehiclePitch = 0;
float sinVehicleYaw = 0, cosVehicleYaw = 0;

pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter;

// state estimation callback function
void odometryHandler(const nav_msgs::msg::Odometry::ConstSharedPtr odom) {
  double roll, pitch, yaw;
  geometry_msgs::msg::Quaternion geoQuat = odom->pose.pose.orientation;
  tf2::Matrix3x3(tf2::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w))
      .getRPY(roll, pitch, yaw);

  vehicleRoll = roll;
  vehiclePitch = pitch;
  vehicleYaw = yaw;
  vehicleX = odom->pose.pose.position.x;
  vehicleY = odom->pose.pose.position.y;
  vehicleZ = odom->pose.pose.position.z;

  sinVehicleRoll = sin(vehicleRoll);
  cosVehicleRoll = cos(vehicleRoll);
  sinVehiclePitch = sin(vehiclePitch);
  cosVehiclePitch = cos(vehiclePitch);
  sinVehicleYaw = sin(vehicleYaw);
  cosVehicleYaw = cos(vehicleYaw);

  if (noDataInited == 0) { // 新接收的数据
    vehicleXRec = vehicleX;
    vehicleYRec = vehicleY;
    noDataInited = 1;
  }
  if (noDataInited == 1) {
    float dis = sqrt((vehicleX - vehicleXRec) * (vehicleX - vehicleXRec) +
                     (vehicleY - vehicleYRec) * (vehicleY - vehicleYRec));
    if (dis >= noDecayDis) // 车辆移动超过一定的距离
      noDataInited = 2;
  }
}

// registered laser scan callback function
// 接收到的激光雷达的坐标系在map系下
void laserCloudHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr laserCloud2) {
  laserCloudTime = rclcpp::Time(laserCloud2->header.stamp).seconds();
  if (!systemInited) { // 以第一帧激光雷达为系统初始化时间
    systemInitTime = laserCloudTime;
    systemInited = true;
  }

  laserCloud->clear();
  pcl::fromROSMsg(*laserCloud2, *laserCloud);
  // 点云裁剪，保留车辆附近的点
  pcl::PointXYZI point;
  laserCloudCrop->clear();
  int laserCloudSize = laserCloud->points.size();
  for (int i = 0; i < laserCloudSize; i++) {
    point = laserCloud->points[i];

    float pointX = point.x;
    float pointY = point.y;
    float pointZ = point.z;
    // 计算该点到vehicle中心的水平距离
    float dis = sqrt((pointX - vehicleX) * (pointX - vehicleX) +
                     (pointY - vehicleY) * (pointY - vehicleY));
    // 从两个维度(竖直和水平)对点云进行裁剪 minRelZ: -1.5  maxRelZ: 0.3 水平 11m
    // disRatioZ 是一个调整因子，表示高度裁剪范围随水平距离的变化。越远的点，允许的高度范围越宽
    // 高度裁剪范围随距离变化, 可以动态适应不同环境需求(比如负值比较大可以应对大下坡场景)
    // 如果机器仅仅是在水平地面行走的话，这两个值根据激光雷达到机器人顶部以及地面的距离取值即可(因为建图的时候是以激光雷达坐标系为准建图的)
    if (pointZ - vehicleZ > minRelZ - disRatioZ * dis &&
        pointZ - vehicleZ < maxRelZ + disRatioZ * dis &&
        dis < terrainVoxelSize * (terrainVoxelHalfWidth + 1)) {
      point.x = pointX;
      point.y = pointY;
      point.z = pointZ;
      point.intensity = laserCloudTime - systemInitTime; // 每个点的强度都自定义为 当前时间-初始化时间
      laserCloudCrop->push_back(point);
    }
  }

  newlaserCloud = true;
}

// joystick callback function
void joystickHandler(const sensor_msgs::msg::Joy::ConstSharedPtr joy) {
  if (joy->buttons[5] > 0.5) { // 右肩键R1  0:未按下 1:按下
    noDataInited = 0;
    clearingCloud = true;
  }
}

// cloud clearing callback function
void clearingHandler(const std_msgs::msg::Float32::ConstSharedPtr dis) {
  noDataInited = 0;
  clearingDis = dis->data;
  clearingCloud = true;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto nh = rclcpp::Node::make_shared("terrainAnalysis");

  nh->declare_parameter<double>("scanVoxelSize", scanVoxelSize);
  nh->declare_parameter<double>("decayTime", decayTime);
  nh->declare_parameter<double>("noDecayDis", noDecayDis);
  nh->declare_parameter<double>("clearingDis", clearingDis);
  nh->declare_parameter<bool>("useSorting", useSorting);
  nh->declare_parameter<double>("quantileZ", quantileZ);
  nh->declare_parameter<bool>("considerDrop", considerDrop);
  nh->declare_parameter<bool>("limitGroundLift", limitGroundLift);
  nh->declare_parameter<double>("maxGroundLift", maxGroundLift);
  nh->declare_parameter<bool>("clearDyObs", clearDyObs);
  nh->declare_parameter<double>("minDyObsDis", minDyObsDis);
  nh->declare_parameter<double>("minDyObsAngle", minDyObsAngle);
  nh->declare_parameter<double>("minDyObsRelZ", minDyObsRelZ);
  nh->declare_parameter<double>("absDyObsRelZThre", absDyObsRelZThre);
  nh->declare_parameter<double>("minDyObsVFOV", minDyObsVFOV);
  nh->declare_parameter<double>("maxDyObsVFOV", maxDyObsVFOV);
  nh->declare_parameter<int>("minDyObsPointNum", minDyObsPointNum);
  nh->declare_parameter<bool>("noDataObstacle", noDataObstacle);
  nh->declare_parameter<int>("noDataBlockSkipNum", noDataBlockSkipNum);
  nh->declare_parameter<int>("minBlockPointNum", minBlockPointNum);
  nh->declare_parameter<double>("vehicleHeight", vehicleHeight);
  nh->declare_parameter<int>("voxelPointUpdateThre", voxelPointUpdateThre);
  nh->declare_parameter<double>("voxelTimeUpdateThre", voxelTimeUpdateThre);
  nh->declare_parameter<double>("minRelZ", minRelZ);
  nh->declare_parameter<double>("maxRelZ", maxRelZ);
  nh->declare_parameter<double>("disRatioZ", disRatioZ);

  nh->get_parameter("scanVoxelSize", scanVoxelSize);
  nh->get_parameter("decayTime", decayTime);
  nh->get_parameter("noDecayDis", noDecayDis);
  nh->get_parameter("clearingDis", clearingDis);
  nh->get_parameter("useSorting", useSorting);
  nh->get_parameter("quantileZ", quantileZ);
  nh->get_parameter("considerDrop", considerDrop);
  nh->get_parameter("limitGroundLift", limitGroundLift);
  nh->get_parameter("maxGroundLift", maxGroundLift);
  nh->get_parameter("clearDyObs", clearDyObs);
  nh->get_parameter("minDyObsDis", minDyObsDis);
  nh->get_parameter("minDyObsAngle", minDyObsAngle);
  nh->get_parameter("minDyObsRelZ", minDyObsRelZ);
  nh->get_parameter("absDyObsRelZThre", absDyObsRelZThre);
  nh->get_parameter("minDyObsVFOV", minDyObsVFOV);
  nh->get_parameter("maxDyObsVFOV", maxDyObsVFOV);
  nh->get_parameter("minDyObsPointNum", minDyObsPointNum);
  nh->get_parameter("noDataObstacle", noDataObstacle);
  nh->get_parameter("noDataBlockSkipNum", noDataBlockSkipNum);
  nh->get_parameter("minBlockPointNum", minBlockPointNum);
  nh->get_parameter("vehicleHeight", vehicleHeight);
  nh->get_parameter("voxelPointUpdateThre", voxelPointUpdateThre);
  nh->get_parameter("voxelTimeUpdateThre", voxelTimeUpdateThre);
  nh->get_parameter("minRelZ", minRelZ);
  nh->get_parameter("maxRelZ", maxRelZ);
  nh->get_parameter("disRatioZ", disRatioZ);

  auto subOdometry = nh->create_subscription<nav_msgs::msg::Odometry>("/state_estimation", 5, odometryHandler);

  auto subLaserCloud = nh->create_subscription<sensor_msgs::msg::PointCloud2>("/registered_scan", 5, laserCloudHandler);

  auto subJoystick = nh->create_subscription<sensor_msgs::msg::Joy>("/joy", 5, joystickHandler);

  auto subClearing = nh->create_subscription<std_msgs::msg::Float32>("/map_clearing", 5, clearingHandler);

  auto pubLaserCloud = nh->create_publisher<sensor_msgs::msg::PointCloud2>("/terrain_map", 2);

  for (int i = 0; i < terrainVoxelNum; i++) {
    terrainVoxelCloud[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
  }

  downSizeFilter.setLeafSize(scanVoxelSize, scanVoxelSize, scanVoxelSize);

  rclcpp::Rate rate(100);
  bool status = rclcpp::ok();
  while (status) {
    rclcpp::spin_some(nh);
    if (newlaserCloud) { // 接收到点云后进行处理
      newlaserCloud = false;

      // terrain voxel roll over
      // 当前地形体素的中心位置(int型，世界坐标系下)
      // 在每次循环开始的时候terrainVoxelShiftX都代表上一次的体素中心坐标(也可以在一定程度上代表机器人在世界的像素坐标系下)
      float terrainVoxelCenX = terrainVoxelSize * terrainVoxelShiftX;
      float terrainVoxelCenY = terrainVoxelSize * terrainVoxelShiftY;
      // 下面做的内容都是滑动地图，类似于滑窗，使得整个terrainVoxelCloud都是以当前机器人位置为中心
      // 机器人x负方向移动超过一个体素大小
      // 则所有体素朝x正方向移动一个体素大小，并清空x负方向最左边的一列体素
      while (vehicleX - terrainVoxelCenX < -terrainVoxelSize) {
        for (int indY = 0; indY < terrainVoxelWidth; indY++) {
          pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
              terrainVoxelCloud[terrainVoxelWidth * (terrainVoxelWidth - 1) +
                                indY];
          for (int indX = terrainVoxelWidth - 1; indX >= 1; indX--) { // 将所有点云数据往左移动一列(即图示中x负方向)
            terrainVoxelCloud[terrainVoxelWidth * indX + indY] =
                terrainVoxelCloud[terrainVoxelWidth * (indX - 1) + indY];
          }
          terrainVoxelCloud[indY] = terrainVoxelCloudPtr; // 参考图示，最左边一列清空，等待该次循环新增的点云来填入
          terrainVoxelCloud[indY]->clear();
        }
        terrainVoxelShiftX--;
        terrainVoxelCenX = terrainVoxelSize * terrainVoxelShiftX;
      }
      // 机器人x正方向移动超过一个体素大小
      while (vehicleX - terrainVoxelCenX > terrainVoxelSize) {
        for (int indY = 0; indY < terrainVoxelWidth; indY++) {
          pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
              terrainVoxelCloud[indY];
          for (int indX = 0; indX < terrainVoxelWidth - 1; indX++) {
            terrainVoxelCloud[terrainVoxelWidth * indX + indY] =
                terrainVoxelCloud[terrainVoxelWidth * (indX + 1) + indY];
          }
          terrainVoxelCloud[terrainVoxelWidth * (terrainVoxelWidth - 1) +
                            indY] = terrainVoxelCloudPtr;
          terrainVoxelCloud[terrainVoxelWidth * (terrainVoxelWidth - 1) + indY]
              ->clear();
        }
        terrainVoxelShiftX++;
        terrainVoxelCenX = terrainVoxelSize * terrainVoxelShiftX;
      }
      // 机器人y负方向移动超过一个体素大小
      while (vehicleY - terrainVoxelCenY < -terrainVoxelSize) {
        for (int indX = 0; indX < terrainVoxelWidth; indX++) {
          pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
              terrainVoxelCloud[terrainVoxelWidth * indX +
                                (terrainVoxelWidth - 1)];
          for (int indY = terrainVoxelWidth - 1; indY >= 1; indY--) {
            terrainVoxelCloud[terrainVoxelWidth * indX + indY] =
                terrainVoxelCloud[terrainVoxelWidth * indX + (indY - 1)];
          }
          terrainVoxelCloud[terrainVoxelWidth * indX] = terrainVoxelCloudPtr;
          terrainVoxelCloud[terrainVoxelWidth * indX]->clear();
        }
        terrainVoxelShiftY--;
        terrainVoxelCenY = terrainVoxelSize * terrainVoxelShiftY;
      }
      // 机器人y正方向移动超过一个体素大小
      while (vehicleY - terrainVoxelCenY > terrainVoxelSize) {
        for (int indX = 0; indX < terrainVoxelWidth; indX++) {
          pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
              terrainVoxelCloud[terrainVoxelWidth * indX];
          for (int indY = 0; indY < terrainVoxelWidth - 1; indY++) {
            terrainVoxelCloud[terrainVoxelWidth * indX + indY] =
                terrainVoxelCloud[terrainVoxelWidth * indX + (indY + 1)];
          }
          terrainVoxelCloud[terrainVoxelWidth * indX +
                            (terrainVoxelWidth - 1)] = terrainVoxelCloudPtr;
          terrainVoxelCloud[terrainVoxelWidth * indX + (terrainVoxelWidth - 1)]
              ->clear();
        }
        terrainVoxelShiftY++;
        terrainVoxelCenY = terrainVoxelSize * terrainVoxelShiftY;
      }

      // stack registered laser scans
      pcl::PointXYZI point;
      // laserCloudCrop中的点都为map坐标
      int laserCloudCropSize = laserCloudCrop->points.size();
      // 首先将当前帧点云中的点(裁剪后)加入到对应低分辨率体素数组中，并更新对应体素中拥有的相应的点云数目
      for (int i = 0; i < laserCloudCropSize; i++) {
        point = laserCloudCrop->points[i];
        // 由于地形体素一直是以机器人的当前位姿为中心的，所以这里就是计算当前点在地形体素中的体素索引
        // 这个体素(也可以说是像素，或者栅格)坐标系与世界坐标系的方向相同，但是原点其实是在左下角，方便转换为一维数组
        int indX = int((point.x - vehicleX + terrainVoxelSize / 2) /
                       terrainVoxelSize) +
                   terrainVoxelHalfWidth; // 加上半宽是因为体素数组在地图的左下角才是(0,0)
        int indY = int((point.y - vehicleY + terrainVoxelSize / 2) /
                       terrainVoxelSize) +
                   terrainVoxelHalfWidth;
        // 补偿由于强制类型转换带来的舍入误差
        if (point.x - vehicleX + terrainVoxelSize / 2 < 0)
          indX--;
        if (point.y - vehicleY + terrainVoxelSize / 2 < 0)
          indY--;
        // 将当前帧的点加入到对应的存储当前地形体素的一维数组中
        if (indX >= 0 && indX < terrainVoxelWidth && indY >= 0 &&
            indY < terrainVoxelWidth) { // 只处理的处于规定地形体素范围内的点
          terrainVoxelCloud[terrainVoxelWidth * indX + indY]->push_back(point); // 注意，这个一维数组是列主序
          terrainVoxelUpdateNum[terrainVoxelWidth * indX + indY]++;
        }
      }
      // 循环遍历更新当前低分辨率地形体素数组数据
      for (int ind = 0; ind < terrainVoxelNum; ind++) { // 遍历所有地形体素一维数组的数据
        // 有以下三种情况就进行低分辨率体素更新: 
        // 1. 当前低分辨率体素内点的数量达到一定阈值(指的是距离上一次更新后经过所有点云帧数据插入后(有可能不止一次插帧!!!!)); 
        // 2. 当前时间距离上一次更新该低分辨率体素达到了规定的时间阈值; 
        // 3. 收到了清除点云的指令
        if (terrainVoxelUpdateNum[ind] >= voxelPointUpdateThre ||
            laserCloudTime - systemInitTime - terrainVoxelUpdateTime[ind] >=
                voxelTimeUpdateThre ||
            clearingCloud) {
          pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
              terrainVoxelCloud[ind]; // 指向当前选定的体素点云 智能指针
          // 对选定低分辨率体素内的点云进行降采样
          laserCloudDwz->clear();
          downSizeFilter.setInputCloud(terrainVoxelCloudPtr); // 降采样尺寸大小为 0.05 
          downSizeFilter.filter(*laserCloudDwz);

          terrainVoxelCloudPtr->clear(); // 清除掉原体素中存储的数据
          int laserCloudDwzSize = laserCloudDwz->points.size();
          // 对于降采样后的点，再进行一轮筛选，然后再放入到低分辨率体素中
          for (int i = 0; i < laserCloudDwzSize; i++) { // 遍历降采样后的点云数据
            point = laserCloudDwz->points[i];
            float dis = sqrt((point.x - vehicleX) * (point.x - vehicleX) +
                             (point.y - vehicleY) * (point.y - vehicleY));
            // 更新后的点符合一下几个要求: 1. 高度符合一定范围; 2. 时间不能太久(即允许保存一定时间内的点云数据)或者小于一定距离 3. 这个条件没看懂
            if (point.z - vehicleZ > minRelZ - disRatioZ * dis &&
                point.z - vehicleZ < maxRelZ + disRatioZ * dis &&
                (laserCloudTime - systemInitTime - point.intensity <
                     decayTime || // 这句话的意思是多久以前的点云就不考虑了，只考虑最近decayTime内的点云数据 或者 距离在noDecayDis内的点云数据是永久保存的???存疑！！！
                 dis < noDecayDis) &&
                !(dis < clearingDis && clearingCloud)) { // ！(P && Q) 等价于 ！P | !Q （德摩根定律）
                  // 上面这句是针对clearingCloud的，如果清除点云命令，就不会添加新的点云数据了，而且前面已经对点云数据做了清除  这句代码写的有点拗口
              terrainVoxelCloudPtr->push_back(point); // 将更新的点添加到该体素中
            }
          }
          // 重置一些变量
          terrainVoxelUpdateNum[ind] = 0; // 如果这里做了置零处理的话，那么terrainVoxelUpdateNum[ind]其实就表示距离上一次更新后当前voxel插入的点云数量了
          terrainVoxelUpdateTime[ind] = laserCloudTime - systemInitTime; // 更新当前低分辨率体素的更新时间
        }
      }
      // 获取附近5米范围(100m^2)的点云数据(点云存储在低分辨率体素中)
      terrainCloud->clear();
      for (int indX = terrainVoxelHalfWidth - 5;
           indX <= terrainVoxelHalfWidth + 5; indX++) {
        for (int indY = terrainVoxelHalfWidth - 5;
             indY <= terrainVoxelHalfWidth + 5; indY++) {
          *terrainCloud += *terrainVoxelCloud[terrainVoxelWidth * indX + indY];
        }
      }

      // estimate ground and compute elevation for each point
      // 重置高分辨率平面体素相关的一些变量先
      for (int i = 0; i < planarVoxelNum; i++) {
        planarVoxelElev[i] = 0;
        planarVoxelEdge[i] = 0;
        planarVoxelDyObs[i] = 0;
        planarPointElev[i].clear();
      }
      // 对机器人附近5m(100m^2)范围的点云，划分为400个分辨率为0.2的格子(更细的分辨率)，统计他们的高程值，以及碍物数量估计
      // 这里的平面体素其像素坐标系与地形体素的像素坐标系是差不多的，左下角为原点，其他跟世界坐标系的方向对齐
      int terrainCloudSize = terrainCloud->points.size();
      for (int i = 0; i < terrainCloudSize; i++) {
        point = terrainCloud->points[i];
        // 计算在高分辨率平面体素中的索引
        int indX =
            int((point.x - vehicleX + planarVoxelSize / 2) / planarVoxelSize) +
            planarVoxelHalfWidth;
        int indY =
            int((point.y - vehicleY + planarVoxelSize / 2) / planarVoxelSize) +
            planarVoxelHalfWidth;

        if (point.x - vehicleX + planarVoxelSize / 2 < 0)
          indX--;
        if (point.y - vehicleY + planarVoxelSize / 2 < 0)
          indY--;
        // 将当前点的高度值存储到当前高分辨率平面体素及其邻域的 3×3 网格单元体素中，邻域扩展的目的是处理网格边界或降低数据稀疏性
        // 由于一个体素的大小是0.2m，所以其实这样做会影响到周围0.3m远的方框中
        // 只考虑高度值在一定范围内的点(这个条件在前面点的筛选中已经做过了很多次了)
        if (point.z - vehicleZ > minRelZ && point.z - vehicleZ < maxRelZ) {
          for (int dX = -1; dX <= 1; dX++) {
            for (int dY = -1; dY <= 1; dY++) {
              if (indX + dX >= 0 && indX + dX < planarVoxelWidth &&
                  indY + dY >= 0 && indY + dY < planarVoxelWidth) {
                planarPointElev[planarVoxelWidth * (indX + dX) + indY + dY] // 里面可能存在多个值 std::vector<float>
                    .push_back(point.z); // elevation 高程 就是点的z坐标(map系下)
              }
            }
          }
        }
        // 如果要清除掉动态障碍物的话，首先统计每个高分辨率平面体素中点的个数(机器人附近5m的所有点)， 
        // 先假设所有的障碍物都是动态的障碍物并统计所有障碍物，然后利用当前帧去除动态障碍
        if (clearDyObs) {
          // 如果该点处于高分辨率平面体素地图的范围内(其实就是附近5m，原则上都会满足这一句)
          if (indX >= 0 && indX < planarVoxelWidth && indY >= 0 &&
              indY < planarVoxelWidth) {
            // 激光点相对机器人的相对物理坐标(只是位置，不一定是角度)
            float pointX1 = point.x - vehicleX;
            float pointY1 = point.y - vehicleY;
            float pointZ1 = point.z - vehicleZ;
            // 动态障碍物的初步距离和角度筛选//
            // 1. 世界坐标系下的初筛
            float dis1 = sqrt(pointX1 * pointX1 + pointY1 * pointY1);
            if (dis1 > minDyObsDis) { //我们认为处于这个距离不会产生动态障碍物(这个距离其实就是车身的宽度)
              // 减去这个minDyObsRelZ值 等于加上0.3 ，也就是考虑与diablo雷达水平面以下0.3m的点(因为这里minDyObsAngle等于0)
              float angle1 = atan2(pointZ1 - minDyObsRelZ, dis1) * 180.0 / PI;
              if (angle1 > minDyObsAngle) {
                // 将该点转到车身坐标系下(前面算出的相对坐标其实还是在map系下) ZYX顺序
                float pointX2 =
                    pointX1 * cosVehicleYaw + pointY1 * sinVehicleYaw;
                float pointY2 =
                    -pointX1 * sinVehicleYaw + pointY1 * cosVehicleYaw;
                float pointZ2 = pointZ1;

                float pointX3 =
                    pointX2 * cosVehiclePitch - pointZ2 * sinVehiclePitch;
                float pointY3 = pointY2;
                float pointZ3 =
                    pointX2 * sinVehiclePitch + pointZ2 * cosVehiclePitch;

                float pointX4 = pointX3;
                float pointY4 =
                    pointY3 * cosVehicleRoll + pointZ3 * sinVehicleRoll;
                float pointZ4 =
                    -pointY3 * sinVehicleRoll + pointZ3 * cosVehicleRoll;
                // 这里才将点真正的转换到车身坐标系下
                // 2. 车身坐标系的二次筛选(FOV,高度的绝对值)
                float dis4 = sqrt(pointX4 * pointX4 + pointY4 * pointY4);
                float angle4 = atan2(pointZ4, dis4) * 180.0 / PI;
                if (angle4 > minDyObsVFOV && angle4 < maxDyObsVFOV || fabs(pointZ4) < absDyObsRelZThre) {
                  planarVoxelDyObs[planarVoxelWidth * indX + indY]++; // 对于满足要求的点，增加它在平面体素动态障碍物的数目
                }
              }
            } else { // 距离较近的点的处理
              planarVoxelDyObs[planarVoxelWidth * indX + indY] +=
                  minDyObsPointNum;
            }
          }
        }
      }
      // 上一步clearDyObs的时候统计了高分辨率平面体素内障碍物的个数，这一步是根据当前帧的点云来进行动态障碍物的去除!!!
      // 也就是如果该帧点云处在了某个具体高分辨率平面体素中，那么就说明他不太可能是动态障碍物，就将该数量清零！
      if (clearDyObs) {
        for (int i = 0; i < laserCloudCropSize; i++) {
          point = laserCloudCrop->points[i];

          int indX = int((point.x - vehicleX + planarVoxelSize / 2) /
                         planarVoxelSize) +
                     planarVoxelHalfWidth;
          int indY = int((point.y - vehicleY + planarVoxelSize / 2) /
                         planarVoxelSize) +
                     planarVoxelHalfWidth;

          if (point.x - vehicleX + planarVoxelSize / 2 < 0)
            indX--;
          if (point.y - vehicleY + planarVoxelSize / 2 < 0)
            indY--;

          if (indX >= 0 && indX < planarVoxelWidth && indY >= 0 &&
              indY < planarVoxelWidth) {
            float pointX1 = point.x - vehicleX;
            float pointY1 = point.y - vehicleY;
            float pointZ1 = point.z - vehicleZ;

            float dis1 = sqrt(pointX1 * pointX1 + pointY1 * pointY1);
            float angle1 = atan2(pointZ1 - minDyObsRelZ, dis1) * 180.0 / PI;
            if (angle1 > minDyObsAngle) {
              planarVoxelDyObs[planarVoxelWidth * indX + indY] = 0;
            }
          }
        }
      }

      // 对高分辨率平面体素中保存的一系列高程值进行分析，得到最终的高程值
      // 一个高分辨率平面体素中可能存在很多个点的高程数据，同时，一个点的高程值会扩散到周围九个高分辨率平面体素中
      if (useSorting) { // 如果对高程值进行排序处理
        for (int i = 0; i < planarVoxelNum; i++) { // 遍历所有高分辨率平面体素内的所有高程数据
          int planarPointElevSize = planarPointElev[i].size(); // 当前高分辨率平面体素的高程值数目
          if (planarPointElevSize > 0) { // 跳过空的体素
            // 对每一个平面体素的高程值进行升序排序
            sort(planarPointElev[i].begin(), planarPointElev[i].end());
            // 计算量化ID
            int quantileID = int(quantileZ * planarPointElevSize);
            // 异常处理，一般在quantileZ设置不对的时候才会发生 正常范围是(0,1)
            if (quantileID < 0)
              quantileID = 0;
            else if (quantileID >= planarPointElevSize)
              quantileID = planarPointElevSize - 1;
            // 计算最终高程值
            if (planarPointElev[i][quantileID] >
                    planarPointElev[i][0] + maxGroundLift &&
                limitGroundLift) {
              planarVoxelElev[i] = planarPointElev[i][0] + maxGroundLift;
            } else {
              planarVoxelElev[i] = planarPointElev[i][quantileID];
            }
          }
        }
      } else { // 如果不对高程值进行排序
        for (int i = 0; i < planarVoxelNum; i++) {
          int planarPointElevSize = planarPointElev[i].size(); // 当前高分辨率平面体素的高程值数目
          if (planarPointElevSize > 0) {
            float minZ = 1000.0;
            int minID = -1;
            for (int j = 0; j < planarPointElevSize; j++) {
              if (planarPointElev[i][j] < minZ) {
                minZ = planarPointElev[i][j];
                minID = j;
              }
            }
            // 选取最低点的高程值作为该高分辨率平面体素最终的高程值
            if (minID != -1) {
              planarVoxelElev[i] = planarPointElev[i][minID];
            }
          }
        }
      }

      terrainCloudElev->clear();
      int terrainCloudElevSize = 0;
      for (int i = 0; i < terrainCloudSize; i++) { // 遍历附近5m范围的点云数据
        point = terrainCloud->points[i];
        if (point.z - vehicleZ > minRelZ && point.z - vehicleZ < maxRelZ) {
          // 计算该点在高分辨率平面体素中的索引
          int indX = int((point.x - vehicleX + planarVoxelSize / 2) /
                         planarVoxelSize) +
                     planarVoxelHalfWidth;
          int indY = int((point.y - vehicleY + planarVoxelSize / 2) /
                         planarVoxelSize) +
                     planarVoxelHalfWidth;

          if (point.x - vehicleX + planarVoxelSize / 2 < 0)
            indX--;
          if (point.y - vehicleY + planarVoxelSize / 2 < 0)
            indY--;

          if (indX >= 0 && indX < planarVoxelWidth && indY >= 0 &&
              indY < planarVoxelWidth) {
            // 如果该点不是动态障碍物  或者 清除动态障碍物的标志位为假
            if (planarVoxelDyObs[planarVoxelWidth * indX + indY] <
                    minDyObsPointNum ||
                !clearDyObs) {
              // 当前点的高度值与当前高分辨率平面体素高程的差
              float disZ =
                  point.z - planarVoxelElev[planarVoxelWidth * indX + indY];
              if (considerDrop) // 考虑当前点比高程值还低的情况(也就是当做障碍物处理)，如果不开，那么就会忽略掉坑洞，有可能会掉进去...
                disZ = fabs(disZ);
              int planarPointElevSize =
                  planarPointElev[planarVoxelWidth * indX + indY].size();
              // 如果当前点的高度差满足要求 且 所处的高分辨率平面体素中至少有minBlockPointNum个高程数据(附近点比较密集，可以一定程度上去掉某些噪点)
              if (disZ >= 0 && disZ < vehicleHeight &&
                  planarPointElevSize >= minBlockPointNum) {
                terrainCloudElev->push_back(point);
                terrainCloudElev->points[terrainCloudElevSize].intensity = disZ; // 强度值设置为高程差

                terrainCloudElevSize++;
              }
            }
          }
        }
      }

      // 如果开启了noDataObstacle 且 机器人运动了一定距离
      if (noDataObstacle && noDataInited == 2) {
        for (int i = 0; i < planarVoxelNum; i++) { // 遍历所有高分辨率平面体素
          int planarPointElevSize = planarPointElev[i].size();
          if (planarPointElevSize < minBlockPointNum) { // 如果该平面体素内的高程数据少于一定数量，就有可能是一些边缘点？
            planarVoxelEdge[i] = 1; // 应该是为了识别数据稀疏的体素，比如平面边缘或者噪声区域
          }
        }

        for (int noDataBlockSkipCount = 0;
             noDataBlockSkipCount < noDataBlockSkipNum;
             noDataBlockSkipCount++) { // 多次迭代拓展标记
          for (int i = 0; i < planarVoxelNum; i++) {
            if (planarVoxelEdge[i] >= 1) {
              int indX = int(i / planarVoxelWidth);
              int indY = i % planarVoxelWidth;
              bool edgeVoxel = false;
              for (int dX = -1; dX <= 1; dX++) {
                for (int dY = -1; dY <= 1; dY++) {
                  if (indX + dX >= 0 && indX + dX < planarVoxelWidth &&
                      indY + dY >= 0 && indY + dY < planarVoxelWidth) {
                    if (planarVoxelEdge[planarVoxelWidth * (indX + dX) + indY +
                                        dY] < planarVoxelEdge[i]) { // 检查邻域体素是否比当前体素planarVoxelEdge值低，如果是则认为当前体素是边缘体素
                      edgeVoxel = true;
                    }
                  }
                }
              }

              if (!edgeVoxel)
                planarVoxelEdge[i]++;
            }
          }
        }
        // 生成边缘点云数据
        for (int i = 0; i < planarVoxelNum; i++) {
          if (planarVoxelEdge[i] > noDataBlockSkipNum) {
            int indX = int(i / planarVoxelWidth);
            int indY = i % planarVoxelWidth;

            point.x =
                planarVoxelSize * (indX - planarVoxelHalfWidth) + vehicleX;
            point.y =
                planarVoxelSize * (indY - planarVoxelHalfWidth) + vehicleY;
            point.z = vehicleZ;
            point.intensity = vehicleHeight;

            point.x -= planarVoxelSize / 4.0;
            point.y -= planarVoxelSize / 4.0;
            terrainCloudElev->push_back(point);

            point.x += planarVoxelSize / 2.0;
            terrainCloudElev->push_back(point);

            point.y += planarVoxelSize / 2.0;
            terrainCloudElev->push_back(point);

            point.x -= planarVoxelSize / 2.0;
            terrainCloudElev->push_back(point);
          }
        }
      }

      clearingCloud = false;

      // publish points with elevation
      sensor_msgs::msg::PointCloud2 terrainCloud2;
      pcl::toROSMsg(*terrainCloudElev, terrainCloud2);
      terrainCloud2.header.stamp = rclcpp::Time(static_cast<uint64_t>(laserCloudTime * 1e9));
      terrainCloud2.header.frame_id = "map";
      pubLaserCloud->publish(terrainCloud2);
    }

    // status = ros::ok();
    status = rclcpp::ok();
    rate.sleep();
  }

  return 0;
}
