#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <chrono>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/clock.hpp"
#include "builtin_interfaces/msg/time.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.h>
#include <geometry_msgs/msg/polygon_stamped.h>
#include <sensor_msgs/msg/imu.h>

#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

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

double sensorOffsetX = 0;
double sensorOffsetY = 0;
double vehicleHeight = 0.75;
double terrainVoxelSize = 0.05;
double groundHeightThre = 0.1;
bool adjustZ = false;
double terrainRadiusZ = 0.5;
int minTerrainPointNumZ = 10;
double smoothRateZ = 0.2;
bool adjustIncl = false;
double terrainRadiusIncl = 1.5;
int minTerrainPointNumIncl = 500;
double smoothRateIncl = 0.2;
double InclFittingThre = 0.2;
double maxIncl = 30.0;

pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloudIncl(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloudDwz(new pcl::PointCloud<pcl::PointXYZI>());

rclcpp::Time odomTime;

float vehicleX = 0;
float vehicleY = 0;
float vehicleZ = 0;
float vehicleRoll = 0;
float vehiclePitch = 0;
float vehicleYaw = 0;

float vehicleYawRate = 0;
float vehicleSpeed = 0;

float terrainZ = 0;
float terrainRoll = 0;
float terrainPitch = 0;

pcl::VoxelGrid<pcl::PointXYZI> terrainDwzFilter;

void terrainCloudHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr terrainCloud2)
{
  if (!adjustZ && !adjustIncl)
  {
    return;
  }

  terrainCloud->clear();
  pcl::fromROSMsg(*terrainCloud2, *terrainCloud);

  pcl::PointXYZI point;
  terrainCloudIncl->clear();
  int terrainCloudSize = terrainCloud->points.size();
  double elevMean = 0;
  int elevCount = 0;
  bool terrainValid = true;
  for (int i = 0; i < terrainCloudSize; i++)
  {
    point = terrainCloud->points[i];

    float dis = sqrt((point.x - vehicleX) * (point.x - vehicleX) + (point.y - vehicleY) * (point.y - vehicleY));

    if (dis < terrainRadiusZ)
    {
      if (point.intensity < groundHeightThre)
      {
        elevMean += point.z;
        elevCount++;
      }
      else
      {
        terrainValid = false;
      }
    }

    if (dis < terrainRadiusIncl && point.intensity < groundHeightThre)
    {
      terrainCloudIncl->push_back(point);
    }
  }

  if (elevCount >= minTerrainPointNumZ)
    elevMean /= elevCount;
  else
    terrainValid = false;

  if (terrainValid && adjustZ)
  {
    terrainZ = (1.0 - smoothRateZ) * terrainZ + smoothRateZ * elevMean;
  }

  terrainCloudDwz->clear();
  terrainDwzFilter.setInputCloud(terrainCloudIncl);
  terrainDwzFilter.filter(*terrainCloudDwz);
  int terrainCloudDwzSize = terrainCloudDwz->points.size();

  if (terrainCloudDwzSize < minTerrainPointNumIncl || !terrainValid)
  {
    return;
  }

  cv::Mat matA(terrainCloudDwzSize, 2, CV_32F, cv::Scalar::all(0));
  cv::Mat matAt(2, terrainCloudDwzSize, CV_32F, cv::Scalar::all(0));
  cv::Mat matAtA(2, 2, CV_32F, cv::Scalar::all(0));
  cv::Mat matB(terrainCloudDwzSize, 1, CV_32F, cv::Scalar::all(0));
  cv::Mat matAtB(2, 1, CV_32F, cv::Scalar::all(0));
  cv::Mat matX(2, 1, CV_32F, cv::Scalar::all(0));

  int inlierNum = 0;
  matX.at<float>(0, 0) = terrainPitch;
  matX.at<float>(1, 0) = terrainRoll;
  for (int iterCount = 0; iterCount < 5; iterCount++)
  {
    int outlierCount = 0;
    for (int i = 0; i < terrainCloudDwzSize; i++)
    {
      point = terrainCloudDwz->points[i];

      matA.at<float>(i, 0) = -point.x + vehicleX;
      matA.at<float>(i, 1) = point.y - vehicleY;
      matB.at<float>(i, 0) = point.z - elevMean;

      if (fabs(matA.at<float>(i, 0) * matX.at<float>(0, 0) + matA.at<float>(i, 1) * matX.at<float>(1, 0) -
               matB.at<float>(i, 0)) > InclFittingThre &&
          iterCount > 0)
      {
        matA.at<float>(i, 0) = 0;
        matA.at<float>(i, 1) = 0;
        matB.at<float>(i, 0) = 0;
        outlierCount++;
      }
    }

    cv::transpose(matA, matAt);
    matAtA = matAt * matA;
    matAtB = matAt * matB;
    cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

    if (inlierNum == terrainCloudDwzSize - outlierCount)
      break;
    inlierNum = terrainCloudDwzSize - outlierCount;
  }

  if (inlierNum < minTerrainPointNumIncl || fabs(matX.at<float>(0, 0)) > maxIncl * PI / 180.0 ||
      fabs(matX.at<float>(1, 0)) > maxIncl * PI / 180.0)
  {
    terrainValid = false;
  }

  if (terrainValid && adjustIncl)
  {
    terrainPitch = (1.0 - smoothRateIncl) * terrainPitch + smoothRateIncl * matX.at<float>(0, 0);
    terrainRoll = (1.0 - smoothRateIncl) * terrainRoll + smoothRateIncl * matX.at<float>(1, 0);
  }
}

void speedHandler(const geometry_msgs::msg::TwistStamped::ConstSharedPtr speedIn)
{
  vehicleSpeed = speedIn->twist.linear.x;
  vehicleYawRate = speedIn->twist.angular.z;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto nh = rclcpp::Node::make_shared("vehicleSimulator");

  nh->declare_parameter<double>("sensorOffsetX", sensorOffsetX);
  nh->declare_parameter<double>("sensorOffsetY", sensorOffsetY);
  nh->declare_parameter<double>("vehicleHeight", vehicleHeight);
  nh->declare_parameter<double>("vehicleX", vehicleX);
  nh->declare_parameter<double>("vehicleY", vehicleY);
  nh->declare_parameter<double>("vehicleZ", vehicleZ);
  nh->declare_parameter<double>("terrainZ", terrainZ);
  nh->declare_parameter<double>("vehicleYaw", vehicleYaw);
  nh->declare_parameter<double>("terrainVoxelSize", terrainVoxelSize);
  nh->declare_parameter<double>("groundHeightThre", groundHeightThre);
  nh->declare_parameter<bool>("adjustZ", adjustZ);
  nh->declare_parameter<double>("terrainRadiusZ", terrainRadiusZ);
  nh->declare_parameter<int>("minTerrainPointNumZ", minTerrainPointNumZ);
  nh->declare_parameter<bool>("adjustIncl", adjustIncl);
  nh->declare_parameter<double>("terrainRadiusIncl", terrainRadiusIncl);
  nh->declare_parameter<int>("minTerrainPointNumIncl", minTerrainPointNumIncl);
  nh->declare_parameter<double>("InclFittingThre", InclFittingThre);
  nh->declare_parameter<double>("maxIncl", maxIncl);

  nh->get_parameter("sensorOffsetX", sensorOffsetX);
  nh->get_parameter("sensorOffsetY", sensorOffsetY);
  nh->get_parameter("vehicleHeight", vehicleHeight);
  nh->get_parameter("vehicleX", vehicleX);
  nh->get_parameter("vehicleY", vehicleY);
  nh->get_parameter("vehicleZ", vehicleZ);
  nh->get_parameter("terrainZ", terrainZ);
  nh->get_parameter("vehicleYaw", vehicleYaw);
  nh->get_parameter("terrainVoxelSize", terrainVoxelSize);
  nh->get_parameter("groundHeightThre", groundHeightThre);
  nh->get_parameter("adjustZ", adjustZ);
  nh->get_parameter("terrainRadiusZ", terrainRadiusZ);
  nh->get_parameter("minTerrainPointNumZ", minTerrainPointNumZ);
  nh->get_parameter("adjustIncl", adjustIncl);
  nh->get_parameter("terrainRadiusIncl", terrainRadiusIncl);
  nh->get_parameter("minTerrainPointNumIncl", minTerrainPointNumIncl);
  nh->get_parameter("InclFittingThre", InclFittingThre);
  nh->get_parameter("maxIncl", maxIncl);

  auto subTerrainCloud = nh->create_subscription<sensor_msgs::msg::PointCloud2>("/terrain_map", 2, terrainCloudHandler);

  auto subSpeed = nh->create_subscription<geometry_msgs::msg::TwistStamped>("/cmd_vel", 5, speedHandler);

  auto pubVehicleOdom = nh->create_publisher<nav_msgs::msg::Odometry>("/state_estimation", 5);
  nav_msgs::msg::Odometry odomData;
  odomData.header.frame_id = "map";
  odomData.child_frame_id = "sensor";

  auto tfBroadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*nh);
  tf2::Stamped<tf2::Transform> odomTrans;
  geometry_msgs::msg::TransformStamped transformTfGeom ; 
  odomTrans.frame_id_ = "map";

  auto pubModelState = nh->create_publisher<geometry_msgs::msg::PoseStamped>("/unity_sim/set_model_state", 5);
  geometry_msgs::msg::PoseStamped robotState;
  robotState.header.frame_id = "map";

  terrainDwzFilter.setLeafSize(terrainVoxelSize, terrainVoxelSize, terrainVoxelSize);

  RCLCPP_INFO(nh->get_logger(), "Simulation started.");
  
  rclcpp::Rate rate(200);
  bool status = rclcpp::ok();
  while (status)
  {
    rclcpp::spin_some(nh);
    float vehicleRecRoll = vehicleRoll;
    float vehicleRecPitch = vehiclePitch;
    float vehicleRecZ = vehicleZ;

    vehicleRoll = terrainRoll * cos(vehicleYaw) + terrainPitch * sin(vehicleYaw);
    vehiclePitch = -terrainRoll * sin(vehicleYaw) + terrainPitch * cos(vehicleYaw);
    vehicleYaw += 0.005 * vehicleYawRate;
    if (vehicleYaw > PI)
      vehicleYaw -= 2 * PI;
    else if (vehicleYaw < -PI)
      vehicleYaw += 2 * PI;

    vehicleX += 0.005 * cos(vehicleYaw) * vehicleSpeed +
                0.005 * vehicleYawRate * (-sin(vehicleYaw) * sensorOffsetX - cos(vehicleYaw) * sensorOffsetY);
    vehicleY += 0.005 * sin(vehicleYaw) * vehicleSpeed +
                0.005 * vehicleYawRate * (cos(vehicleYaw) * sensorOffsetX - sin(vehicleYaw) * sensorOffsetY);
    vehicleZ = terrainZ + vehicleHeight;

    odomTime = nh->now();

    // publish 200Hz odometry messages
    tf2::Quaternion quat_tf;
    quat_tf.setRPY(vehicleRoll, vehiclePitch, vehicleYaw);
    geometry_msgs::msg::Quaternion geoQuat;
    tf2::convert(quat_tf, geoQuat);

    odomData.header.stamp = odomTime;
    odomData.pose.pose.orientation = geoQuat;
    odomData.pose.pose.position.x = vehicleX;
    odomData.pose.pose.position.y = vehicleY;
    odomData.pose.pose.position.z = vehicleZ;
    odomData.twist.twist.angular.x = 200.0 * (vehicleRoll - vehicleRecRoll);
    odomData.twist.twist.angular.y = 200.0 * (vehiclePitch - vehicleRecPitch);
    odomData.twist.twist.angular.z = vehicleYawRate;
    odomData.twist.twist.linear.x = vehicleSpeed;
    odomData.twist.twist.linear.z = 200.0 * (vehicleZ - vehicleRecZ);
    pubVehicleOdom->publish(odomData);

    // publish 200Hz tf messages
    odomTrans.setRotation(tf2::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w));
    odomTrans.setOrigin(tf2::Vector3(vehicleX, vehicleY, vehicleZ));
    transformTfGeom = tf2::toMsg(odomTrans);
    transformTfGeom.child_frame_id = "sensor";
    transformTfGeom.header.stamp = odomTime;
    tfBroadcaster->sendTransform(transformTfGeom);

    // publish 200Hz Unity model state messages (this is for Unity Simulation)
    robotState.header.stamp = odomTime;
    robotState.pose.orientation = geoQuat;
    robotState.pose.position.x = vehicleX;
    robotState.pose.position.y = vehicleY;
    robotState.pose.position.z = vehicleZ;
    pubModelState->publish(robotState);

    status = rclcpp::ok();
    rate.sleep();
  }

  return 0;
}
