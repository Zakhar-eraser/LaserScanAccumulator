#ifndef _LASER_SCAN_ACCUMULATOR_HPP_
#define _LASER_SCAN_ACCUMULATOR_HPP_
#define LASER_SCAN_ANGLE_DELIMETER 0.05f
#include <vector>
#include <cmath>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <string>

class LaserScanAccumulator {
  private:
    struct Pose {
      float r, p, y;
    };
    struct Vector3 {
        float x, y, z;
    };
    struct Orientation {
      float r, p;
    };

    float angleDelim;

    ros::Subscriber scanSub;
    ros::Subscriber poseSub;
    ros::Publisher pcPub;
    ros::NodeHandle *nh;
    geometry_msgs::PoseStamped lastPose;
    sensor_msgs::PointCloud2 pc;
    sensor_msgs::PointCloud2Modifier *pcMod;
    std::vector<Orientation> orientationsDict;

    void LaserScanCallback(sensor_msgs::LaserScanConstPtr scan);
    void PoseCallback(geometry_msgs::PoseStampedConstPtr pose);

    bool checkPose(Orientation orient);
    Orientation quatToEuler(geometry_msgs::Quaternion q);
  public:
    LaserScanAccumulator(std::string scanTopic, std::string poseTopic, std::string pcTopic);
    ~LaserScanAccumulator();
    void InsertScanIntoPointCloud(
      sensor_msgs::PointCloud2Iterator<float> pcIter,
      sensor_msgs::LaserScanConstPtr scan);
    void SetAngleDelimeter(float delimeter);
};
#endif  // _LASER_SCAN_ACCUMULATOR_HPP_
