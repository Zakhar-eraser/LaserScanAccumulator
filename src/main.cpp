#include "LaserScanAccumulator.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "pointcloud_accumulator");
    LaserScanAccumulator accum("ray/scan", "/mavros/local_position/pose", "accumulated_pointcloud");
    ros::spin();
    return 0;
}