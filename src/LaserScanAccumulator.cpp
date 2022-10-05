#include "LaserScanAccumulator.hpp"

void LaserScanAccumulator::InsertScanIntoPointCloud(
      sensor_msgs::PointCloud2Iterator<float> pcIter,
      sensor_msgs::LaserScanConstPtr scan) {

    float w = lastPose.pose.orientation.w;
    float x = lastPose.pose.orientation.x;
    float y = lastPose.pose.orientation.y;
    float z = lastPose.pose.orientation.z;
    float m11 = 1 - 2 * y * y - 2 * z * z;
    float m12 = 2 * x * y - 2 * z * w;
    //float m13 = 2 * x * z + 2 * y * w;

    float m21 = 2 * x * y + 2 * z * w;
    float m22 = 1 - 2 * x * x - 2 * z * z;
    //float m23 = 2 * y * z - 2 * x * w;

    float m31 = 2 * x * z - 2 * y * w;
    float m32 = 2 * y * z + 2 * x * w;
    //float m33 = 1 - 2 * x * x - 2 * y * y;

    float angle = 0.f;
    for (unsigned int i = 0; i < scan->ranges.size() - 1; i++, angle += scan->angle_increment, ++pcIter) {
        float xx = - scan->ranges[i] * cosf(angle);
        float yy = - scan->ranges[i] * sinf(angle);
        pcIter[0] = xx * m11 + yy * m12;
        pcIter[1] = xx * m21 + yy * m22;
        pcIter[2] = xx * m31 + yy * m32;
    }
}

LaserScanAccumulator::LaserScanAccumulator(std::string scanTopic, std::string poseTopic, std::string pcTopic) {
    nh = new ros::NodeHandle();
    scanSub = nh->subscribe(scanTopic, 1, &LaserScanAccumulator::LaserScanCallback, this);
    poseSub = nh->subscribe(poseTopic, 1, &LaserScanAccumulator::PoseCallback, this);
    pcPub = nh->advertise<sensor_msgs::PointCloud2>(pcTopic, 1);

    pc.header.frame_id = "base_link";
    pcMod = new sensor_msgs::PointCloud2Modifier(pc);
    pcMod->setPointCloud2FieldsByString(1, "xyz");
    angleDelim = LASER_SCAN_ANGLE_DELIMETER;
}

LaserScanAccumulator::~LaserScanAccumulator() {
    delete pcMod;
    delete nh;
}

bool LaserScanAccumulator::checkPose(Orientation orient) {
    bool out = true;
    for (int i = orientationsDict.size() - 1; (i >= 0) && out; i--) {
        if ((fabs(orient.p - orientationsDict[i].p) < angleDelim) &&
            (fabs(orient.r - orientationsDict[i].r) < angleDelim)) {
            out = false;
        }
    }
    if (out)
        orientationsDict.emplace_back(orient);
    return out;
}

void LaserScanAccumulator::LaserScanCallback(sensor_msgs::LaserScanConstPtr scan) {
    Orientation lastOrient = quatToEuler(lastPose.pose.orientation);
    if (checkPose(lastOrient)) {
        size_t size = pcMod->size();
        pcMod->resize(size + scan->ranges.size() - 1);
        sensor_msgs::PointCloud2Iterator<float> iter(pc, "x");
        InsertScanIntoPointCloud(iter + size, scan);
        pc.header.stamp = ros::Time::now();
        pcPub.publish(pc);
    }
}

void LaserScanAccumulator::PoseCallback(geometry_msgs::PoseStampedConstPtr pose) {
    lastPose = *pose.get();
}

LaserScanAccumulator::Orientation LaserScanAccumulator::quatToEuler(geometry_msgs::Quaternion q) {
    Orientation out;
    out.p = asin(-2.0*(q.x*q.z - q.w*q.y));
    out.r = atan2(2.0*(q.x*q.y + q.w*q.z), q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z);
    return out;
}

void LaserScanAccumulator::SetAngleDelimeter(float delimeter) {
    angleDelim = delimeter;
}
