#ifndef _LASER_SCAN_ACCUMULATOR_HPP_
#define _LASER_SCAN_ACCUMULATOR_HPP_
#include <vector>

struct Vector3 {
  float x, y, z;
  Vector3(float x, float y, float z);
};

class LaserScanAccumulator;

class LaserScanAccumulatorDestroyer {
  private:
    LaserScanAccumulator* p_instance;
  public:
    ~LaserScanAccumulatorDestroyer();
    void initialize( LaserScanAccumulator* p );
};

class LaserScanAccumulator {
  private:
    static LaserScanAccumulator* p_instance;
    static LaserScanAccumulatorDestroyer destroyer;
  protected: 
    LaserScanAccumulator() { }
    LaserScanAccumulator( const LaserScanAccumulator& );
    LaserScanAccumulator& operator=( LaserScanAccumulator& );
   ~LaserScanAccumulator() { }
    friend class LaserScanAccumulatorDestroyer;
  public:
    static LaserScanAccumulator& getInstance();
    std::vector<Vector3> GetPointCloudFromLaserScan(std::vector<float> ranges, float angleInc);
};
#endif  // _LASER_SCAN_ACCUMULATOR_HPP_
