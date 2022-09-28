#include "LaserScanAccumulator.hpp"
  
LaserScanAccumulator * LaserScanAccumulator::p_instance = 0;
LaserScanAccumulatorDestroyer LaserScanAccumulator::destroyer;

Vector3::Vector3(float x = 0.f, float y = 0.f, float z = 0.f) {
    this->x = x;
    this->y = y;
    this->z = z;
}
  
LaserScanAccumulatorDestroyer::~LaserScanAccumulatorDestroyer() {   
    delete p_instance; 
}
void LaserScanAccumulatorDestroyer::initialize( LaserScanAccumulator* p ) {
    p_instance = p; 
}
LaserScanAccumulator& LaserScanAccumulator::getInstance() {
    if(!p_instance)     {
        p_instance = new LaserScanAccumulator();
        destroyer.initialize( p_instance);     
    }
    return *p_instance;
}