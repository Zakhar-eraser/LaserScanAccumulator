#ifndef _INCLUDE_ROS_STRUCTS_HPP_
#define _INCLUDE_ROS_STRUCTS_HPP_
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>

class LaserScanAccumulator

// Singleton.h
class Singleton;  // опережающее объявление
  
class SingletonDestroyer
{
  private:
    Singleton* p_instance;
  public:    
    ~SingletonDestroyer();
    void initialize( Singleton* p );
};
  
class Singleton
{
  private:
    static Singleton* p_instance;
    static SingletonDestroyer destroyer;
  protected: 
    Singleton() { }
    Singleton( const Singleton& );
    Singleton& operator=( Singleton& );
   ~Singleton() { }
    friend class SingletonDestroyer;
  public:
    static Singleton& getInstance();    
};
  
// Singleton.cpp
#include "Singleton.h"
  
Singleton * Singleton::p_instance = 0;
SingletonDestroyer Singleton::destroyer;
  
SingletonDestroyer::~SingletonDestroyer() {   
    delete p_instance; 
}
void SingletonDestroyer::initialize( Singleton* p ) {
    p_instance = p; 
}
Singleton& Singleton::getInstance() {
    if(!p_instance)     {
        p_instance = new Singleton();
        destroyer.initialize( p_instance);     
    }
    return *p_instance;
}
#endif  // _INCLUDE_ROS_STRUCTS_HPP_
