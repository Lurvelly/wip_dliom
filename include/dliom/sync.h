#pragma once

#include <ros/time.h>
#include <mutex>

namespace dliom {

class SensorTimeSync {
public:
  SensorTimeSync() : t_0(ros::Time::now()), t_0_sensor() {}

  ~SensorTimeSync() = default;
  ros::Time apply(const ros::Time& t_sensor) {
    std::unique_lock<decltype(this->mtx)> lock(this->mtx);
    if (t_0_sensor == ros::Time(0)) {
      t_0_sensor = t_sensor;
    }
    return t_0 + (t_sensor - t_0_sensor);
  }
private:
  std::mutex mtx;
  ros::Time t_0;
  ros::Time t_0_sensor;
};

} // end namespace dliom
