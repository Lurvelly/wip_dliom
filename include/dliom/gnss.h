#include <Eigen/Eigen>
#include <sensor_msgs/NavSatFix.h>
#include <GeographicLib/LocalCartesian.hpp> 

// TODO apply extrinsics to estimate

namespace dliom {

class ENU {
public:
  inline double& e() noexcept { return data_[0]; }
  inline const double& e() const noexcept { return data_[0]; }
  inline double& n() noexcept { return data_[1]; }
  inline const double& n() const noexcept { return data_[1]; }
  inline double& u() noexcept { return data_[2]; }
  inline const double& u() const noexcept { return data_[2]; }
private:
  Eigen::Vector3d data_ = Eigen::Vector3d::Zero();
};

class LLA {
public:
  LLA() : data_{0, 0, 0} {}
  LLA(const sensor_msgs::NavSatFix& fix)
    : LLA(fix.latitude, fix.longitude, fix.altitude) {}
  LLA(double latitude, double longitude, double altitude) 
    : data_{latitude, longitude, altitude} {}
  inline double& lat() noexcept { return data_.x(); }
  inline const double& lat() const noexcept { return data_.x(); }
  inline double& lon() noexcept { return data_.y(); }
  inline const double& lon() const noexcept { return data_.y(); }
  inline double& alt() noexcept { return data_.z(); }
  inline const double& alt() const noexcept { return data_.z(); }
private:
  Eigen::Vector3d data_;
};

class LocalCartesianGNSS {
public:
  LocalCartesianGNSS() = default;
  ~LocalCartesianGNSS() = default;

  void resetOrigin(double latitude, double longitude, double altitude)
  {
    geo_converter_.Reset(latitude, longitude, altitude);
    origin_lla_.lat() = latitude;
    origin_lla_.lon() = longitude;
    origin_lla_.alt() = altitude;
  }

  void resetOrigin(const LLA &lla) {
    resetOrigin(lla.lat(), lla.lon(), lla.alt());
  }

  
  void update(double latitude, double longitude, double altitude) {
    geo_converter_.Forward(latitude, longitude, altitude, enu_.e(), enu_.n(), enu_.u());
  }

  void update(const LLA &lla) {
    update(lla.lat(), lla.lon(), lla.alt());
  }
  
  void reverse(double e, double n, double u, double &lat, double &lon, double &alt) {
    geo_converter_.Reverse(e, n, u, lat, lon, alt);
  }
  
  void setExtrinsics(const Eigen::Vector3d& trans, const Eigen::Quaterniond& rot)
  {
    gnss_T_wrt_lidar_ = trans;
    gnss_R_wrt_lidar_ = rot;
  }

  inline double& e() noexcept { return enu_.e(); }
  inline const double& e() const noexcept { return enu_.e(); }
  inline double& n() noexcept { return enu_.n(); }
  inline const double& n() const noexcept { return enu_.n(); }
  inline double& u() noexcept { return enu_.u(); }
  inline const double& u() const noexcept { return enu_.u(); }
  inline const ENU &enu() const noexcept { return enu_; }
  inline ENU &enu() noexcept { return enu_; }

private:
  GeographicLib::LocalCartesian geo_converter_ {};
  Eigen::Quaterniond gnss_R_wrt_lidar_ {};
  Eigen::Vector3d gnss_T_wrt_lidar_ {};
  ENU enu_ {};
  LLA origin_lla_ {};
  Eigen::Vector3d pose_cov_ {};
};


} // end namespace dliom
