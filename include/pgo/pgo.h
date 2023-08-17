#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/dataset.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovariance.h>

#include "optimizer.h"

namespace pgo 
{

class PoseGraph
{
public:
    PoseGraph() noexcept;
    ~PoseGraph() noexcept = default;
    void addOdomFactor(const geometry_msgs::Pose& pose);
    void addGPSFactor(const geometry_msgs::PoseWithCovariance &pose);
    void addLoopFactor();
    void optimize();
    void correctKeyframes();
    inline const std::vector<gtsam::Pose3>& getEstimate() const noexcept { return keyframes_; };
private:
    ISAMOptimizer optimizer_ {};
    std::vector<gtsam::Pose3> keyframes_ {};
    std::vector<gtsam::GPSFactor> gps_keyframes_ {};
    bool loop_closed_ = false;
    bool gps_alignment_init_ = false;
    double gpsDistanceThreshold = 5.0;
};

} // namespace gpo
