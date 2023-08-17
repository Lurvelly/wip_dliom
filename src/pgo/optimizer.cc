#include <ros/console.h>
#include "pgo/optimizer.h"

#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

namespace pgo
{

void ISAMOptimizer::optimize(bool loop_closed)
{
    optimizer_.update(graph_, estimate_);
    optimizer_.update();

    if (loop_closed)
    {
        optimizer_.update();
        optimizer_.update();
        optimizer_.update();
        optimizer_.update();
        optimizer_.update();
    }

    // update() expects changes only, so reset
    // graph and estimate
    graph_.resize(0);
    estimate_.clear();
}

void ISAMOptimizer::addPrior(const PoseWithIdx &p, const gtsam::Vector &variance)
{
    const auto &pose = p.data();
    const auto poseIdx = p.idx();
    gtsam::noiseModel::Diagonal::shared_ptr noise = gtsam::noiseModel::Diagonal::Variances(variance);
    graph_.add(gtsam::PriorFactor<gtsam::Pose3>(poseIdx, pose, noise));
    estimate_.insert(p.idx(), p.data());
    ROS_DEBUG("Added Prior: %lu", poseIdx);
}

void
ISAMOptimizer::addBetween(const PoseFrom &p1, const PoseTo &p2, const gtsam::Vector &variance)
{
    const auto &poseFrom = p1.data();
    const auto poseFromIdx = p1.idx();
    const auto &poseTo = p2.data();
    const auto poseToIdx = p2.idx();
    auto constraint = poseFrom.between(poseTo);

    gtsam::noiseModel::Diagonal::shared_ptr noise = gtsam::noiseModel::Diagonal::Variances(variance);
    graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(poseFromIdx, poseToIdx, constraint, noise));
    estimate_.insert(poseToIdx, poseTo);
    ROS_DEBUG("Added Constraint: %lu -> %lu", poseFromIdx, poseToIdx);
}

void ISAMOptimizer::addGPS(const PointWithIdx &point, const gtsam::Vector &variance)
{
    gtsam::noiseModel::Diagonal::shared_ptr noise = gtsam::noiseModel::Diagonal::Variances(variance);
    gtsam::GPSFactor gpsFactor(point.idx(), point.data(), noise);
    graph_.add(gpsFactor);
    ROS_DEBUG("Added GPS Factor. Pose id: %lu", point.idx());
}

} // end namespace pgo
