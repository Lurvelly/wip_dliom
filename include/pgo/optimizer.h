#pragma once

#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/geometry/Pose3.h>

namespace pgo
{

template <typename T>
struct XWithIdx
{
public:
    XWithIdx(const T& data, size_t idx) noexcept : data_(data), idx_(idx) {}
    ~XWithIdx() noexcept = default;
    inline const T& data() const noexcept { return data_; }
    inline size_t idx() const noexcept { return idx_; }
private:
    const T& data_;
    size_t idx_;
};

using PoseWithIdx = XWithIdx<gtsam::Pose3>;
using PoseFrom = XWithIdx<gtsam::Pose3>;
using PoseTo = XWithIdx<gtsam::Pose3>;
using PointWithIdx = XWithIdx<gtsam::Point3>;


class ISAMOptimizer
{
public:
    ISAMOptimizer() = default;
    ~ISAMOptimizer() = default;
    void optimize(bool loop_closed = false);
    void addPrior(const PoseWithIdx &pose, const gtsam::Vector &noise);
    void addBetween(const PoseFrom &poseFrom, const PoseTo &poseTo, const gtsam::Vector &noise);
    void addGPS(const PointWithIdx &pose, const gtsam::Vector &noise);
public:
    template <typename T>
    inline void add(const T &factor) { graph_.add(factor); }
    inline const gtsam::Values& estimate() const noexcept { return estimate_; }
    inline const gtsam::ISAM2& isam() const noexcept { return optimizer_; }
private:
    gtsam::NonlinearFactorGraph graph_ {};
    gtsam::Values estimate_ {};
    gtsam::ISAM2 optimizer_ {};
};

} // end namespace gtsam_wrapper
