
/**
  * @file GTSAMManager.cpp
  * @author julian 
  * @date 7/22/23
 */
#include "pgo/pgo.h"
#include "pgo/optimizer.h"
#include "pgo/util.h"

void pgo::PoseGraph::addOdomFactor(const geometry_msgs::Pose &pose)
{
    const auto gtsam_pose = util::rosPose2gtsamPose(pose);
    auto n_keyframes = keyframes_.size();

    if (keyframes_.empty())
    {
        auto variances = (gtsam::Vector(6) << 1e-2, 1e-2, M_PI * M_PI, 1e8, 1e8, 1e8).finished();
        pgo::PoseWithIdx poseWIdx(gtsam_pose, 0);
        optimizer_.addPrior(poseWIdx, variances);
    }
    else
    {
        auto variances = (gtsam::Vector(6) << 1e-1, 1e-1, 1e-1, 1e-1, 1e-1, 1e-1).finished();
        pgo::PoseFrom poseFrom(keyframes_.back(), n_keyframes - 1);
        pgo::PoseFrom poseTo(gtsam_pose, n_keyframes);
        optimizer_.addBetween(poseFrom, poseTo, variances);
    }

    keyframes_.push_back(gtsam_pose);
}

void pgo::PoseGraph::addGPSFactor(const geometry_msgs::PoseWithCovariance &pose)
{
//    if (keyframes_.size() < 2)
//    {
//        return;
//    }

//    auto last_keyframe_it = keyframes_.end();
//    if (util::pointDistance(*last_keyframe_it--, *last_keyframe_it) < gpsDistanceThreshold)
//    {
//        return;
//    }

    // TODO check data covariance

    double noise_x = pose.covariance[0];
    double noise_y = pose.covariance[7];
    double noise_z = pose.covariance[14];

    const auto &gps_x = pose.pose.position.x;
    const auto &gps_y = pose.pose.position.y;
    const auto &gps_z = pose.pose.position.z;

    //        if (!useGpsElevation) {
//            gps_z = transformTobeMapped[5];
//            noise_z = 0.01;
//        }

    if (abs(gps_x) < 1e-6 && abs(gps_y) < 1e-6)
    {
        ROS_DEBUG("Invalid gps_x and gps_y values");
        return;
    }

    gtsam::Vector noise(3);
    noise << noise_x, noise_y, noise_z;
    // Vector3 << max(noise_x, 1.0f), max(noise_y, 1.0f), max(noise_z, 1.0f);
    gtsam::noiseModel::Diagonal::shared_ptr gps_noise = gtsam::noiseModel::Diagonal::Variances(noise);
    gtsam::GPSFactor gps_factor(keyframes_.size() - 1,
                                gtsam::Point3(gps_x, gps_y, gps_z),
                                gps_noise);
    ROS_DEBUG_STREAM("Adding GPS constraint " << keyframes_.size() - 1);
    gps_keyframes_.push_back(gps_factor);

    if (gps_keyframes_.size() < 20) {
        ROS_DEBUG("Accumulated gps factor: %lu / 20", gps_keyframes_.size());
        return;
    }


    if (!gps_alignment_init_)
    {
        ROS_DEBUG("Aligning GPS and Odom frame");
        for (size_t i = 0; i < gps_keyframes_.size(); ++i)
        {
            gtsam::GPSFactor gpsFactor = gps_keyframes_.at(i);
            optimizer_.add(gpsFactor);
//            gpsIndexContainer[gpsFactor.key()] = i;
        }
        gps_alignment_init_ = true;
    }
    else
    {
        // After the coordinate systems are aligned, in theory, the GPS z and
        // the z estimated by the current LIO system should not be too
        // different. Otherwise, there is a problem with the quality of the
        // secondary GPS point.
        //                if (abs(gps_z - cloudKeyPoses3D->back().z) > 10.0) {
        //                    // ROS_WARN("Too large GNSS z noise %f", noise_z);
        //                    gtsam::Vector Vector3(3);
        //                    Vector3 << max(noise_x, 10000.0f), max(noise_y,
        //                    10000.0f), max(noise_z, 100000.0f);
        //                    // gps_noise =
        //                    gtsam::noiseModel::Diagonal::Variances(Vector3);
        //                    // gps_factor =
        //                    gtsam::GPSFactor(cloudKeyPoses3D->size(),
        //                    gtsam::Point3(gps_x, gps_y, gps_z),
        //                    // gps_noise);
        //                }
        // add loop constriant
        optimizer_.add(gps_factor);
    }
}

void pgo::PoseGraph::addLoopFactor()
{
    loop_closed_ = true;
//    if (loopIndexQueue.empty()) return;
//    
//    for (int i = 0; i < (int) loopIndexQueue.size(); ++i) {
//        int indexFrom = loopIndexQueue[i].first;
//        int indexTo = loopIndexQueue[i].second;
//        gtsam::Pose3 poseBetween = loopPoseQueue[i];
//        gtsam::gtsam::noiseModel::Diagonal::shared_ptr noiseBetween = loopNoiseQueue[i];
//        mtxGraph.lock();
//        graph_.add(
//            BetweenFactor<gtsam::Pose3>(indexFrom, indexTo, poseBetween, noiseBetween));
//        mtxGraph.unlock();
//    }
//    
//    loopIndexQueue.clear();
//    loopPoseQueue.clear();
//    loopNoiseQueue.clear();
//    aLoopIsClosed = true;
}

pgo::PoseGraph::PoseGraph() noexcept
{
    keyframes_.reserve(1000);
    gps_keyframes_.reserve(1000);
}

void pgo::PoseGraph::optimize()
{
    if (!gps_alignment_init_)
    {
        return;
    }

    optimizer_.optimize(loop_closed_);
}

void pgo::PoseGraph::correctKeyframes()
{
    if (!gps_alignment_init_)
    {
        return;
    }

    auto current_estimate = optimizer_.isam().calculateEstimate();
    auto estimate_size = current_estimate.size();
    if (estimate_size != keyframes_.size())
    {
        ROS_WARN_STREAM("estimate size does not match keyframe size something went wrong");
        return;
    }

    for (size_t i = 0; i < estimate_size; ++i)
    {
        const auto &estimate = current_estimate.at<gtsam::Pose3>(i);
        auto &current_keyframe = keyframes_[i];
        current_keyframe = gtsam::Pose3::Create(estimate.rotation(), estimate.translation());
    }
}
