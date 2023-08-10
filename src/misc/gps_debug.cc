#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>

#include "dliom/gnss.h"

class GNSSDebug {
public:
    GNSSDebug() {
        ros::NodeHandle nh;

        navsat_sub_ = nh.subscribe("gps", 10, &GNSSDebug::navSatCallback, this);
        marker_pub_ = nh.advertise<visualization_msgs::Marker>("gps_marker", 10);
        path_pub_ = nh.advertise<nav_msgs::Path>("gps_path", 10);

        marker_.header.frame_id = "map";
        marker_.ns = "gps_debug";
        marker_.id = 0;
        marker_.type = visualization_msgs::Marker::SPHERE;
        marker_.action = visualization_msgs::Marker::ADD;
        marker_.color.r = 1.0;
        marker_.color.a = 1.0;
    }

    void navSatCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
        if (!first_msg_) {
          gnss_.resetOrigin(msg->latitude, msg->longitude, msg->altitude);
        }

        gnss_.update(msg->latitude, msg->longitude, msg->altitude);

        geometry_msgs::Pose pose;
        pose.orientation.w = 1;
        pose.position.x = gnss_.e();
        pose.position.y = gnss_.n();
        pose.position.z = gnss_.u();

        marker_.header.stamp = msg->header.stamp;
        marker_.pose = pose;
        marker_.scale.x = std::min(msg->position_covariance[0] * 10000., 30.);
        marker_.scale.y = std::min(msg->position_covariance[4] * 10000., 30.);
        marker_.scale.z = std::min(msg->position_covariance[8] * 10000., 30.);
        marker_pub_.publish(marker_);

        geometry_msgs::PoseStamped p;
        p.header.stamp = msg->header.stamp;
        p.header.frame_id = target_frame;
        p.pose = pose;
        path_.header.stamp = msg->header.stamp;
        path_.header.frame_id = target_frame;
        path_.poses.push_back(p);
        path_pub_.publish(path_);

        first_msg_ = true;
    }

private:
    visualization_msgs::Marker marker_ {};
    nav_msgs::Path path_ {};
    dliom::LocalCartesianGNSS gnss_ {};
    bool first_msg_ = false;
    std::string target_frame {"map"};
    
    ros::Subscriber navsat_sub_;
    ros::Publisher marker_pub_;
    ros::Publisher path_pub_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "gps_debug_node");
    GNSSDebug gps {};
    ros::spin();
    return 0;
}

