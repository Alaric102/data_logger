#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <string_view>
#include <string>
#include <experimental/filesystem>

class CloudLogger
{
public:
    CloudLogger(ros::NodeHandle& nh);
private:
    void callback_(const sensor_msgs::PointCloud2ConstPtr& cloud_message);
private:
    ros::Subscriber topic_subscriber_;
    size_t frame_number_ = 0u;
    const std::experimental::filesystem::path output_path_;
};