#include "cloud_logger_node.h"
#include "pcl_conversions.h"
#include "point_xyzir.hpp"

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include <string_view>
#include <string>
#include <experimental/filesystem>

namespace fs = std::experimental::filesystem;

namespace {
constexpr std::string_view NODE_NAME = "cloud_logger_node";
constexpr std::string_view TOPIC_PARAM = "topic";
constexpr std::string_view OUTPUT_FOLDER = "output_path";
constexpr std::string_view PCD_FILE_FORMAT = ".pcd";
constexpr uint32_t ROS_QUEUE_SIZE = 10;

bool create_folder(const std::string& folder_name){
    const fs::path folder_path(folder_name);
    if (!fs::exists(folder_path)){
        return fs::create_directories(folder_path);
    }
    return true;
}

template<typename PointT>
bool save_cloud(const std::string& file_name, const pcl::PointCloud<PointT>& cloud)
{
    return !pcl::io::savePCDFileASCII(file_name, cloud);
}

}

CloudLogger::CloudLogger(ros::NodeHandle& nh)
: output_path_(nh.param<std::string>(std::string(OUTPUT_FOLDER), ""))
{
    const auto topic_name = nh.param<std::string>(std::string(TOPIC_PARAM), "");
    topic_subscriber_ = nh.subscribe<sensor_msgs::PointCloud2>(topic_name, ROS_QUEUE_SIZE, &CloudLogger::callback_, this);
    if (!create_folder(output_path_)){
        throw std::logic_error("Can't create directory: " + output_path_.string());
    }
}

void CloudLogger::callback_(const sensor_msgs::PointCloud2ConstPtr& cloud_message){
    pcl::PointCloud<pcl::PointXYZIR>::Ptr p_cloud(new pcl::PointCloud<pcl::PointXYZIR>());
    fromROSMsg(*cloud_message, *p_cloud);
    const auto file_name = std::to_string(frame_number_) + std::string(PCD_FILE_FORMAT);
    const auto full_file_name = output_path_ / file_name;
    if (save_cloud(full_file_name.string(), *p_cloud)){
        ROS_INFO_STREAM("Saved frame: " << frame_number_++);
    } else {
        ROS_INFO_STREAM("Can't save frame: " << frame_number_);
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, std::string(NODE_NAME));
    ros::NodeHandle private_nh("~");
    try{
        CloudLogger logger(private_nh);
        ros::spin();
    } catch (std::exception& e){
        ROS_INFO_STREAM(e.what());
    }
    return 0;
}