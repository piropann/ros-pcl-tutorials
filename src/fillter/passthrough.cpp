#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <iostream>
#include <pcl/common/transforms.h>

int main(int argc, char **argv)
{
    ros::init (argc, argv, "pub_pcl");
    ros::NodeHandle nh;

    sensor_msgs::PointCloud2 cloud_ros;
    sensor_msgs::PointCloud2 cloud_ros2;
    ros::Publisher cloud_pub = n.advertise<sensor_msgs::PointCloud2>("input",1);
    ros::Publisher cloud_pub2 = n.advertise<sensor_msgs::PointCloud2>("output",1);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    std::string filepath;
    filepath = ros::package::getPath("pcl_ros_tutorial") + "/data/" + filename;
    std::string filename = "out.pcd";
    bool save = false; 
    pcl::io::loadPCDFile(filename, *cloud);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits(0, 1.0);
    pass.filter(*cloud_filtered);

    //sensor_msgs::PointCloud2 cloud_ros;
    pcl::toROSMsg(*cloud,cloud_ros);
    pcl::toROSMsg(*cloud_filtered,cloud_ros2);
    cloud_ros.header.frame_id = "base_link";
    cloud_ros2.header.frame_id = "base_link";

    pcl::io::savePCDFileBinary(ros::package::getPath("pointcloud_registration") + "/data/f_" + filename , *cloud_filtered);

    ros::Rate loop_rate(1);
    while(ros::ok()){
        cloud_pub.publish(cloud_ros);
        cloud_pub2.publish(cloud_ros2);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}