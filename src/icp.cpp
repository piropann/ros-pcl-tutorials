//ICP
//反復最接近点アルゴリズムを使用し、２つの点群が座標変換されてるか判断する。
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h> 
#include <pcl/io/ply_io.h>

#include <iostream>
#include <string>


int main(int argc, char **argv)
{
    ros::init (argc, argv, "pub_pcl");
    ros::NodeHandle nh;

    //ROSでPCLを使う場合、パブリッシャーとサブスクライバーを使い、トピックを変換して処理する
    //Pub/Subの設定
    sensor_msgs::PointCloud2 cloud_ros;
    sensor_msgs::PointCloud2 cloud_ros2;
    ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("input",1);
    ros::Publisher cloud_pub2 = nh.advertise<sensor_msgs::PointCloud2>("output",1);

    //点群の格納場所を作成
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
    //処理をしたあとの点群の格納場所を作成
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
    
    //点群の作成
    cloud_in->width = 10;
    cloud_in->height = 1;
    cloud_in->points.resize (cloud_in->width * cloud_in->height);

    for (auto& point : *cloud_in)
    {
        point.x = 1024 * rand() / (RAND_MAX + 1.0f);
        point.y = 1024 * rand() / (RAND_MAX + 1.0f);
        point.z = 1024 * rand() / (RAND_MAX + 1.0f);
    }

    std::cout << "Saved " << cloud_in->size () << " data points to input:" << std::endl;
    
    for (auto& point : *cloud_in)
    std::cout << point << std::endl;
    
    *cloud_out = *cloud_in;

    //ICPで使用する点群の作成
    std::cout << "size:" << cloud_out->size() << std::endl;
    for (auto& point : *cloud_out)
        point.x += 0.7f;

    std::cout << "Transformed " << cloud_in->size () << " data points:" << std::endl;
      
    for (auto& point : *cloud_out)
        std::cout << point << std::endl;

    //ICP設定
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(cloud_in);//1つ目の座標
    icp.setInputTarget(cloud_out);//2つ目の座標

    pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(Final);

    //ICPで求まった座標の表示
    std::cout << "has converged:" << icp.hasConverged() << " score: " <<
    icp.getFitnessScore() << std::endl;
    std::cout << icp.getFinalTransformation() << std::endl;

    //sensor_msgs::PointCloud2 cloud_ros;
    pcl::toROSMsg(*cloud_in,cloud_ros);
    pcl::toROSMsg(*cloud_out,cloud_ros2);
    cloud_ros.header.frame_id = "base_link";
    cloud_ros2.header.frame_id = "base_link";


    ros::Rate loop_rate(1);
    while(ros::ok()){
        cloud_pub.publish(cloud_ros);
        cloud_pub2.publish(cloud_ros2);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}