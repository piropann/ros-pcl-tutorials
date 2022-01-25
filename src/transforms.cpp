//座標変換
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>    //Passthroughライブラリ
#include <iostream>
#include <pcl/common/transforms.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr loadPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string filename) {
    //ファイルパスの取得
    std::string file_path;
    file_path = ros::package::getPath("ros-pcl-tutorials") + "/data/" + filename ;
    pcl::io::loadPCDFile(file_path,*cloud);
    return cloud;
} 
 
pcl::PointCloud<pcl::PointXYZRGB>::Ptr loadPointCloudRGB(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::string filename) {
    //ファイルパスの取得
    std::string file_path;
    file_path = ros::package::getPath("ros-pcl-tutorials") + "/data/" + filename ;
    pcl::io::loadPCDFile(file_path,*cloud);
    return cloud;
} 


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
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

    std::string file_path;
    std::string file_name = "sample.pcd";

    //点群読み込み
    cloud = loadPointCloudRGB(cloud,file_name);

    //処理をしたあとの点群の格納場所を作成
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);

    //行列を使用して座標変換
    Eigen::Matrix4d transformation_matrix;
    transformation_matrix << 1, 0, 0, 0,
                             0, 0, 1, 0,
                             -1, -1, 1, 0,
                             0, 0, 0, 1;
    pcl::transformPointCloud(*cloud, *cloud_filtered, transformation_matrix);

    //sensor_msgs::PointCloud2 cloud_ros;
    pcl::toROSMsg(*cloud,cloud_ros);
    pcl::toROSMsg(*cloud_filtered,cloud_ros2);
    cloud_ros.header.frame_id = "base_link";
    cloud_ros2.header.frame_id = "base_link";

    pcl::io::savePCDFileBinary(ros::package::getPath("ros-pcl-tutorials") + "/data/" + "sample" + "1.pcd", *cloud_filtered);

    ros::Rate loop_rate(1);
    while(ros::ok()){
        cloud_pub.publish(cloud_ros);
        cloud_pub2.publish(cloud_ros2);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}