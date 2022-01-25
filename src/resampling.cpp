//平滑化と法線推定
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <iostream>

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
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    std::string file_path;
    std::string file_name = "bun0.pcd";

    //点群読み込み
    //cloud = loadPointCloud(cloud,file_name);
    pcl::io::loadPCDFile ("bun0.pcd", *cloud);
    
    //kdtreeを作成
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

    //処理をしたあとの点群の格納場所を作成(法線情報を出すためNormal型)
    pcl::PointCloud<pcl::PointNormal> cloud_mts; //(new pcl::PointCloud<pcl::PointNormal>);

    
    //入力する点群のタイプと出力する点群のタイプの指定
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;

    //mls.setComputeNormals (true);

    //平滑化の実装
    mls.setInputCloud (cloud);//処理する点群
    mls.setPolynomialOrder (2);//次数の設定
    mls.setSearchMethod (tree);
    mls.setSearchRadius (0.03);//最近傍の半径設定

    mls.process (cloud_mts);//出力

    pcl::toROSMsg(*cloud,cloud_ros);
    pcl::toROSMsg(cloud_mts,cloud_ros2);
    cloud_ros.header.frame_id = "base_link";
    cloud_ros2.header.frame_id = "base_link";

    pcl::io::savePCDFileBinary(ros::package::getPath("ros-pcl-tutorials") + "/data/result/resampling_" + file_name , cloud_mts);


    ros::Rate loop_rate(1);
    while(ros::ok()){
        cloud_pub.publish(cloud_ros);
        cloud_pub2.publish(cloud_ros2);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}