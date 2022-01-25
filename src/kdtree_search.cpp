//kdtree
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <iostream>

int main(int argc, char **argv)
{
    ros::init (argc, argv, "pub_pcl");
    ros::NodeHandle nh;

    srand (time (NULL));

    //ROSでPCLを使う場合、パブリッシャーとサブスクライバーを使い、トピックを変換して処理する
    //Pub/Subの設定
    sensor_msgs::PointCloud2 cloud_ros;
    ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("input",1);
    ros::Publisher cloud_pub2 = nh.advertise<sensor_msgs::PointCloud2>("output",1);

    //点群の格納場所を作成
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    //点群ランダム生成
    cloud->width = 1000;
    cloud->height = 1;
    cloud->points.resize (cloud->width * cloud->height);
    for (std::size_t i = 0; i < cloud->size (); ++i)
    {
    (*cloud)[i].x = 1024.0f * rand () / (RAND_MAX + 1.0f);
    (*cloud)[i].y = 1024.0f * rand () / (RAND_MAX + 1.0f);
    (*cloud)[i].z = 1024.0f * rand () / (RAND_MAX + 1.0f);
    }
    
    //kdtreeの実装
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud (cloud);//処理する点群
    
    pcl::PointXYZ searchPoint;

    searchPoint.x = 1024.0f * rand () / (RAND_MAX + 1.0f);
    searchPoint.y = 1024.0f * rand () / (RAND_MAX + 1.0f);
    searchPoint.z = 1024.0f * rand () / (RAND_MAX + 1.0f);

    // 選ばれた点に隣接するK個の点群を検索

    int K = 10;

    std::vector<int> pointIdxKNNSearch(K);
    std::vector<float> pointKNNSquaredDistance(K);
    
    std::cout << "K nearest neighbor search at (" << searchPoint.x 
                << " " << searchPoint.y 
                << " " << searchPoint.z
                << ") with K=" << K << std::endl;

    if ( kdtree.nearestKSearch (searchPoint, K, pointIdxKNNSearch, pointKNNSquaredDistance) > 0 )
    {
        for (std::size_t i = 0; i < pointIdxKNNSearch.size (); ++i)
            std::cout << "    "  <<   (*cloud)[ pointIdxKNNSearch[i] ].x 
                    << " " << (*cloud)[ pointIdxKNNSearch[i] ].y 
                    << " " << (*cloud)[ pointIdxKNNSearch[i] ].z 
                    << " (squared distance: " << pointKNNSquaredDistance[i] << ")" << std::endl;
    }

    // 半径で探索を行う

    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    float radius = 256.0f * rand () / (RAND_MAX + 1.0f);

    std::cout << "Neighbors within radius search at (" << searchPoint.x 
                << " " << searchPoint.y 
                << " " << searchPoint.z
                << ") with radius=" << radius << std::endl;


    if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
    {
        for (std::size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
            std::cout << "    "  <<   (*cloud)[ pointIdxRadiusSearch[i] ].x 
                        << " " << (*cloud)[ pointIdxRadiusSearch[i] ].y 
                        << " " << (*cloud)[ pointIdxRadiusSearch[i] ].z 
                        << " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
    }

    pcl::toROSMsg(*cloud,cloud_ros);
    cloud_ros.header.frame_id = "base_link";

    ros::Rate loop_rate(1);
    while(ros::ok()){
        cloud_pub.publish(cloud_ros);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}