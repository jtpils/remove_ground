#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <iostream>
#include "laser_handle.h"

const int N_SCANS = 16;

Eigen::Vector3f ground(0, 0, 1);

int num = 0;

void callback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr laser_ptr(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*msg, *laser_ptr);

    std::vector<pcl::PointCloud<pcl::PointXYZI> > scan(N_SCANS);
    for(int i = 0; i < laser_ptr->points.size(); i ++)
    {
        pcl::PointXYZI point = laser_ptr->points[i];
        float dis = sqrt(pow(point.x, 2) + pow(point.y,2 ) + pow(point.z, 2));

        if((point.x > -20 && point.x < 0.5 && point.y > -0.5 && point.y < 3.0) || dis > 50.0)
        {
            continue;
        }
        int scan_id = round(atan2(point.z, sqrt(pow(point.x, 2) + pow(point.y, 2))) * 180.0 / M_PI);
        if(scan_id < 0)
        {
            scan_id += (N_SCANS - 1);
        }
        if(scan_id < 0 || scan_id > N_SCANS - 1)
        {
            continue;
        }
        point.intensity = scan_id;
        scan[scan_id].push_back(point);
    }

    laser_ptr->clear();
    for(int i = 0; i < N_SCANS; i ++)
    {
        (*laser_ptr) += scan[i];
    }

    LaserHandle lh(*laser_ptr);
    lh.setgroundNormal(ground);
    lh.setLidarHeight(1.7);
    lh.setEpsAngle(10.0);
    pcl::PointCloud<pcl::PointXYZI> newLaser;
    lh.removeground(newLaser);

    num ++;
    std::stringstream ss;
    ss << num;
    ss << "_laser.pcd";
    pcl::io::savePCDFile(ss.str(), newLaser);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "without_ground");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points_200", 100, callback);
    
    while(ros::ok())
    {
        ros::spinOnce();
    }

    return 0;
}

