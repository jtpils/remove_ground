#ifndef LASER_HANDLE_H
#define LASER_HANDLE_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class LaserHandle
{
public:
    LaserHandle();
    LaserHandle(pcl::PointCloud<pcl::PointXYZI> laser);
    void setInputCloud(pcl::PointCloud<pcl::PointXYZI> laser);
    void setgroundNormal(Eigen::Vector3f ground);
    void setEpsAngle(double delta);
    void setLidarHeight(double height);
    void removeground(pcl::PointCloud<pcl::PointXYZI> &laser);
    ~LaserHandle();

private:

    pcl::PointCloud<pcl::PointXYZI>::Ptr laser_ptr_;
    Eigen::Vector3f ground_norm_;
    double delta_;
    double height_;

    pcl::SACSegmentation<pcl::PointXYZI> segment_;
    pcl::ExtractIndices<pcl::PointXYZI> filters_;
};

#endif

