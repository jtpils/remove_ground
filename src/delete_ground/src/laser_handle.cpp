#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <iostream>
#include "laser_handle.h"

//构造函数
LaserHandle::LaserHandle()
{
    segment_.setOptimizeCoefficients(true);
    segment_.setModelType(pcl::SACMODEL_PLANE);
    segment_.setMethodType(pcl::SAC_RANSAC);
    segment_.setDistanceThreshold(0.2);
    segment_.setMaxIterations(500);
    segment_.setProbability(0.8);

    filters_.setNegative(true);
}

//析构函数
LaserHandle::~LaserHandle()
{}

//构造函数
LaserHandle::LaserHandle(const pcl::PointCloud<pcl::PointXYZI> laser)
{
    laser_ptr_ = laser.makeShared();
    
    //分割平面实例参数设置
    segment_.setOptimizeCoefficients(true);
    segment_.setModelType(pcl::SACMODEL_PLANE);
    segment_.setMethodType(pcl::SAC_RANSAC);
    segment_.setDistanceThreshold(0.3);
    segment_.setMaxIterations(500);
    segment_.setProbability(0.8);

    //过滤器参数设置
    filters_.setNegative(true);
}

//设置点云
void LaserHandle::setInputCloud(pcl::PointCloud<pcl::PointXYZI> laser)
{
    laser_ptr_ = laser.makeShared();
}

//设置可容忍角度偏差
void LaserHandle::setEpsAngle(double delta)
{
    delta_ = delta;
}

//设置地面法线(0,0,1) z轴上
void LaserHandle::setgroundNormal(Eigen::Vector3f ground)
{
    ground_norm_ = ground;
}

//设置雷达安装高度
void LaserHandle::setLidarHeight(double height)
{
    height_ = height;
}

//剔除地面
void LaserHandle::removeground(pcl::PointCloud<pcl::PointXYZI> &laser)
{
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

    while(true)
    {
        segment_.setInputCloud(laser_ptr_);
        segment_.segment(*inliers, *coefficients);

        if(inliers->indices.size() < 100)
        {
            break;
        }

        Eigen::Vector3f normal(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
        float angle = acos(normal.dot(ground_norm_)) * 180.0 / M_PI;

        if((angle > delta_ && angle < (180.0 - delta_)) || -(coefficients->values[3] / coefficients->values[2]) > -height_ * 0.95)
        {
            pcl::PointCloud<pcl::PointXYZI> plane;
            pcl::copyPointCloud(*laser_ptr_, inliers->indices, plane);
            laser += plane;
        }

        filters_.setInputCloud(laser_ptr_);
        filters_.setIndices(inliers);
        filters_.filter(*laser_ptr_);
    }

    laser += *laser_ptr_;
}
