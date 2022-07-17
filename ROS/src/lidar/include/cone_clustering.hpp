#ifndef CONECLUSTERING_HPP
#define CONECLUSTERING_HPP

#include "sensor_msgs/PointCloud.h"
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

namespace ns_lidar
{
    class ConeClustering
    {

    public:
        ConeClustering(ros::NodeHandle &n);

        sensor_msgs::PointCloud euclidianClustering(
            const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud);
        sensor_msgs::PointCloud stringClustering(
            const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud);

    private:
        ros::NodeHandle &n_;

        double cluster_tolerance_; // The cone clustering tolerance (m)

        float hypot3d(float a, float b, float c);
    };
}

#endif
