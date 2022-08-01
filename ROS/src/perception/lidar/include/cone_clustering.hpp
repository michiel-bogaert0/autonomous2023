#ifndef CONECLUSTERING_HPP
#define CONECLUSTERING_HPP

#include "geometry_msgs/Point32.h"
#include "sensor_msgs/PointCloud.h"
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

namespace ns_lidar
{
    typedef struct
    {
        geometry_msgs::Point32 pos;
        bool is_cone;
    } ConeCheck;

    class ConeClustering
    {

    public:
        ConeClustering(ros::NodeHandle &n);

        sensor_msgs::PointCloud cluster(
            const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
            const pcl::PointCloud<pcl::PointXYZI>::Ptr &ground);

    private:
        ros::NodeHandle &n_;

        std::string clustering_method_; // Default: euclidian, others: string
        double cluster_tolerance_; // The cone clustering tolerance (m)
        double point_count_theshold_; // How much % can the cone point count prediction be off from the actual count 

        sensor_msgs::PointCloud euclidianClustering(
            const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
            const pcl::PointCloud<pcl::PointXYZI>::Ptr &ground);
        sensor_msgs::PointCloud stringClustering(
            const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud);
        ConeCheck isCloudCone(pcl::PointCloud<pcl::PointXYZI> cone);
        float hypot3d(float a, float b, float c);
    };
}

#endif
