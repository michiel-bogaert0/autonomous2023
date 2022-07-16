#ifndef LIDAR_HPP
#define LIDAR_HPP

#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Dense>

typedef struct
{
    Eigen::MatrixXf normal_n;
    double d = 0.;
} model_t;

class Lidar
{

public:
    Lidar(ros::NodeHandle &n);

private:
    ros::NodeHandle n_;
    ros::Subscriber rawLidarSubscriber_;
    ros::Publisher preprocessedLidarPublisher_;
    ros::Publisher groundRemovalLidarPublisher_;
    ros::Publisher clusteredLidarPublisher_;
    ros::Publisher conePublisher_;

    int num_iter_;         // Number of iterations
    int num_lpr_;          // number of points used to estimate the LPR: Lowest Point Representative -> a point defined as the average of the num_lpr_ lowest points
    double th_seeds_;      // Threshold for points to be considered initial seeds
    double th_dist_;       // Threshold distance from the plane
    double sensor_height_; // LiDAR sensor height to ground (m)

    double cluster_tolerance_; // The cone clustering tolerance (m)

    void rawPcCallback(const sensor_msgs::PointCloud2 &msg);
    void preprocessing(
        const pcl::PointCloud<pcl::PointXYZI> &raw,
        pcl::PointCloud<pcl::PointXYZI>::Ptr &preprocessed_pc);
    void groundRemoval(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in,
                       pcl::PointCloud<pcl::PointXYZI>::Ptr notground_points,
                       pcl::PointCloud<pcl::PointXYZI>::Ptr ground_points);
    model_t estimatePlane(const pcl::PointCloud<pcl::PointXYZI> &seed_points);
    void extractInitialSeeds(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in, const pcl::PointCloud<pcl::PointXYZI>::Ptr seed_points);
    sensor_msgs::PointCloud coneClustering(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud);
    void publishMarkers(const sensor_msgs::PointCloud cones);
};

#endif
