#include <ros/ros.h>
#include "cone_clustering.hpp"

namespace ns_lidar
{
    // Constructor
    ConeClustering::ConeClustering(ros::NodeHandle &n) : n_(n)
    {
        // Get parameters
        n.param<double>("cluster_tolerance", cluster_tolerance_, 0.5);
    }

    /**
     * @brief Clusters the cones in the final filtered point cloud and generates a ROS message.
     *
     */
    sensor_msgs::PointCloud ConeClustering::coneClustering(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud)
    {
        // Create a PC and channel for the cone colour
        sensor_msgs::PointCloud cluster;
        sensor_msgs::ChannelFloat32 cone_channel;
        cone_channel.name = "cone_type";

        // Creating the KdTree object for the search method of the extraction
        pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(
            new pcl::search::KdTree<pcl::PointXYZI>);
        tree->setInputCloud(cloud);

        // Define the parameters for Euclidian clustering
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
        ec.setClusterTolerance(cluster_tolerance_);
        ec.setMinClusterSize(2);
        ec.setMaxClusterSize(200);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);

        // Iterate over all cluster indices
        for (const auto &iter : cluster_indices)
        {
            // Create PC of the current cone cluster
            pcl::PointCloud<pcl::PointXYZI>::Ptr cone(new pcl::PointCloud<pcl::PointXYZI>);
            for (auto it : iter.indices)
            {
                cone->points.push_back(cloud->points[it]);
            }
            cone->width = cone->points.size();
            cone->height = 1;
            cone->is_dense = true;

            Eigen::Vector4f centroid;
            Eigen::Vector4f min;
            Eigen::Vector4f max;
            pcl::compute3DCentroid(*cone, centroid);
            pcl::getMinMax3D(*cone, min, max);

            float bound_x = std::fabs(max[0] - min[0]);
            float bound_y = std::fabs(max[1] - min[1]);
            float bound_z = std::fabs(max[2] - min[2]);

            // filter based on the shape of cones
            if (bound_x < 0.5 && bound_y < 0.5 && bound_z < 0.4 && centroid[2] < 0.4)
            {
                geometry_msgs::Point32 cone_pos;
                cone_pos.x = centroid[0];
                cone_pos.y = centroid[1];
                cone_pos.z = centroid[2];
                cluster.points.push_back(cone_pos);
                cone_channel.values.push_back(0); // TODO actually get the intensity
            }
        }

        cluster.channels.push_back(cone_channel);

        return cluster;
    }
}
