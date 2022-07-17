#include <algorithm>
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
    sensor_msgs::PointCloud ConeClustering::euclidianClustering(
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

    /**
     * @brief Clusters the cones in the final filtered point cloud and generates a ROS message.
     *
     * This time using String Clustering.
     * @refer Broström, Carpenfelt
     * 
     * @attention This code currently DOES NOT WORK
     *
     */
    sensor_msgs::PointCloud ConeClustering::stringClustering(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud)
    {
        std::vector<pcl::PointCloud<pcl::PointXYZI>> clusters;
        std::vector<pcl::PointXYZI> cluster_rightmost; // The rightmost point in each cluster

        // Iterate over all points, up and down, left to right (elevation & azimuth)
        for (uint16_t i = 0; i < cloud->points.size(); i++)
        {
            pcl::PointXYZI point = cloud->points[i];

            std::vector<uint16_t> clusters_to_keep; // These cluster ids will be kept at the end of this clustering because they can still contain a cone

            bool found_cluster = false;

            // Iterate over the rightmost point in each cluster
            for (uint16_t cluster_id = 0; cluster_id < cluster_rightmost.size(); cluster_id++)
            {
                pcl::PointXYZI rightmost = cluster_rightmost[cluster_id];

                // This distance should be calculated using max(delta_azi, delta_r)
                float r_point = ConeClustering::hypot3d(point.x, point.y, point.z);
                float r_rightmost = ConeClustering::hypot3d(rightmost.x, rightmost.y, rightmost.z);
                float delta_r = std::abs(r_point - r_rightmost);
                float delta_arc = std::abs(std::atan2(point.x, point.y) - atan2(rightmost.x, rightmost.y)) * r_rightmost;
                float dist = std::max(delta_r, delta_arc);

                // A cone is max 285mm wide, check whether this point is within that distance from the rightmost point in each cluster
                if (dist < 0.285)
                {
                    found_cluster = true;

                    // Add the point to the cluster
                    clusters[cluster_id].push_back(point);

                    // Check whether this point is the rightmost point in the cluster
                    if (point.y < cluster_rightmost[cluster_id].y)
                    {
                        cluster_rightmost[cluster_id] = point;
                    }

                    // Everytime a cluster is updated, check whether it can still be a cone
                    Eigen::Vector4f min;
                    Eigen::Vector4f max;
                    pcl::getMinMax3D(clusters[cluster_id], min, max);

                    float bound_x = std::fabs(max[0] - min[0]);
                    float bound_y = std::fabs(max[1] - min[1]);
                    float bound_z = std::fabs(max[2] - min[2]);

                    // Filter based on the shape of cones
                    if (bound_x < 0.5 && bound_y < 0.5 && bound_z < 0.4)
                    {
                        // This cluster can still be a cone
                        clusters_to_keep.push_back(cluster_id);
                    }
                }
            }

            // Remove clusters that cannot represent a cone
            std::vector<pcl::PointCloud<pcl::PointXYZI>> new_clusters;
            std::vector<pcl::PointXYZI> new_cluster_rightmost;
            std::sort(clusters_to_keep.begin(), clusters_to_keep.end()); // Sort the index array
            for (uint16_t id : clusters_to_keep)
            {
                new_clusters.push_back(clusters[id]);
                new_cluster_rightmost.push_back(cluster_rightmost[id]);
            }

            clusters = new_clusters;
            cluster_rightmost = new_cluster_rightmost;

            // We could not find a cluster where we can add the point to, so create a new one
            if (!found_cluster)
            {
                pcl::PointCloud<pcl::PointXYZI> pc;
                pc.points.push_back(point);
                clusters.push_back(pc);
                cluster_rightmost.push_back(point);
            }
        }

        // From the clusters we found, now generate their centroid and add it to the ROS message

        // Create a PC and channel for the cone colour
        sensor_msgs::PointCloud cluster_msg;
        sensor_msgs::ChannelFloat32 cone_channel;
        cone_channel.name = "cone_type";

        for (pcl::PointCloud<pcl::PointXYZI> cluster : clusters)
        {
            Eigen::Vector4f centroid;
            Eigen::Vector4f min;
            Eigen::Vector4f max;
            pcl::compute3DCentroid(cluster, centroid);

            // Filter based on the shape of cones
            if (centroid[2] < 0.4)
            {
                geometry_msgs::Point32 cone_pos;
                cone_pos.x = centroid[0];
                cone_pos.y = centroid[1];
                cone_pos.z = centroid[2];
                cluster_msg.points.push_back(cone_pos);
                cone_channel.values.push_back(0); // TODO actually get the intensity
            }
        }
        cluster_msg.channels.push_back(cone_channel);

        return cluster_msg;
    }

    float ConeClustering::hypot3d(float a, float b, float c)
    {
        return std::sqrt(a * a + b * b + c * c);
    }
}
