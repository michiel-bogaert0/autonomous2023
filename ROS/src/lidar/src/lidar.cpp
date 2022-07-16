#include <ros/ros.h>
#include "lidar.hpp"

// Constructor
Lidar::Lidar(ros::NodeHandle &n) : n_(n)
{
    // Subscribe to the raw lidar topic
    rawLidarSubscriber_ = n.subscribe("perception/raw_pc", 10, &Lidar::rawPcCallback, this);

    // Publish to the filtered and clustered lidar topic
    clusteredLidarPublisher_ = n.advertise<sensor_msgs::PointCloud>("perception/clustered_pc", 5);

    // Get parameters
    n.param<int>("~num_iter", num_iter_, 3);
    n.param<int>("~num_lpr", num_lpr_, 250);
    n.param<double>("~th_seeds", th_seeds_, 1.2);
    n.param<double>("~th_dist", th_dist_, 0.3);
    n.param<double>("~sensor_height", sensor_height_, 0.3);
    n.param<double>("~cluster_tolerance", cluster_tolerance_, 0.5);
}

void Lidar::rawPcCallback(const sensor_msgs::PointCloud2 &msg)
{
    /**
     * @brief Subscribes to the LIDAR raw pointcloud topic and processes the data
     *
     * @arg msg: the PointCloud2 message
     */

    // Create PC objects
    pcl::PointCloud<pcl::PointXYZI> raw_pc_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr preprocessed_pc(
        new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(msg, raw_pc_);

    // Preprocessing
    preprocessing(raw_pc_, preprocessed_pc);

    // Ground plane removal
    pcl::PointCloud<pcl::PointXYZI>::Ptr notground_points(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr ground_points(new pcl::PointCloud<pcl::PointXYZI>());
    groundRemoval(preprocessed_pc, notground_points, ground_points);

    // Cone clustering
    sensor_msgs::PointCloud cluster;
    cluster = coneClustering(notground_points);
    cluster.header.stamp = msg.header.stamp;

    clusteredLidarPublisher_.publish(cluster);
}

void Lidar::preprocessing(
    const pcl::PointCloud<pcl::PointXYZI> &raw,
    pcl::PointCloud<pcl::PointXYZI>::Ptr &preprocessed_pc)
{
    /**
     * @brief Preprocesses the given raw point cloud.
     *
     * Cleans up points containing the car itself and does ground plane removal.
     *
     * @arg raw: the raw PC
     * @arg preprocessed_pc: the resulting filtered PC
     *
     */

    // Clean up the points belonging to the car and noise in the sky
    for (auto &iter : raw.points)
    {
        // Remove points closer than 2m, higher than 0.7m or further than 20m
        if (std::hypot(iter.x, iter.y) < sqrt(2) || iter.z > 0.7 || std::hypot(iter.x, iter.y) > sqrt(20))
            continue;
        preprocessed_pc->points.push_back(iter);
    }
}

void Lidar::groundRemoval(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in,
                          pcl::PointCloud<pcl::PointXYZI>::Ptr notground_points,
                          pcl::PointCloud<pcl::PointXYZI>::Ptr ground_points)
{
    /**
     * @brief Removes ground points
     *
     * @refer: Based on the code https://github.com/chrise96/3D_Ground_Segmentation,
     *   which in turn is based on a paper by Zermas et al.
     *   "Fast segmentation of 3D point clouds: a paradigm on LIDAR data for Autonomous Vehicle Applications"
     *
     */

    pcl::PointCloud<pcl::PointXYZI>::Ptr seed_points(new pcl::PointCloud<pcl::PointXYZI>());

    // 1. Extract initial ground seeds
    extractInitialSeeds(cloud_in, seed_points);

    // 2. Ground plane fit mainloop
    // The points belonging to the ground surface are used as seeds
    // for the refined estimation of a new plane model and the process
    // repeats for num_iter_ number of times
    for (int i = 0; i < num_iter_; ++i)
    {
        /// 3. Estimate the plane model
        model_t model = estimatePlane(*seed_points);

        ground_points->clear();
        notground_points->clear();

        // Pointcloud to matrix
        Eigen::MatrixXf points_matrix(cloud_in->points.size(), 3);
        size_t j = 0u;
        for (auto p : (*cloud_in).points)
        {
            points_matrix.row(j++) << p.x, p.y, p.z;
        }

        // Ground plane model
        Eigen::VectorXf result = points_matrix * model.normal_n;

        // Threshold filter: N^T xi + d = dist < th_dist ==> N^T xi < th_dist - d
        double th_dist_d_ = th_dist_ - model.d;

        // 4. Identify which of the points belong to the ground and non ground
        for (int k = 0; k < result.rows(); ++k)
        {
            if (result[k] < th_dist_d_)
            {
                // TODO think about a more optimized code for this part
                pcl::PointXYZI point;
                point.x = cloud_in->points[k].x;
                point.y = cloud_in->points[k].y;
                point.z = cloud_in->points[k].z;
                point.intensity = cloud_in->points[k].intensity;
                ground_points->points.push_back(point);
            }
            else
            {
                pcl::PointXYZI point;
                point.x = cloud_in->points[k].x;
                point.y = cloud_in->points[k].y;
                point.z = cloud_in->points[k].z;
                point.intensity = cloud_in->points[k].intensity;
                notground_points->points.push_back(point);
            }
        }
    }
}

model_t Lidar::estimatePlane(const pcl::PointCloud<pcl::PointXYZI> &seed_points)
{
    /**
     * @brief Use the set of seed points to estimate the initial plane model
     * of the ground surface.
     */

    Eigen::Matrix3f cov_matrix(3, 3);
    Eigen::Vector4f points_mean;
    model_t model;

    // Compute covariance matrix in single pass
    pcl::computeMeanAndCovarianceMatrix(seed_points, cov_matrix, points_mean);

    // Singular Value Decomposition: SVD
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(cov_matrix, Eigen::DecompositionOptions::ComputeFullU);

    // Use the least singular vector as normal
    model.normal_n = (svd.matrixU().col(2));

    // Mean ground seeds value
    Eigen::Vector3f seeds_mean = points_mean.head<3>();

    // According to normal.T * [x,y,z] = -d
    model.d = -(model.normal_n.transpose() * seeds_mean)(0, 0);

    return model;
}

void Lidar::extractInitialSeeds(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in,
                                const pcl::PointCloud<pcl::PointXYZI>::Ptr seed_points)
{
    /**
     * @brief Extract a set of seed points with low height values.
     */

    // Sort on z-axis values (sortOnHeight)
    std::vector<pcl::PointXYZI> cloud_sorted((*cloud_in).points.begin(), (*cloud_in).points.end());
    sort(cloud_sorted.begin(), cloud_sorted.end(),
         [](pcl::PointXYZI p1, pcl::PointXYZI p2)
         {
             // Similar to defining a bool function
             return p1.z < p2.z;
         });

    // Negative outlier error point removal.
    // As there might be some error mirror reflection under the ground
    std::vector<pcl::PointXYZI>::iterator it = cloud_sorted.begin();
    for (size_t i = 0; i < cloud_sorted.size(); ++i)
    {
        // We define the outlier threshold -1.5 times the height of the LiDAR sensor
        if (cloud_sorted[i].z < -1.5 * sensor_height_)
        {
            it++;
        }
        else
        {
            // Points are in incremental order. Therefore, break loop if here
            break;
        }
    }
    // Remove the outlier points
    cloud_sorted.erase(cloud_sorted.begin(), it);

    // Find the Lowest Point Representive (LPR) of the sorted point cloud
    double LPR_height = 0.;
    for (int i = 0; i < num_lpr_; i++)
    {
        LPR_height += cloud_sorted[i].z;
    }
    LPR_height /= num_lpr_;

    // Iterate, filter for height less than LPR_height + th_seeds_
    (*seed_points).clear();
    for (size_t i = 0; i < cloud_sorted.size(); ++i)
    {
        if (cloud_sorted[i].z < LPR_height + th_seeds_)
        {
            (*seed_points).points.push_back(cloud_sorted[i]);
        }
    }
}

sensor_msgs::PointCloud Lidar::coneClustering(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud)
{
    /**
     * @brief Clusters the cones in the final filtered point cloud and generates a ROS message.
     *
     */

    sensor_msgs::PointCloud cluster;

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
            geometry_msgs::Point32 tmp;
            tmp.x = centroid[0];
            tmp.y = centroid[1];
            tmp.z = centroid[2];
            cluster.points.push_back(tmp);
        }
    }
    
    return cluster;
}