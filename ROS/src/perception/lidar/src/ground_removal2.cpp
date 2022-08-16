#include "ground_removal2.hpp"

namespace ns_lidar {

    GroundRemoval2::GroundRemoval2(ros::NodeHandle &n): n_(n){
        n.param<double>("th_floor", th_floor_, 0.5);
        n.param<int>("radial_buckets", radial_buckets_, 10);
        n.param<int>("angular_buckets", angular_buckets_, 10);
    }

    void GroundRemoval2::groundRemoval2(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in,
    pcl::PointCloud<pcl::PointXYZI>::Ptr notground_points,
    pcl::PointCloud<pcl::PointXYZI>::Ptr ground_points){

        
        
        int bucket_size = angular_buckets_*radial_buckets_;
        pcl::PointCloud<pcl::PointXYZI> buckets[bucket_size];

        for(uint16_t i =0; i< cloud_in->points.size(); i++){
            pcl::PointXYZI point = cloud_in->points[i];
            double hypot = std::hypot(point.x, point.y) - 1;
            double angle = std::atan2(point.x, point.y) - 0.3;
            int angle_bucket = std::floor(angle / (2.5/double(angular_buckets_)));
            int hypot_bucket = std::floor(hypot / (20/double(radial_buckets_)));

            buckets[angular_buckets_*hypot_bucket + angle_bucket].push_back(point);
        }

        for(uint16_t i =0 ; i< bucket_size; i++){
            pcl::PointCloud<pcl::PointXYZI> bucket = buckets[i];
            if(bucket.size() != 0){
                std::sort(bucket.begin(), bucket.end(), GroundRemoval2::zsort);

                double floor = bucket.points[0].z;
                for(uint16_t p =0; p< bucket.points.size(); p++){
                    pcl::PointXYZI point = bucket.points[p];
                    if(point.z - floor < th_floor_){
                        ground_points->push_back(point);
                    }
                    else{
                        notground_points->push_back(point);
                    }
                }
            }
        }
    }
}
