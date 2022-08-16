#include "ground_removal2.hpp"

namespace ns_lidar {

    GroundRemoval2::GroundRemoval2(ros::NodeHandle &n): n_(n){
        n.param<double>("th_floor", th_floor_, 0.5);
    }

    void GroundRemoval2::groundRemoval2(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in,
    pcl::PointCloud<pcl::PointXYZI>::Ptr notground_points,
    pcl::PointCloud<pcl::PointXYZI>::Ptr ground_points){

        
        int bucketsize = 100;

        pcl::PointCloud<pcl::PointXYZI> buckets[bucketsize];

        for(uint16_t i =0; i< cloud_in->points.size(); i++){
            pcl::PointXYZI point = cloud_in->points[i];
            double hypot = std::hypot(point.x, point.y) - 1;
            double angle = std::atan2(point.x, point.y) - 0.3;
            int angle_bucket = std::floor(angle / 0.25);
            int hypot_bucket = std::floor(hypot / 2.0);

            buckets[10*hypot_bucket + angle_bucket].push_back(point);
        }

        for(uint16_t i =0 ; i< bucketsize; i++){
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
