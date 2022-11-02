#ifndef MCL_HPP
#define MCL_HPP

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <ugr_msgs/ObservationWithCovarianceArrayStamped.h>

#include "particle.hpp"

#include <Eigen/Dense>

#include "tf2_ros/message_filter.h"
#include "message_filters/subscriber.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <nav_msgs/Odometry.h>

using namespace std;

#define LANDMARK_CLASS_COUNT 4

namespace slam
{

  struct LandmarkSearchResult
  {
    int index;
    double distance;
  };

  struct LandmarkSearchResult
  {
    int index;
    double distance;
  };

  struct Landmark
  {
    int landmarkClass;
    MatrixXf variance;
    VectorXf pose;
  };

  class MCL
  {
  public:
    MCL(ros::NodeHandle &n);
    ~MCL()
    {
      this->particles.clear();
    }

    static tf2_ros::TransformBroadcaster br;

  private:
    // ROS
    ros::NodeHandle n;

    // ROS parameters
    string base_link_frame;
    string world_frame;
    string slam_world_frame;

    int particle_count;
    int effective_particle_count;

    double observe_dt;
    double eps;

    // Internal parameters
    double latestTime;

    double max_range;
    double max_half_fov;

    vector<Landmark> _map;
    kdt::KDTree<KDTreePoint> _trees[LANDMARK_CLASS_COUNT];

    array<double, 3> prev_state; // x, y, yaw

    chrono::steady_clock::time_point prev_time;

    vector<Particle> particles;

    // Yaw unwrapping threshold
    float yaw_unwrap_threshold;

    Eigen::MatrixXf Q;
    Eigen::MatrixXf R;

    // Subscribers
    ros::Subscriber observationsSubscriber;
    ros::Subscriber mapSubscriber;
    ros::Subscriber initialPositionSubscriber;

    // Publishers
    ros::Publisher odomPublisher;

    // TF2
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;
    message_filters::Subscriber<ugr_msgs::ObservationWithCovarianceArrayStamped> obs_sub;
    tf2_ros::MessageFilter<ugr_msgs::ObservationWithCovarianceArrayStamped> tf2_filter;

    // Handlers
    void handleObservations(const ugr_msgs::ObservationWithCovarianceArrayStampedConstPtr &obs);
    void handleInitialPosition(const nav_msgs::Odometry &obs);
    void handleMap(const ugr_msgs::ObservationWithCovarianceArrayStampedConstPtr &obs);

    void publishOutput();

    void predict(Particle &particle, double dDist, double dYaw);
    double sensor_update(Particle &particle, vector<VectorXf> &z, vector<int> &associations);

    void build_associations(Particle &, ugr_msgs::ObservationWithCovarianceArrayStamped &, vector<VectorXf> &, vector<VectorXf> &, vector<int> &, vector<int> &, vector<int> &);

    void landmarks_to_observations(vector<VectorXf> &lm, vector<VectorXf> &obs, VectorXf &pose);
    void landmark_to_observation(VectorXf &lm, VectorXf &obs, VectorXf &pose);

    void observations_to_landmarks(vector<VectorXf> &obs, vector<VectorXf> &lm, VectorXf &pose);
    void observation_to_landmark(VectorXf &obs, VectorXf &lm, VectorXf &pose);
  };
}

#endif