#!/usr/bin/env python3

import matplotlib
import rospkg
import rospy

matplotlib.use("Agg")
import datetime
import os

import matplotlib.pyplot as plt
import numpy as np
import plot_utils as pu
from nav_msgs.msg import Odometry
from node_fixture.node_fixture import SLAMStatesEnum, StateMachineScopeEnum
from std_srvs.srv import Empty, EmptyResponse
from trajectory import Trajectory
from ugr_msgs.msg import (
    ObservationWithCovariance,
    ObservationWithCovarianceArrayStamped,
    Observation,
    State,
    UInt16,
)

class analyze_while_running:
    align_num_frames = -1

    def __init__(self):
        # ROS initialization
        rospy.init_node("slam_trajectory_evaluation")

        self.odomData = []
        self.gpsData = []
        self.previousData = []

        self.alignmentType = rospy.get_param("~alignmentType")
        if self.alignmentType == "":
            self.alignmentType = "sim3"

        self.subOdo = rospy.Subscriber("/input/odometry", Odometry, self.addDataOdom)
        self.subGps = rospy.Subscriber("/input/gps", Odometry, self.addDataGps)
        self.subLoopClosure = rospy.Subscriber(
            "/output/loopClosure", UInt16, self.loopIsClosed
        )
        self.stateChange = rospy.Subscriber("/state", State, self.stateChanged)
        self.resetTrajectory = rospy.Service(
            "/reset", Empty, self.resetTrajectoryEvaluation
        )
        self.runEvaluation = rospy.Service(
            "/run_evaluation", Empty, self.run_trajectoryEvaluation
        )
        # create dir to store the info of the run
        # we remove the last 4 characters
        rospack = rospkg.RosPack()
        self.dirpath = (
            rospack.get_path("slam_trajectory_evaluation") + "/data-error-correction/"
        )

        if not os.path.exists(self.dirpath):
            os.makedirs(self.dirpath)
        self.track_sub = rospy.Subscriber("/output/observations",ObservationWithCovarianceArrayStamped,self.track_update)
        
    ### reads the data from the cones and the position of them.
    def track_update(self, track: ObservationWithCovarianceArrayStamped):
        """
        Track update is used to collect the ground truth cones
        """
        cones = []
        for cone in track.observations:
            cones.append(
                np.array(
                    [cone.observation.location.x, cone.observation.location.y, cone.observation.location.z, cone.observation.observation_class],
                    dtype=np.float32,
                )
            )
        
        cones = np.array(cones)
        rospy.loginfo(len(cones))
    def run_trajectoryEvaluation(self, msg: Empty):
        """
        analyse the trajectoryEvaluation
        """
        if len(self.odomData) > 1 and len(self.gpsData) > 1:
            self.analyze_trail()
        return EmptyResponse()

    def resetTrajectoryEvaluation(self, msg: Empty):
        """
        reset the trajectoryEvaluation
        """
        self.odomData = []
        self.gpsData = []
        self.previousData = []
        return EmptyResponse()

    def addDataOdom(self, msg: Odometry):
        """
        setting the previous data from odom to add until the gps data has been recieved
        """
        self.previousData = [
            rospy.get_time(),
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        ]

    def addDataGps(self, msg: Odometry):
        """
        adding the gpsdata as path the same from odomdata as a path
        """
        # to make sure that the position is nog 0.0
        if len(self.previousData) > 0 and not (
            msg.pose.pose.position.x == 0.0
            and msg.pose.pose.position.y == 0.0
            and msg.pose.pose.position.z == 0.0
        ):
            self.gpsData.append(
                [
                    rospy.get_time(),
                    msg.pose.pose.position.x,
                    msg.pose.pose.position.y,
                    msg.pose.pose.position.z,
                    msg.pose.pose.orientation.x,
                    msg.pose.pose.orientation.y,
                    msg.pose.pose.orientation.z,
                    msg.pose.pose.orientation.w,
                ]
            )
            self.odomData.append(self.previousData)

    def loopIsClosed(self, totalLaps: UInt16):
        """
        when finishing up a lap we start getting information
        this is temporary until the statmachine works
        """
        self.analyze_trail()

    def stateChanged(self, state: State):
        """
        analyzes the trail if the round is finished
        """
        if (
            state.scope == StateMachineScopeEnum.SLAM
            and state.cur_state == SLAMStatesEnum.FINISHED
        ):
            self.analyze_trail()

    def analyze_trail(self):
        """
        analyse the trail and show the errors in pdf form
        """

        data_gt = np.array(self.gpsData, dtype=float)
        data_est = np.array(self.odomData, dtype=float)
        stamps_gt = [float(v[0]) for v in self.gpsData]
        stamps_est = [float(v[0]) for v in self.odomData]

        self.traj = Trajectory(load_data=False, align_type=self.alignmentType)
        self.traj.align_trajectory_with_data(data_gt, data_est, stamps_gt, stamps_est)
        # compute the absolute error
        self.traj.compute_absolute_error()

        self.traj.compute_relative_errors()

        rel_errors, distances = self.traj.get_relative_errors_and_distance_no_ignore()
        self.print_error_to_file(rel_errors, distances)

    def print_error_to_file(self, rel_errors, distances):
        labels = ["Estimate"]
        FORMAT = ".pdf"
        colors = ["b"]

        self.dirpath += str(
            datetime.datetime.now().date().isoformat()
            + "--"
            + datetime.datetime.now().time().isoformat()
        )
        if not os.path.exists(self.dirpath):
            os.mkdir(self.dirpath)

        # trajectory path
        fig = plt.figure(figsize=(6, 5.5))
        ax = fig.add_subplot(111, aspect="equal", xlabel="x [m]", ylabel="z [m]")

        pu.plot_trajectory_top(ax, self.traj.p_es_aligned, "b", "Estimate")
        pu.plot_trajectory_top(ax, self.traj.p_gt, "m", "Groundtruth")
        pu.plot_aligned_top(
            ax, self.traj.p_es_aligned, self.traj.p_gt, self.traj.align_num_frames
        )

        plt.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.0)
        fig.tight_layout()
        fig.savefig(self.dirpath + "/trajectory_top" + FORMAT, bbox_inches="tight")
        plt.close(fig)

        # trajctory_side
        fig = plt.figure(figsize=(6, 5.5))
        ax = fig.add_subplot(111, aspect="equal", xlabel="x [m]", ylabel="z [m]")

        pu.plot_trajectory_side(ax, self.traj.p_es_aligned, "b", "Estimate")
        pu.plot_trajectory_side(ax, self.traj.p_gt, "m", "Groundtruth")
        plt.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.0)

        fig.tight_layout()
        fig.savefig(self.dirpath + "/trajectory_side" + FORMAT, bbox_inches="tight")
        plt.close(fig)

        # translation error
        fig = plt.figure(figsize=(8, 2.5))
        ax = fig.add_subplot(
            111,
            xlabel="Distance [m]",
            ylabel="Position Drift [mm]",
            xlim=[0, self.traj.accum_distances[-1]],
        )

        pu.plot_error_n_dim(
            ax,
            self.traj.accum_distances,
            self.traj.abs_errors["abs_e_trans_vec"] * 1000,
            self.dirpath,
        )
        ax.legend()
        fig.tight_layout()
        fig.savefig(self.dirpath + "/translation_error" + FORMAT, bbox_inches="tight")
        plt.close(fig)

        # rel errors position
        fig = plt.figure(figsize=(6, 2.5))
        ax = fig.add_subplot(
            111, xlabel="Distance traveled [m]", ylabel="Translation error [m]"
        )
        pu.boxplot_compare(ax, distances, rel_errors["rel_trans"], labels, colors)
        ax.legend()
        fig.tight_layout()
        fig.savefig(
            self.dirpath + "/rel_translation_error" + FORMAT, bbox_inches="tight"
        )
        plt.close(fig)

        # rel errors position in percentage
        fig = plt.figure(figsize=(6, 2.5))
        ax = fig.add_subplot(
            111, xlabel="Distance traveled [m]", ylabel="Translation error [\%]"
        )
        pu.boxplot_compare(ax, distances, rel_errors["rel_trans_perc"], labels, colors)
        fig.tight_layout()
        fig.savefig(
            self.dirpath + "/rel_translation_error_perc" + FORMAT, bbox_inches="tight"
        )
        plt.close(fig)

        # rel errors yaw
        fig = plt.figure(figsize=(6, 2.5))
        ax = fig.add_subplot(
            111, xlabel="Distance traveled [m]", ylabel="Yaw error [deg]"
        )
        pu.boxplot_compare(ax, distances, rel_errors["rel_yaw"], labels, colors)
        fig.tight_layout()
        fig.savefig(self.dirpath + "/rel_yaw_error" + FORMAT, bbox_inches="tight")
        plt.close(fig)
        rospy.loginfo(
            "finished calculating errors it is saved under the folder:" + self.dirpath
        )


node = analyze_while_running()
rospy.spin()
