#!/usr/bin/env python3

import datetime
import math
import os

import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import plot_utils as pu
import rospkg
import rospy
import tf2_geometry_msgs
import tf2_ros as tf
from geometry_msgs.msg import Point, PointStamped
from nav_msgs.msg import Odometry
from node_fixture.fixture import SLAMStatesEnum
from std_msgs.msg import UInt16
from std_srvs.srv import Empty, EmptyResponse
from trajectory import Trajectory
from ugr_msgs.msg import (
    ObservationWithCovarianceArrayStamped,
    State,
    TrajectoryError,
    TrajectoryInfo,
    TrajectoryMapInfo,
)

matplotlib.use("Agg")


def CalculateDistanceSqr(original: Point, other: Point) -> float:
    """
    Calculate the distance from one point to the next one returning the squared not the root
    """
    x = original.x - other.x
    y = original.y - other.y
    z = original.z - other.z
    return x * x + y * y + z * z


class DataCone:
    pos = Point(0, 0, 0)

    def __init__(self, amount):
        self.maxamount = amount
        self.classType = -1
        self.min = 10000
        self.max = 0
        self.distances = []
        self.posEval = []

    def AddItem(self, distance: float, pos: Point):
        if distance < self.min:
            self.min = distance
        elif distance > self.max:
            self.max = distance

        if len(self.distances) < self.maxamount:
            self.distances.append(distance)
            self.posEval.append(pos)

    def CheckPos(self, pos: Point) -> bool:
        for p in self.posEval:
            if abs(p.x - pos.x) < 0.01 and abs(p.y - pos.y) < 0.01:
                return True
        return False


class analyze_while_running:
    align_num_frames = -1

    def __init__(self):
        # ROS initialization
        rospy.init_node("slam_trajectory_evaluation")

        self.odomData = []
        self.gpsData = []
        self.previousData = []
        self.mapped = False
        self.alignmentType = rospy.get_param("~alignmentType")
        self.sampleSize = rospy.get_param("~sampleSize")
        self.amountLaps = rospy.get_param("~lapsToEval")
        self.base_link_frame = rospy.get_param("~base_link_frame", "ugr/car_base_link")

        if self.alignmentType == "":
            self.alignmentType = "sim3"

        self.subOdo = rospy.Subscriber("/input/odometry", Odometry, self.addDataOdom)
        self.subGps = rospy.Subscriber("/input/gps", Odometry, self.addDataGps)
        self.subLoopClosure = rospy.Subscriber(
            "/input/loopClosure", UInt16, self.loopIsClosed
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
        self.track_sub = rospy.Subscriber(
            "/input/observations",
            ObservationWithCovarianceArrayStamped,
            self.PerceptionMap,
        )
        self.slamMap = rospy.Subscriber(
            "/input/obs/fslam", ObservationWithCovarianceArrayStamped, self.SLAMMap
        )
        self.gtMap = rospy.Subscriber(
            "/input/map", ObservationWithCovarianceArrayStamped, self.GTmap
        )
        self.publishResults = rospy.Publisher(
            "output/trajectorEvaluation", TrajectoryInfo, queue_size=10
        )
        self.publishResultsMap = rospy.Publisher(
            "output/trajectorEvaluationMap", TrajectoryMapInfo, queue_size=10
        )
        self.tf_buffer = tf.Buffer()
        self.tf_listener = tf.TransformListener(self.tf_buffer)
        self.path = self.dirpath + str(
            str(datetime.datetime.now().date().isoformat())
            + "--"
            + str(datetime.datetime.now().time().hour)
            + ":"
            + str(datetime.datetime.now().time().minute)
        )

        self.analizing = False
        if not os.path.exists(self.path):
            os.mkdir(self.path)

    def PerceptionMap(self, track: ObservationWithCovarianceArrayStamped):
        """
        Track update is used to collect the ground truth cones with transformation
        """
        # Transform the observations!
        # This only transforms from sensor frame to base link frame, which should be a static transformation in normal conditions

        if self.mapped:
            return

        transform = self.tf_buffer.lookup_transform(
            track.header.frame_id,
            self.base_link_frame,
            rospy.Time(0),
        )

        for con in track.observations:
            minDist = 10000
            minidx = 0

            pose_s = PointStamped(point=con.observation.location)
            pose_t = tf2_geometry_msgs.do_transform_point(pose_s, transform)
            p = pose_t.point
            for idx, absPos in enumerate(self.cones):
                dist = CalculateDistanceSqr(p, absPos.pos)
                if minDist > dist:
                    minDist = dist
                    minidx = idx
            self.cones[minidx].AddItem(math.sqrt(minDist), p)

    def SLAMMap(self, track: ObservationWithCovarianceArrayStamped):
        """
        read the data from fastslam and pushes it to the SLAMcones
        """
        if self.mapped:
            return
        for con in track.observations:
            minDist = 100000
            minidx = 0

            for idx, absPos in enumerate(self.SLAMcones):
                dist = CalculateDistanceSqr(con.observation.location, absPos.pos)
                if minDist > dist:
                    minDist = dist
                    minidx = idx
            if not self.SLAMcones[minidx].CheckPos(con.observation.location):
                self.SLAMcones[minidx].AddItem(
                    math.sqrt(minDist), con.observation.location
                )

    def GTmap(self, track: ObservationWithCovarianceArrayStamped):
        """
        reads the information of the GTmap of the cones and add alle the data of the cones
        """
        if self.mapped:
            return
        self.cones = []
        self.SLAMcones = []

        for cone in track.observations:
            DaCo = DataCone(self.sampleSize)
            DaCo.pos = cone.observation.location
            DaCo.classType = cone.observation.observation_class
            self.cones.append(DaCo)

            DaCo = DataCone(self.sampleSize)
            DaCo.pos = cone.observation.location
            DaCo.classType = cone.observation.observation_class
            self.SLAMcones.append(DaCo)

    def run_trajectoryEvaluation(self, msg: Empty):
        """
        analyse the trajectoryEvaluation
        """
        if len(self.odomData) > 1 and len(self.gpsData) > 1:
            self.PrintMapToTopic(self.cones, self.SLAMcones)
            self.AnalyzeAndPrintMapToFile(self.cones, self.SLAMcones)
            self.analyze_trail()
        return EmptyResponse()

    def resetTrajectoryEvaluation(self, msg: Empty):
        """
        reset the trajectoryEvaluation
        """
        self.odomData = []
        self.gpsData = []
        self.previousData = []
        self.analizing = False
        return EmptyResponse()

    def addDataOdom(self, msg: Odometry):
        """
        setting the previous data from odom to add until the gps data has been recieved
        """
        if self.analizing:
            return
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
        if self.analizing:
            return
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

    def loopIsClosed(self, totalLaps):
        """
        when finishing up a lap we start getting information
        this is temporary until the statmachine works
        """
        if self.cur_state == SLAMStatesEnum.RACING:
            if totalLaps.data == self.amountLaps and self.amountLaps != 0:
                self.analyze_trail()

    def stateChanged(self, state: State):
        """
        analyzes the trail if the round is finished
        """
        self.cur_state = state.cur_state
        if (
            state.prev_state == SLAMStatesEnum.EXPLORATION
            and state.cur_state != SLAMStatesEnum.EXPLORATION
        ):
            self.resetTrajectoryEvaluation(Empty())
            self.PrintMapToTopic(self.cones, self.SLAMcones)
            self.AnalyzeAndPrintMapToFile(self.cones, self.SLAMcones)
        elif state.cur_state == SLAMStatesEnum.FINISHED and not self.analizing:
            self.analyze_trail()

    def analyze_trail(self):
        """
        analyse the trail and show the errors in pdf form
        """
        self.analizing = True
        rospy.loginfo("startanalyzing")
        data_gt = np.array(self.gpsData.copy(), dtype=float)
        data_est = np.array(self.odomData.copy(), dtype=float)
        stamps_gt = [float(v[0]) for v in self.gpsData.copy()]
        stamps_est = [float(v[0]) for v in self.odomData.copy()]

        traj = Trajectory(load_data=False, align_type=self.alignmentType)
        traj.align_trajectory_with_data(data_gt, data_est, stamps_gt, stamps_est)
        # compute the absolute error
        rospy.loginfo("aligned")
        traj.compute_absolute_error()

        rospy.loginfo("absolut error calculate")
        traj.compute_relative_errors()
        rospy.loginfo("relative error calculate")

        rel_errors, distances = traj.get_relative_errors_and_distance_no_ignore()
        rospy.loginfo("printTopic")
        self.PrintToTopic(rel_errors, distances, traj)
        rospy.loginfo("printed")
        self.PrintTrajectoryError(rel_errors, distances, traj)

    def PrintMapToTopic(self, cones, slamCones):
        self.mapped = True
        info = TrajectoryMapInfo()
        for i in cones:
            for y in i.distances:
                info.avgDistanceToConePerception.append(y)
        for i in slamCones:
            for y in i.distances:
                info.avgDistanceToConeSLAM.append(y)
        self.publishResultsMap.publish(info)

    def PrintToTopic(self, rel_errors, distances, traj):
        """
        print to the topic for trajectoryinfo
        """
        info = TrajectoryInfo()
        for i in distances:
            info.trajectoryErrorDistance.append(i)
        for i in rel_errors["rel_trans"][0]:
            err = TrajectoryError()
            for k in i:
                err.errorTraj.append(k)
            info.relTranslationError.append(err)
        for i in rel_errors["rel_yaw"][0]:
            err = TrajectoryError()
            for k in i:
                err.errorTraj.append(k)
            info.relYawError.append(err)
        for i in traj.accum_distances:
            info.absTranslationError.append(i)
        for i in traj.abs_errors["abs_e_trans_vec"][0]:
            info.absTranslationError.append(i * 1000)
        self.publishResults.publish(info)

    def printCones(self, c) -> plt.figure:
        """
        returns a figure that prints the data from the cones
        """
        fig = plt.figure(figsize=(10, 10))
        ax = fig.add_subplot()
        pos = [[], [], [], []]
        colorTruth = ["#0000FF", "#FFFF00", "#FFA500", "#ff0000"]
        colorGuess = ["#00FFFF", "#FFe600", "#fcc662", "#ff6200"]
        posView = [[[], []], [[], []], [[], []], [[], []]]

        id = 0
        for con in c:
            for poss in con.posEval:
                if con.classType <= 3:
                    posView[con.classType][0].append(poss.x)
                    posView[con.classType][1].append(poss.y)
            if con.classType <= 3:
                pos[con.classType].append([con.pos.x, con.pos.y, id])
            id += 1

        size = 20
        for coloridx, color in enumerate(colorTruth):
            for p in pos[coloridx]:
                if len(p) >= 3:
                    ax.scatter(p[0], p[1], s=size, color=color)
                    ax.annotate(p[2], (p[0], p[1]))
        for idx, color in enumerate(colorGuess):
            if len(posView[idx]) >= 2:
                ax.scatter(
                    posView[idx][0],
                    posView[idx][1],
                    s=size / 2,
                    marker="x",
                    color=color,
                )
        plt.close(fig)
        return fig

    def printDistances(self, c) -> plt.figure:
        """
        prints the distance from the individual cones to a figure
        """
        fig = plt.figure(figsize=(30, 10))
        ax = fig.add_subplot()
        id = 0
        for i in c:
            for y in i.distances:
                ax.scatter(id, y)
            id += 1

        plt.close(fig)
        return fig

    def AnalyzeAndPrintMapToFile(self, cones, slamCones):
        cones = self.cones.copy()
        slamCones = self.SLAMcones.copy()
        FORMAT = ".pdf"
        figCS = self.printCones(slamCones)
        figC = self.printCones(cones)
        figmapD = self.printDistances(slamCones)
        figpercD = self.printDistances(cones)
        rospy.loginfo("exported and ready to save")
        figC.savefig(self.path + "/perceptionPos" + FORMAT, bbox_inches="tight")
        figCS.savefig(self.path + "/mapPos" + FORMAT, bbox_inches="tight")
        figmapD.savefig(self.path + "/mapDist" + FORMAT, bbox_inches="tight")
        figpercD.savefig(self.path + "/perceptionDist" + FORMAT, bbox_inches="tight")

    def PrintTrajectoryError(self, rel_errors, distances, traj):
        """
        print all the data to the folder
        """
        labels = ["Estimate"]
        FORMAT = ".pdf"
        colors = ["b"]
        # trajectory path
        fig = plt.figure(figsize=(6, 5.5))
        ax = fig.add_subplot(111, aspect="equal", xlabel="x [m]", ylabel="z [m]")

        pu.plot_trajectory_top(ax, traj.p_es_aligned, "b", "Estimate")
        pu.plot_trajectory_top(ax, traj.p_gt, "m", "Groundtruth")
        pu.plot_aligned_top(ax, traj.p_es_aligned, traj.p_gt, traj.align_num_frames)

        plt.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.0)
        fig.tight_layout()
        fig.savefig(self.path + "/trajectory_top" + FORMAT, bbox_inches="tight")
        plt.close(fig)

        # trajctory_side
        fig = plt.figure(figsize=(6, 5.5))
        ax = fig.add_subplot(111, aspect="equal", xlabel="x [m]", ylabel="z [m]")

        pu.plot_trajectory_side(ax, traj.p_es_aligned, "b", "Estimate")
        pu.plot_trajectory_side(ax, traj.p_gt, "m", "Groundtruth")
        plt.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.0)

        fig.tight_layout()
        fig.savefig(self.path + "/trajectory_side" + FORMAT, bbox_inches="tight")
        plt.close(fig)

        # translation error
        fig = plt.figure(figsize=(8, 2.5))
        ax = fig.add_subplot(
            111,
            xlabel="Distance [m]",
            ylabel="Position Drift [mm]",
            xlim=[0, traj.accum_distances[-1]],
        )

        pu.plot_error_n_dim(
            ax,
            traj.accum_distances,
            traj.abs_errors["abs_e_trans_vec"] * 1000,
            self.path,
        )
        ax.legend()
        fig.tight_layout()
        fig.savefig(self.path + "/translation_error" + FORMAT, bbox_inches="tight")
        plt.close(fig)

        # rel errors position
        fig = plt.figure(figsize=(6, 2.5))
        ax = fig.add_subplot(
            111, xlabel="Distance traveled [m]", ylabel="Translation error [m]"
        )
        pu.boxplot_compare(ax, distances, rel_errors["rel_trans"], labels, colors)
        ax.legend()
        fig.tight_layout()
        fig.savefig(self.path + "/rel_translation_error" + FORMAT, bbox_inches="tight")
        plt.close(fig)

        # rel errors position in percentage
        fig = plt.figure(figsize=(6, 2.5))
        ax = fig.add_subplot(
            111, xlabel="Distance traveled [m]", ylabel="Translation error [%]"
        )
        pu.boxplot_compare(ax, distances, rel_errors["rel_trans_perc"], labels, colors)
        fig.tight_layout()
        fig.savefig(
            self.path + "/rel_translation_error_perc" + FORMAT, bbox_inches="tight"
        )
        plt.close(fig)

        # rel errors yaw
        fig = plt.figure(figsize=(6, 2.5))
        ax = fig.add_subplot(
            111, xlabel="Distance traveled [m]", ylabel="Yaw error [deg]"
        )
        pu.boxplot_compare(ax, distances, rel_errors["rel_yaw"], labels, colors)
        fig.tight_layout()
        fig.savefig(self.path + "/rel_yaw_error" + FORMAT, bbox_inches="tight")
        plt.close(fig)

        rospy.loginfo(
            "finished calculating errors it is saved under the folder:" + self.zpath
        )
        return


node = analyze_while_running()
rospy.spin()
