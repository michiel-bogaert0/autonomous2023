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
import math
from nav_msgs.msg import Odometry
from node_fixture.node_fixture import SLAMStatesEnum, StateMachineScopeEnum
from std_srvs.srv import Empty, EmptyResponse
from std_msgs.msg import UInt16
from geometry_msgs.msg import Point
from trajectory import Trajectory
from ugr_msgs.msg import (
    ObservationWithCovariance,
    ObservationWithCovarianceArrayStamped,
    Observation,
    State,
)
def CalculateDistanceSqr(original: Point, other:Point ) ->float:
        """
        Calculate the distance from one point to the next one returning the squared not the root
        """
        x = original.x - other.x
        y = original.y - other.y
        z = original.z - other.z
        return x*x + y*y +z*z

class DataCone:
    pos = Point(0,0,0)
    
    def __init__(self, amount):
        self.maxamount = amount
        self.classType = -1
        self.min = 10000
        self.max = 0
        self.distances=[]
        self.posEval=[]
    def AddItem(self, distance: float, pos: Point):
        if distance < self.min: self.min = distance
        elif distance > self.max: self.max = distance
        
        if len(self.distances) < self.maxamount : 
            self.distances.append(distance) 
            self.posEval.append(pos)
    def CheckPos(self,pos: Point) -> bool:
        for p in self.posEval:
            if(abs(p.x - pos.x) < 0.01 and abs(p.y - pos.y) < 0.01):
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

        self.alignmentType = rospy.get_param("~alignmentType")
        self.sampleSize = rospy.get_param("~sampleSize")
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
        self.slamMap = rospy.Subscriber("/output/obs/fslam",ObservationWithCovarianceArrayStamped,self.SLAMMap)
        self.gtMap = rospy.Subscriber("/output/map",ObservationWithCovarianceArrayStamped,self.GTmap)
        
    ### reads the data from the cones and the position of them.
    def track_update(self, track: ObservationWithCovarianceArrayStamped):
        """
        Track update is used to collect the ground truth cones
        """

        for con in track.observations:
            minDist = 10000
            minidx = 0
            for idx, absPos in enumerate(self.cones):
                dist = CalculateDistanceSqr(con.observation.location, absPos.pos)
                if minDist > dist:
                    minDist = dist
                    minidx = idx
            #min distance is found, so the cone is found
            self.cones[minidx].AddItem(math.sqrt(minDist), con.observation.location)

        #for cone in self.cones:
            #rospy.loginfo(cone.distances)

    def SLAMMap(self, track: ObservationWithCovarianceArrayStamped):
        for con in track.observations:
            minDist2 = 100000
            minidx = 0
            for idx, absPos in enumerate(self.cones):
                dist = CalculateDistanceSqr(con.observation.location, absPos.pos)
                if minDist2 > dist:
                    minDist2 = dist
                    minidx = idx
            if not self.SLAMcones[minidx].CheckPos(con.observation.location):
                self.SLAMcones[minidx].AddItem(math.sqrt(minDist2), con.observation.location)
            
    def GTmap(self, track: ObservationWithCovarianceArrayStamped):
        """ 
        reads the information of the GTmap of the cones and add alle the data of the cones
        """
        self.cones = []
        self.SLAMcones = []

        #rospy.loginfo(len(self.cones))
        
        for cone in track.observations:
            DaCo = DataCone(self.sampleSize)
            DaCo.pos = cone.observation.location
            DaCo.classType = cone.observation.observation_class
            self.cones.append(DaCo)
            self.SLAMcones.append(DaCo)

            #rospy.loginfo(DaCo.pos)
   

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

    def loopIsClosed(self, totalLaps):
        """
        when finishing up a lap we start getting information
        this is temporary until the statmachine works
        """
        if(totalLaps.data == 1):
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
        rospy.loginfo("startanalyzing")
        data_gt = np.array(self.gpsData, dtype=float)
        data_est = np.array(self.odomData, dtype=float)
        stamps_gt = [float(v[0]) for v in self.gpsData]
        stamps_est = [float(v[0]) for v in self.odomData]

        #self.traj = Trajectory(load_data=False, align_type=self.alignmentType)
        #self.traj.align_trajectory_with_data(data_gt, data_est, stamps_gt, stamps_est)
        # compute the absolute error
        rospy.loginfo("aligned")
        #self.traj.compute_absolute_error()

        rospy.loginfo("absolut error calculate")
        #self.traj.compute_relative_errors()
        rospy.loginfo("relative error calculate")

        #rel_errors, distances = self.traj.get_relative_errors_and_distance_no_ignore()
        #self.print_error_to_file(rel_errors, distances)
        self.print_error_to_file([],[])

    def printCones(self, cones) -> plt.figure:
        fig = plt.figure(figsize=(10,10))
        ax = fig.add_subplot()
        pos =[[],[],[],[]]
        colorTruth =['#0000FF', "#FFFF00","#FFA500", "#ff0000"]
        colorGuess =['#00FFFF', "#FFe600","#fcc662", "#ff6200"]
        posView = [[],[],[],[]]

        id = 0
        for con in cones:
            for poss in con.posEval:
                if con.classType <= 3:
                    posView[con.classType].append([poss.x,poss.y])
            if con.classType <= 3:
                pos[con.classType].append([con.pos.x, con.pos.y, id])
            id+=1

        size = 20
        for idx, color in enumerate(colorTruth):
            if len(pos[idx]) >= 3:
                ax.scatter(pos[idx][0],pos[idx][1], s=size, color=color)
                ax.annotate(pos[idx][2], (pos[idx][0], pos[idx][1]))
        for idx, color in enumerate(colorGuess):
            if len(posView[idx]) >= 2:
                ax.scatter(posView[idx][0], posView[idx][1], s=size/2,marker='x',color=color)
        plt.close(fig)
        return fig
    
    def printDistances(self, cones) -> plt.figure:
        fig = plt.figure(figsize=(30,10))
        ax = fig.add_subplot()
        id = 0
        for i in cones:
            for y in i.distances:
                ax.scatter(id,y)
            id += 1
        
        plt.close(fig)
        return fig
    
    def print_error_to_file(self, rel_errors, distances):
        labels = ["Estimate"]
        FORMAT = ".pdf"
        colors = ["b"]
        rospy.loginfo("print")

        self.dirpath += str(
            datetime.datetime.now().date().isoformat()
            + "--"
            + datetime.datetime.now().time().isoformat()
        )
        if not os.path.exists(self.dirpath):
            os.mkdir(self.dirpath)

        #print the map
        fig = plt.figure(figsize=(10,10))
        ax = fig.add_subplot()
        #defaultPos
        posx0 = []
        posy0 = []
        posx1 = []
        posy1 = []
        posx2 = []
        posy2 = []
        posx3 = []
        posy3 = []
        id0 = []
        id1 = []
        id2 = []
        id3 = []
        posxView0 = []
        posyView0 = []
        posxView1 = []
        posyView1 = []
        posxView2 = []
        posyView2 = []
        posxView3 = []
        posyView3 = []
        id = 0
        for con in self.SLAMcones:
            for x in con.posEval:
                if con.classType == 0:
                    posxView0.append(x.x)
                    posyView0.append(x.y)
                elif con.classType == 1:
                    posxView1.append(x.x)
                    posyView1.append(x.y)
                elif con.classType == 2:
                    posxView2.append(x.x)
                    posyView2.append(x.y)
                elif con.classType == 3:
                    posxView3.append(x.x)
                    posyView3.append(x.y)
            if con.classType == 0:
                posx0.append(con.pos.x)
                posy0.append(con.pos.y)
                id0.append(id)
            elif con.classType == 1:
                posx1.append(con.pos.x)
                posy1.append(con.pos.y)
                id1.append(id)
            elif con.classType == 2:
                posx2.append(con.pos.x)
                posy2.append(con.pos.y)
                id2.append(id)
            elif con.classType == 3:
                posx3.append(con.pos.x)
                posy3.append(con.pos.y)
                id3.append(id)
            id+=1

        size = 20
        ax.scatter(posx0, posy0, s=size, color='#0000FF')
        ax.scatter(posx1, posy1, s=size, color="#FFFF00")
        ax.scatter(posx2, posy2, s=size, color="#FFA500")
        ax.scatter(posx3, posy3, s=size, color="r")
        ax.scatter(posxView0, posyView0, s=size/2,marker='x', color='#00FFFF')
        ax.scatter(posxView1, posyView1, s=size/2,marker='x', color="#FFe600")
        ax.scatter(posxView2, posyView2, s=size/2,marker='x', color="#FFA500")
        ax.scatter(posxView3, posyView3, s=size/2,marker='x', color="r")

        id = 0
        for i in id0:
            ax.annotate(i, (posx0[id], posy0[id]))
            id+=1
        id = 0
        for i in id1:
            ax.annotate(i, (posx1[id], posy1[id]))
            id+=1
        id = 0
        for i in id2:
            ax.annotate(i, (posx2[id], posy2[id]))
            id+=1
        id = 0
        for i in id3:
            ax.annotate(i, (posx3[id], posy3[id]))
            id+=1

        fig.savefig(self.dirpath + "/mapPos" + FORMAT, bbox_inches="tight") 
        plt.close(fig)
        fig.tight_layout()

        fig = plt.figure(figsize=(30,10))
        ax = fig.add_subplot()
        id = 0
        for i in self.cones:
            for y in i.distances:
                ax.scatter(id,y)
            id += 1
        
        fig.tight_layout()
        #fig.savefig(self.dirpath + "/SLAMdist" + FORMAT, bbox_inches="tight") 
        #plt.close(fig)
        self.printCones(self.cones).savefig(self.dirpath + "/mapPos" + FORMAT, bbox_inches="tight") 
        self.printDistances(self.cones).savefig(self.dirpath + "/mapDist" + FORMAT, bbox_inches="tight")
        return
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
