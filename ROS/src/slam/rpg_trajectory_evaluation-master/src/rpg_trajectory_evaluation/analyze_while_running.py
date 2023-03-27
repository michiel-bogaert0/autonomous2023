#!/usr/bin/env python3

import string
from node_fixture.node_fixture import AddSubscriber, ROSNode
import rospy
import rospkg
from nav_msgs.msg import Odometry
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import plot_utils as pu
import numpy as np
# importing os module
import os
from trajectory import Trajectory
from std_msgs.msg import UInt16

import datetime

class analyze_while_running(ROSNode):
    rel_error_cached_nm = 'cached_rel_err'
    rel_error_prefix = 'relative_error_statistics_'
    saved_res_dir_nm = 'saved_results'
    cache_res_dir_nm = 'cached'
    default_boxplot_perc = [0.1, 0.2, 0.3, 0.4, 0.5]
    platform=''
    alg=''
    dataset_short_name=''
    align_type='sim3'
    align_num_frames=-1
    suffix_str=''
    est_type='traj_est'
    nm_gt='stamped_groundtruth.txt'
    nm_est='stamped_traj_estimate.txt'
    nm_matches='stamped_est_gt_matches.txt'
    preset_boxplot_distances=[]
    preset_boxplot_percentages=[],
    max_diff=0.02
    
    def __init__(self):
        super().__init__("analyser")
        self.odomData = []
        self.gpsData = []
        self.previousData = []
        
        #create dir to store the info of the run
        # this is saved under .ros in root folder
        self.dirpath = os.getcwd() + '../autonomous2023/data-error-correction/' 
        if not os.path.exists(self.dirpath):
            os.makedirs(self.dirpath)
        self.dirpath += str(datetime.datetime.now().minute)
        if not os.path.exists(self.dirpath):
            os.mkdir(self.dirpath)

    @AddSubscriber("/input/odometry")
    def addDataOdom(self, msg: Odometry):
        """
        setting the previous data from odom to add until the gps data has been recieved
        """
        self.previousData = [rospy.get_time(),
            msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z,
            msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z, msg.pose.pose.orientation.w
            ]
    
    @AddSubscriber("/input/gps")
    def addDataGps(self, msg: Odometry):
        """
            adding the gpsdata as path the same from odomdata as a path
            
        """
        if len(self.previousData) > 0 and not (msg.pose.pose.position.x == 0.0 and msg.pose.pose.position.y == 0.0 and msg.pose.pose.position.z == 0.0):   
            self.gpsData.append([rospy.get_time(),
                msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z,
                msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z, msg.pose.pose.orientation.w
                ])
            self.odomData.append(self.previousData)
    

    @AddSubscriber("/output/loopClosure")
    def loopIsClosed(self, totalLaps:UInt16):
        """
        when finishing up a lap we start getting information 
        this is temp, need to look to the end of the rosbag file
        """
        #we do nothing with the totalLaps

        self.analyze_trail()


    def analyze_trail(self):
        """
        analyse the trail and show the errors in pdf form 
        """        
        #starting the calculation of the errors.
        #dividing the data in timestamp
        
        data_gt = np.array(self.gpsData, dtype=float)
        data_est = np.array(self.odomData, dtype=float)
        stamps_gt = [float(v[0]) for v in self.gpsData]
        stamps_est = [float(v[0]) for v in self.odomData]
        
        self.traj = Trajectory(load_data=False)
        self.traj.align_trajectory_with_data(data_gt, data_est, stamps_gt, stamps_est)
        # compute the absolute error
        self.traj.compute_absolute_error()
        
        self.traj.compute_relative_errors()
        
        # rel_error, distance = traj.get_relative_errors_and_distances(error_types='rel_trans')
        
        rel_errors, distances = self.traj.get_relative_errors_and_distance_no_ignore()
        self.print_error_to_file(rel_errors, distances)

        
    def print_error_to_file(self,  rel_errors, distances):
        labels = ['Estimate']
        FORMAT = '.pdf'
        colors = ['b']

        if not os.path.exists(self.dirpath):
            return 
        
        #trajectory path
        fig = plt.figure(figsize=(6, 5.5))
        ax = fig.add_subplot(111, aspect='equal',xlabel='x [m]', ylabel='z [m]')

        pu.plot_trajectory_top(ax, self.traj.p_es_aligned, 'b', 'Estimate')
        pu.plot_trajectory_top(ax, self.traj.p_gt, 'm', 'Groundtruth')
        pu.plot_aligned_top(ax, self.traj.p_es_aligned, self.traj.p_gt,self.traj.align_num_frames)

        plt.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.)
        fig.tight_layout()
        fig.savefig(self.dirpath+'/trajectory_top' + FORMAT, bbox_inches="tight")
        plt.close(fig)

        #trajctory_side
        fig = plt.figure(figsize=(6, 5.5))
        ax = fig.add_subplot(111, aspect='equal',xlabel='x [m]', ylabel='z [m]')

        pu.plot_trajectory_side(ax, self.traj.p_es_aligned, 'b', 'Estimate')
        pu.plot_trajectory_side(ax, self.traj.p_gt, 'm', 'Groundtruth')
        plt.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.)

        fig.tight_layout()
        fig.savefig(self.dirpath+'/trajectory_side' + FORMAT, bbox_inches="tight")
        plt.close(fig)

        #translation error
        fig = plt.figure(figsize=(8, 2.5))
        ax = fig.add_subplot(111, xlabel='Distance [m]', ylabel='Position Drift [mm]',xlim=[0, self.traj.accum_distances[-1]])

        pu.plot_error_n_dim(ax, self.traj.accum_distances,self.traj.abs_errors['abs_e_trans_vec']*1000,self.dirpath)
        ax.legend()
        fig.tight_layout()
        fig.savefig(self.dirpath + '/translation_error' + FORMAT, bbox_inches="tight")
        plt.close(fig)


        #rel errors position 
        fig = plt.figure(figsize=(6, 2.5))
        ax = fig.add_subplot(111, xlabel='Distance traveled [m]',ylabel='Translation error [m]')
        pu.boxplot_compare(ax, distances, rel_errors['rel_trans'],labels, colors)
        return
        ax.legend()
        fig.tight_layout()
        fig.savefig(self.dirpath+'/rel_translation_error' + FORMAT,bbox_inches="tight")
        plt.close(fig)

        # rel errors position in percentage
        fig = plt.figure(figsize=(6, 2.5))
        ax = fig.add_subplot(
            111, xlabel='Distance traveled [m]',
            ylabel='Translation error [\%]')
        pu.boxplot_compare(
            ax, distances, rel_errors['rel_trans_perc'], labels, colors)
        fig.tight_layout()
        fig.savefig(self.dirpath+'/rel_translation_error_perc'+FORMAT,                    bbox_inches="tight")
        plt.close(fig)

        # rel errors yaw
        fig = plt.figure(figsize=(6, 2.5))
        ax = fig.add_subplot(
            111, xlabel='Distance traveled [m]',
            ylabel='Yaw error [deg]')
        pu.boxplot_compare(ax, distances, rel_errors['rel_yaw'],
                           labels, colors)
        fig.tight_layout()
        fig.savefig(self.dirpath+'/rel_yaw_error' + FORMAT,
                    bbox_inches="tight")
        plt.close(fig)


node = analyze_while_running()
node.start()
        