#!/usr/bin/env python3

import string
from node_fixture.node_fixture import AddSubscriber, ROSNode
import rospy
import rospkg
from nav_msgs.msg import Odometry
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
        self.dirpath = os.getcwd() + '/data-error-correction/' 
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
        rospy.loginfo(data_gt[100])
        self.traj = Trajectory(load_data=False)
        
        self.traj.align_trajectory_with_data(data_gt, data_est, stamps_gt, stamps_est)
        
        # compute the absolute error
        self.traj.compute_absolute_error()
        
        self.traj.compute_relative_errors()
        
        # rel_error, distance = traj.get_relative_errors_and_distances(error_types='rel_trans')
        
        #the keys are the distance for rel errors
            # traj.rel_errors.keys()
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
        ax = fig.add_subplot(111, aspect='equal',
                             xlabel='x [m]', ylabel='z [m]')
        pu.plot_trajectory_top(ax, self.traj.p_es_aligned, 'b', 'Estimate')
        pu.plot_trajectory_top(ax, self.traj.p_gt, 'm', 'Groundtruth')
        plt.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.)
        fig.tight_layout()
        fig.savefig(self.dirpath+'/trajectory_top' + FORMAT, bbox_inches="tight")


        #rel errors position 
        fig = plt.figure(figsize=(6, 2.5))
        ax = fig.add_subplot(
            111, xlabel='Distance traveled [m]',
            ylabel='Translation error [m]')
        pu.boxplot_compare(ax, distances, rel_errors['rel_trans'],
                           labels, colors)
        fig.tight_layout()
        fig.savefig(self.dirpath+'/rel_translation_error' + FORMAT,
                    bbox_inches="tight")
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
        
    #     def calculateError(self):
    #         #calculate errors the data itself
    #         self.align_trajectory()

    #         self.compute_absolute_error()
    #         if self.compute_odometry_error:
    #             self.compute_relative_errors()
    #         if self.success:
    #             self.cache_current_error()

    #         # after calculating the error it shoedbe published to a topic 
    #         # later to be determained

    #         # don't know if we need it right now
    #         # mt_error = MulTrajError()
    #         # if self.success and not preset_boxplot_distances:
    #         #     #print("Save the boxplot distances for next trials.")
    #         #     preset_boxplot_distances = self.preset_boxplot_distances

    #         # if self.success:
    #         #     mt_error.addTrajectoryError(self, 0)
    #         return
             
    # def cache_current_error(self):
    #     if self.rel_errors:
    #         with open(self.cached_rel_err_fn, 'wb') as f:
    #             pickle.dump(self.rel_errors, f)
    #         print(Fore.YELLOW + "Saved relative error to {0}.".format(
    #             self.cached_rel_err_fn))

    # @staticmethod
    # def get_suffix_str(suffix):
    #     if suffix != '':
    #         return "_#"+suffix
    #     else:
    #         return suffix

    # @staticmethod
    # def remove_cached_error(data_dir, est_type='', suffix=''):
    #     #print("To remove cached error in {0}".format(data_dir))
    #     suffix_str = analyze_while_running.get_suffix_str(suffix)
    #     base_fn = analyze_while_running.rel_error_cached_nm+suffix_str+'.pickle'
    #     analyze_while_running.remove_files_in_cache_dir(data_dir, est_type, base_fn)

    # @staticmethod
    # def _safe_remove_file(abs_rm_fn):
    #     if os.path.exists(abs_rm_fn):
    #         os.remove(abs_rm_fn)
    
    # @staticmethod
    # def remove_files_in_cache_dir(data_dir, est_type, base_fn):
    #     rm_fn = os.path.join(data_dir, analyze_while_running.saved_res_dir_nm,
    #                          est_type, analyze_while_running.cache_res_dir_nm, base_fn)
    #     analyze_while_running._safe_remove_file(rm_fn)

    # @staticmethod
    # def remove_files_in_save_dir(data_dir, est_type, base_fn):
    #     rm_fn = os.path.join(data_dir, analyze_while_running.saved_res_dir_nm,
    #                          est_type, base_fn)
    #     analyze_while_running._safe_remove_file(rm_fn)

    # @staticmethod
    # def truncate(number, decimals=0):
    #     """
    #     Returns a value truncated to a specific number of decimal places.
    #     """
    #     if not isinstance(decimals, int):
    #         raise TypeError("decimal places must be an integer.")
    #     elif decimals < 0:
    #         raise ValueError("decimal places has to be 0 or more.")
    #     elif decimals == 0:
    #         return math.trunc(number)

    #     factor = 10.0 ** decimals
    #     return math.trunc(number * factor) / factor


    # def compute_boxplot_distances(self):
    #     self.preset_boxplot_distances = [self.truncate(pct*self.traj_length, 2)
    #                                       for pct in self.boxplot_pcts]
    # def align_trajectory(self):

    #     n = int(self.align_num_frames)
    #     if n < 0.0:
    #         #print('To align all frames.')
    #         n = len(self.p_es)
    #     else:
    #         print('To align trajectory using ' + str(n) + ' frames.')

    #     self.trans = np.zeros((3,))
    #     self.rot = np.eye(3)
    #     self.scale = 1.0
    #     if self.align_type == 'none':
    #         pass
    #     else:
    #         self.scale, self.rot, self.trans = au.alignTrajectory(
    #             self.p_es, self.p_gt, self.q_es, self.q_gt,
    #             self.align_type, self.align_num_frames)

    #     self.p_es_aligned = np.zeros(np.shape(self.p_es))
    #     self.q_es_aligned = np.zeros(np.shape(self.q_es))
    #     start = time.time()
    #     for i in range(np.shape(self.p_es)[0]):
    #         self.p_es_aligned[i, :] = self.scale * \
    #             self.rot.dot(self.p_es[i, :]) + self.trans
    #         q_es_R = self.rot.dot(
    #             tf.quaternion_matrix(self.q_es[i, :])[0:3, 0:3])
    #         q_es_T = np.identity(4)
    #         q_es_T[0:3, 0:3] = q_es_R
    #         self.q_es_aligned[i, :] = tf.quaternion_from_matrix(q_es_T)

    #     self.data_aligned = True
    #     #print(Fore.GREEN+"... trajectory alignment done.")

    # def compute_absolute_error(self):
    #     if self.abs_errors:
    #         print("Absolute errors already calculated")
    #     else:
    #         print(Fore.RED+'Calculating RMSE...')
    #         # align trajectory if necessary
    #         s = time.time()
    #         self.align_trajectory()
    #         e_trans, e_trans_vec, e_rot, e_ypr, e_scale_perc =\
    #             traj_err.compute_absolute_error(self.p_es_aligned,
    #                                             self.q_es_aligned,
    #                                             self.p_gt,
    #                                             self.q_gt)
    #         stats_trans = res_writer.compute_statistics(e_trans)
    #         stats_rot = res_writer.compute_statistics(e_rot)
    #         stats_scale = res_writer.compute_statistics(e_scale_perc)
    #         e = time.time()
    #         print(e-s)
    #         self.abs_errors['abs_e_trans'] = e_trans
    #         self.abs_errors['abs_e_trans_stats'] = stats_trans

    #         self.abs_errors['abs_e_trans_vec'] = e_trans_vec

    #         self.abs_errors['abs_e_rot'] = e_rot
    #         self.abs_errors['abs_e_rot_stats'] = stats_rot

    #         self.abs_errors['abs_e_ypr'] = e_ypr

    #         self.abs_errors['abs_e_scale_perc'] = e_scale_perc
    #         self.abs_errors['abs_e_scale_stats'] = stats_scale
    #         print(Fore.GREEN+'...RMSE calculated.')
    #     return

    # def compute_relative_error_at_subtraj_len(self, subtraj_len,
    #                                           max_dist_diff=-1):
    #     if max_dist_diff < 0:
    #         max_dist_diff = 0.2 * subtraj_len

    #     if self.rel_errors and (subtraj_len in self.rel_errors):
    #         x = 1
    #         #print("Relative error at sub-trajectory length {0} is already "
    #         #      "computed or loaded from cache.".format(subtraj_len))
    #     else:
    #         #print("Computing relative error at sub-trajectory "
    #         #      "length {0}".format(subtraj_len))
    #         Tcm = np.identity(4)
    #         _, e_trans, e_trans_perc, e_yaw, e_gravity, e_rot, e_rot_deg_per_m =\
    #             traj_err.compute_relative_error(
    #                 self.p_es, self.q_es, self.p_gt, self.q_gt, Tcm,
    #                 subtraj_len, max_dist_diff, self.accum_distances,
    #                 self.scale)
    #         dist_rel_err = {'rel_trans': e_trans,
    #                         'rel_trans_stats':
    #                         res_writer.compute_statistics(e_trans),
    #                         'rel_trans_perc': e_trans_perc,
    #                         'rel_trans_perc_stats':
    #                         res_writer.compute_statistics(e_trans_perc),
    #                         'rel_rot': e_rot,
    #                         'rel_rot_stats':
    #                         res_writer.compute_statistics(e_rot),
    #                         'rel_yaw': e_yaw,
    #                         'rel_yaw_stats':
    #                         res_writer.compute_statistics(e_yaw),
    #                         'rel_gravity': e_gravity,
    #                         'rel_gravity_stats':
    #                         res_writer.compute_statistics(e_gravity),
    #                         'rel_rot_deg_per_m': e_rot_deg_per_m,
    #                         'rel_rot_deg_per_m_stats':
    #                         res_writer.compute_statistics(e_rot_deg_per_m)}
    #         self.rel_errors[subtraj_len] = dist_rel_err
    #     return True

    # def compute_relative_errors(self, subtraj_lengths=[]):
    #     suc = True
    #     if subtraj_lengths:
    #         for l in subtraj_lengths:
    #             suc = suc and self.compute_relative_error_at_subtraj_len(l)
    #     else:
    #         print(Fore.RED+"Computing the relative errors based on preset"
    #               " subtrajectory lengths...")
    #         for l in self.preset_boxplot_distances:
    #             suc = suc and self.compute_relative_error_at_subtraj_len(l)
    #     self.success = suc
    #     print(Fore.GREEN+"...done.")

    # def get_relative_errors_and_distances(
    #         self, error_types=['rel_trans', 'rel_trans_perc', 'rel_yaw']):
    #     rel_errors = {}
    #     for err_i in error_types:
    #         assert err_i in kRelMetrics
    #         rel_errors[err_i] = [[self.rel_errors[d][err_i]
    #                              for d in self.preset_boxplot_distances]]
    #     return rel_errors, self.preset_boxplot_distances
    #endregion