
import os
import time
from multiple_traj_errors import MulTrajError
import yaml
import math
from sensor_msgs.msg import Imu, NavSatFix
from ROS.src.general.node_fixture.src.node_fixture.node_fixture import AddSubscriber, ROSNode
import rospy
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
import pickle

import numpy as np
from colorama import init, Fore

import trajectory_utils as traj_utils
import trajectory_loading as traj_loading
import results_writer as res_writer
import compute_trajectory_errors as traj_err
import align_utils as au
from metrics import kRelMetrics, kRelMetricLables

import transformations as tf

FORMAT = '.pdf'
class location:
    x = 0.0
    y = 0.0
    z = 0.0
    qx = 0.0
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
    preset_boxplot_percentages=[]
    
    def __init__(self, results_dir):
        super().__init__("AnalyzeWhileRunning")
        self.publish("errors", Odometry)
        assert os.path.exists(results_dir), "directory{0} doesn't exist".format(results_dir)
        self.result_dir = results_dir
        self.uid = self.platform + '_' + self.alg + '_' + self.dataset_short_name
        self.data_dir = results_dir
        self.data_loaded = False
        self.data_aligned = False
        self.success = False

        #creation of directory to store the data
        self.saved_results_dir = os.path.join(
            os.path.join(self.data_dir, self.saved_res_dir_nm),
            self.est_type)
        if not os.path.exists(self.saved_results_dir):
            os.makedirs(self.saved_results_dir)

        self.cache_results_dir = os.path.join(
            self.saved_results_dir, self.cache_res_dir_nm)
        if not os.path.exists(self.cache_results_dir):
            os.makedirs(self.cache_results_dir)

        self.eval_cfg = os.path.join(self.data_dir, 'eval_cfg.yaml')
        
        if os.path.exists(self.eval_cfg):
            with open(self.eval_cfg, 'r') as f:
                eval_cfg = yaml.load(f, Loader=yaml.FullLoader)
                self.align_type = eval_cfg['align_type']
                self.align_num_frames = eval_cfg['align_num_frames']
        self.align_str = self.align_type + '_' + str(self.align_num_frames)

        self.start_end_time_fn = os.path.join(self.data_dir, 'start_end_time.yaml')
        self.start_time_sec = -float('inf')
        self.end_time_sec = float('inf')
        if os.path.exists(self.start_end_time_fn):
            #print("Find start end time for evaluation.")
            with open(self.start_end_time_fn, 'r') as f:
                d = yaml.load(f, Loader=yaml.FullLoader)
                if 'start_time_sec' in d:
                    self.start_time_sec = d['start_time_sec']
                if 'end_time_sec' in d:
                    self.end_time_sec = d['end_time_sec']

        self.abs_errors = {}

        self.rel_errors = {}
        self.cached_rel_err_fn = os.path.join(
            self.cache_results_dir,
            self.rel_error_cached_nm+self.suffix_str+".pickle")

        self.boxplot_pcts = self.preset_boxplot_percentages

        #data to start with
        self.odomData = []
        self.gpsData = []
        self.previousData = []

        @AddSubscriber("/input/odometry")
        def addDataOdom(self, msg: Odometry):
            """
            setting the previous data from odom to add until the gps data has been recieved
            """
            self.previousData = [
                msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z,
                msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z
                ]
        
        @AddSubscriber("/input/gps")
        def addDataGps(self, msg: Odometry):
            """
                adding the gpsdata as path the same from odomdata as a path
                
            """
            self.gpsData.append([
                msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z,
                msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z
                ])
            self.odomData.append(self.previousData)
            self.accum_distances = traj_utils.get_distance_from_start(self.p_gt_raw)
            self.traj_length = self.accum_distances[-1]
            self.accum_distances = traj_utils.get_distance_from_start(self.p_gt)
            self.calculateError()
        
        def calculateError(self):
            #calculate errors the data itself
            self.align_trajectory()

            self.compute_absolute_error()
            if self.compute_odometry_error:
                self.compute_relative_errors()
            if self.success:
                self.cache_current_error()

            # after calculating the error it shoedbe published to a topic 
            # later to be determained

            # don't know if we need it right now
            # mt_error = MulTrajError()
            # if self.success and not preset_boxplot_distances:
            #     #print("Save the boxplot distances for next trials.")
            #     preset_boxplot_distances = self.preset_boxplot_distances

            # if self.success:
            #     mt_error.addTrajectoryError(self, 0)
            return
             
    def cache_current_error(self):
        if self.rel_errors:
            with open(self.cached_rel_err_fn, 'wb') as f:
                pickle.dump(self.rel_errors, f)
            print(Fore.YELLOW + "Saved relative error to {0}.".format(
                self.cached_rel_err_fn))

    @staticmethod
    def get_suffix_str(suffix):
        if suffix != '':
            return "_#"+suffix
        else:
            return suffix

    @staticmethod
    def remove_cached_error(data_dir, est_type='', suffix=''):
        #print("To remove cached error in {0}".format(data_dir))
        suffix_str = analyze_while_running.get_suffix_str(suffix)
        base_fn = analyze_while_running.rel_error_cached_nm+suffix_str+'.pickle'
        analyze_while_running.remove_files_in_cache_dir(data_dir, est_type, base_fn)

    @staticmethod
    def _safe_remove_file(abs_rm_fn):
        if os.path.exists(abs_rm_fn):
            os.remove(abs_rm_fn)
    
    @staticmethod
    def remove_files_in_cache_dir(data_dir, est_type, base_fn):
        rm_fn = os.path.join(data_dir, analyze_while_running.saved_res_dir_nm,
                             est_type, analyze_while_running.cache_res_dir_nm, base_fn)
        analyze_while_running._safe_remove_file(rm_fn)

    @staticmethod
    def remove_files_in_save_dir(data_dir, est_type, base_fn):
        rm_fn = os.path.join(data_dir, analyze_while_running.saved_res_dir_nm,
                             est_type, base_fn)
        analyze_while_running._safe_remove_file(rm_fn)

    @staticmethod
    def truncate(number, decimals=0):
        """
        Returns a value truncated to a specific number of decimal places.
        """
        if not isinstance(decimals, int):
            raise TypeError("decimal places must be an integer.")
        elif decimals < 0:
            raise ValueError("decimal places has to be 0 or more.")
        elif decimals == 0:
            return math.trunc(number)

        factor = 10.0 ** decimals
        return math.trunc(number * factor) / factor


    def compute_boxplot_distances(self):
        self.preset_boxplot_distances = [self.truncate(pct*self.traj_length, 2)
                                          for pct in self.boxplot_pcts]
    def align_trajectory(self):

        n = int(self.align_num_frames)
        if n < 0.0:
            #print('To align all frames.')
            n = len(self.p_es)
        else:
            print('To align trajectory using ' + str(n) + ' frames.')

        self.trans = np.zeros((3,))
        self.rot = np.eye(3)
        self.scale = 1.0
        if self.align_type == 'none':
            pass
        else:
            self.scale, self.rot, self.trans = au.alignTrajectory(
                self.p_es, self.p_gt, self.q_es, self.q_gt,
                self.align_type, self.align_num_frames)

        self.p_es_aligned = np.zeros(np.shape(self.p_es))
        self.q_es_aligned = np.zeros(np.shape(self.q_es))
        start = time.time()
        for i in range(np.shape(self.p_es)[0]):
            self.p_es_aligned[i, :] = self.scale * \
                self.rot.dot(self.p_es[i, :]) + self.trans
            q_es_R = self.rot.dot(
                tf.quaternion_matrix(self.q_es[i, :])[0:3, 0:3])
            q_es_T = np.identity(4)
            q_es_T[0:3, 0:3] = q_es_R
            self.q_es_aligned[i, :] = tf.quaternion_from_matrix(q_es_T)

        self.data_aligned = True
        #print(Fore.GREEN+"... trajectory alignment done.")

    def compute_absolute_error(self):
        if self.abs_errors:
            print("Absolute errors already calculated")
        else:
            print(Fore.RED+'Calculating RMSE...')
            # align trajectory if necessary
            s = time.time()
            self.align_trajectory()
            e_trans, e_trans_vec, e_rot, e_ypr, e_scale_perc =\
                traj_err.compute_absolute_error(self.p_es_aligned,
                                                self.q_es_aligned,
                                                self.p_gt,
                                                self.q_gt)
            stats_trans = res_writer.compute_statistics(e_trans)
            stats_rot = res_writer.compute_statistics(e_rot)
            stats_scale = res_writer.compute_statistics(e_scale_perc)
            e = time.time()
            print(e-s)
            self.abs_errors['abs_e_trans'] = e_trans
            self.abs_errors['abs_e_trans_stats'] = stats_trans

            self.abs_errors['abs_e_trans_vec'] = e_trans_vec

            self.abs_errors['abs_e_rot'] = e_rot
            self.abs_errors['abs_e_rot_stats'] = stats_rot

            self.abs_errors['abs_e_ypr'] = e_ypr

            self.abs_errors['abs_e_scale_perc'] = e_scale_perc
            self.abs_errors['abs_e_scale_stats'] = stats_scale
            print(Fore.GREEN+'...RMSE calculated.')
        return

    def compute_relative_error_at_subtraj_len(self, subtraj_len,
                                              max_dist_diff=-1):
        if max_dist_diff < 0:
            max_dist_diff = 0.2 * subtraj_len

        if self.rel_errors and (subtraj_len in self.rel_errors):
            x = 1
            #print("Relative error at sub-trajectory length {0} is already "
            #      "computed or loaded from cache.".format(subtraj_len))
        else:
            #print("Computing relative error at sub-trajectory "
            #      "length {0}".format(subtraj_len))
            Tcm = np.identity(4)
            _, e_trans, e_trans_perc, e_yaw, e_gravity, e_rot, e_rot_deg_per_m =\
                traj_err.compute_relative_error(
                    self.p_es, self.q_es, self.p_gt, self.q_gt, Tcm,
                    subtraj_len, max_dist_diff, self.accum_distances,
                    self.scale)
            dist_rel_err = {'rel_trans': e_trans,
                            'rel_trans_stats':
                            res_writer.compute_statistics(e_trans),
                            'rel_trans_perc': e_trans_perc,
                            'rel_trans_perc_stats':
                            res_writer.compute_statistics(e_trans_perc),
                            'rel_rot': e_rot,
                            'rel_rot_stats':
                            res_writer.compute_statistics(e_rot),
                            'rel_yaw': e_yaw,
                            'rel_yaw_stats':
                            res_writer.compute_statistics(e_yaw),
                            'rel_gravity': e_gravity,
                            'rel_gravity_stats':
                            res_writer.compute_statistics(e_gravity),
                            'rel_rot_deg_per_m': e_rot_deg_per_m,
                            'rel_rot_deg_per_m_stats':
                            res_writer.compute_statistics(e_rot_deg_per_m)}
            self.rel_errors[subtraj_len] = dist_rel_err
        return True

    def compute_relative_errors(self, subtraj_lengths=[]):
        suc = True
        if subtraj_lengths:
            for l in subtraj_lengths:
                suc = suc and self.compute_relative_error_at_subtraj_len(l)
        else:
            print(Fore.RED+"Computing the relative errors based on preset"
                  " subtrajectory lengths...")
            for l in self.preset_boxplot_distances:
                suc = suc and self.compute_relative_error_at_subtraj_len(l)
        self.success = suc
        print(Fore.GREEN+"...done.")

    def get_relative_errors_and_distances(
            self, error_types=['rel_trans', 'rel_trans_perc', 'rel_yaw']):
        rel_errors = {}
        for err_i in error_types:
            assert err_i in kRelMetrics
            rel_errors[err_i] = [[self.rel_errors[d][err_i]
                                 for d in self.preset_boxplot_distances]]
        return rel_errors, self.preset_boxplot_distances
