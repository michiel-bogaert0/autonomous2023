#!/usr/bin/env python3

import math
import os
import pickle

import align_utils as au
import compute_trajectory_errors as traj_err
import numpy as np
import results_writer as res_writer
import trajectory_loading as traj_loading
import trajectory_utils as traj_utils
import transformations as tf
import yaml
from colorama import Fore
from metrics import kRelMetricLables, kRelMetrics


# class Trajectory(ROSNode):
class Trajectory:
    rel_error_cached_nm = "cached_rel_err"
    rel_error_prefix = "relative_error_statistics_"
    saved_res_dir_nm = "saved_results"
    cache_res_dir_nm = "cached"
    default_boxplot_perc = [0.1, 0.2, 0.3, 0.4, 0.5]

    def __init__(
        self,
        results_dir="",
        platform="",
        alg_name="",
        dataset_name="",
        align_type="sim3",
        align_num_frames=-1,
        suffix="",
        est_type="traj_est",
        nm_gt="stamped_groundtruth.txt",
        nm_est="stamped_traj_estimate.txt",
        nm_matches="stamped_est_gt_matches.txt",
        preset_boxplot_distances=None,
        preset_boxplot_percentages=None,
        load_data=True,
    ):
        if preset_boxplot_distances is None:
            preset_boxplot_distances = []

        if preset_boxplot_percentages is None:
            preset_boxplot_distances = []

        if load_data:
            assert os.path.exists(
                results_dir
            ), "Specified directory {0} does not exist.".format(results_dir)
            assert align_type in ["first_frame", "sim3", "se3"]
        self.platform = platform
        self.alg = alg_name
        self.dataset_short_name = dataset_name
        self.uid = self.platform + "_" + self.alg + "_" + self.dataset_short_name
        self.est_type = est_type
        self.suffix_str = self.get_suffix_str(suffix)
        self.success = False

        self.data_dir = results_dir
        self.data_loaded = False
        self.data_aligned = False
        self.saved_results_dir = os.path.join(
            os.path.join(self.data_dir, Trajectory.saved_res_dir_nm), self.est_type
        )
        if not os.path.exists(self.saved_results_dir):
            os.makedirs(self.saved_results_dir)

        self.cache_results_dir = os.path.join(
            self.saved_results_dir, Trajectory.cache_res_dir_nm
        )
        if not os.path.exists(self.cache_results_dir):
            os.makedirs(self.cache_results_dir)

        self.align_type = align_type
        self.align_num_frames = int(align_num_frames)

        self.eval_cfg = os.path.join(self.data_dir, "eval_cfg.yaml")

        if os.path.exists(self.eval_cfg):
            print("Find evaluation configuration, will overwrite default.")
            with open(self.eval_cfg, "r") as f:
                eval_cfg = yaml.load(f, Loader=yaml.FullLoader)
                print("The current evaluation configuration is " "{0}".format(eval_cfg))
                self.align_type = eval_cfg["align_type"]
                self.align_num_frames = eval_cfg["align_num_frames"]
        self.align_str = self.align_type + "_" + str(self.align_num_frames)

        self.start_end_time_fn = os.path.join(self.data_dir, "start_end_time.yaml")
        self.start_time_sec = -float("inf")
        self.end_time_sec = float("inf")
        if os.path.exists(self.start_end_time_fn):
            with open(self.start_end_time_fn, "r") as f:
                d = yaml.load(f, Loader=yaml.FullLoader)
                if "start_time_sec" in d:
                    self.start_time_sec = d["start_time_sec"]
                if "end_time_sec" in d:
                    self.end_time_sec = d["end_time_sec"]

        self.abs_errors = {}

        # we cache relative error since it is time-comsuming to compute
        self.rel_errors = {}
        self.cached_rel_err_fn = os.path.join(
            self.cache_results_dir,
            self.rel_error_cached_nm + self.suffix_str + ".pickle",
        )

        if load_data:
            self.data_loaded = self.load_data(nm_gt, nm_est, nm_matches)
            self.boxplot_pcts = preset_boxplot_percentages
            if len(preset_boxplot_distances) != 0:
                self.preset_boxplot_distances = preset_boxplot_distances
            else:
                if not self.boxplot_pcts:
                    self.boxplot_pcts = Trajectory.default_boxplot_perc
                self.compute_boxplot_distances()

            if not self.data_loaded:
                print(Fore.RED + "Loading data failed.")
                return
            self.align_trajectory()

    def align_trajectory_with_data(self, data_gt, data_est, stamps_gt, stamps_est):
        self.data_loaded = True
        (
            self.t_es,
            self.p_es,
            self.q_es,
            self.t_gt,
            self.p_gt,
            self.q_gt,
        ) = traj_loading.load_estimate_and_associate_only_data(
            stamps_gt, stamps_est, data_est, data_gt, 0.02
        )
        self.t_gt_raw, self.p_gt_raw, self.q_gt_raw = traj_loading.get_raw_groundtruth(
            data_gt
        )
        self.accum_distances = traj_utils.get_distance_from_start(self.p_gt_raw)
        self.traj_length = self.accum_distances[-1]
        self.accum_distances = traj_utils.get_distance_from_start(self.p_gt)
        self.align_trajectory()
        self.boxplot_pcts = []

        if len(self.boxplot_pcts) != 0:
            self.preset_boxplot_distances = self.boxplot_pcts
        else:
            if not self.boxplot_pcts:
                self.boxplot_pcts = Trajectory.default_boxplot_perc
            self.compute_boxplot_distances()

    def load_data(self, nm_gt, nm_est, nm_matches):
        """
        Loads the trajectory data. The resuls {p_es, q_es, p_gt, q_gt} is
        synchronized and has the same length.
        """
        print(self.data_dir)
        if not os.path.exists(os.path.join(self.data_dir, nm_gt)) or not os.path.exists(
            os.path.join(self.data_dir, nm_est)
        ):
            print(Fore.RED + "Either groundtruth or estimate does not exist")
            return False

        print(Fore.RED + "Loading trajectory data...")

        # only timestamped pose series is supported
        # this can be changed by the info getted from odometry
        (
            self.t_es,
            self.p_es,
            self.q_es,
            self.t_gt,
            self.p_gt,
            self.q_gt,
        ) = traj_loading.load_stamped_dataset(
            self.data_dir,
            nm_gt,
            nm_est,
            os.path.join(Trajectory.saved_res_dir_nm, self.est_type, nm_matches),
            start_t_sec=self.start_time_sec,
            end_t_sec=self.end_time_sec,
        )
        self.t_gt_raw, self.p_gt_raw, self.q_gt_raw = traj_loading.load_raw_groundtruth(
            self.data_dir,
            nm_gt,
            start_t_sec=self.start_time_sec,
            end_t_sec=self.end_time_sec,
        )
        if self.p_es.size == 0:
            print(Fore.RED + "Empty estimate file.")
            return False
        self.accum_distances = traj_utils.get_distance_from_start(self.p_gt_raw)
        self.traj_length = self.accum_distances[-1]
        self.accum_distances = traj_utils.get_distance_from_start(self.p_gt)

        if os.path.isfile(self.cached_rel_err_fn):
            print(
                "Loading cached relative (odometry) errors from "
                + self.cached_rel_err_fn
            )
            with open(self.cached_rel_err_fn, "rb") as f:
                self.rel_errors = pickle.load(f)
            print(
                "Loaded odometry error calcualted at {0}".format(self.rel_errors.keys())
            )

        return True

    def cache_current_error(self):
        if self.rel_errors:
            with open(self.cached_rel_err_fn, "wb") as f:
                pickle.dump(self.rel_errors, f)
            print(
                Fore.YELLOW
                + "Saved relative error to {0}.".format(self.cached_rel_err_fn)
            )

    @staticmethod
    def get_suffix_str(suffix):
        if suffix != "":
            return "_#" + suffix
        else:
            return suffix

    @staticmethod
    def remove_cached_error(data_dir, est_type="", suffix=""):
        suffix_str = Trajectory.get_suffix_str(suffix)
        base_fn = Trajectory.rel_error_cached_nm + suffix_str + ".pickle"
        Trajectory.remove_files_in_cache_dir(data_dir, est_type, base_fn)

    @staticmethod
    def _safe_remove_file(abs_rm_fn):
        if os.path.exists(abs_rm_fn):
            os.remove(abs_rm_fn)

    @staticmethod
    def remove_files_in_cache_dir(data_dir, est_type, base_fn):
        rm_fn = os.path.join(
            data_dir,
            Trajectory.saved_res_dir_nm,
            est_type,
            Trajectory.cache_res_dir_nm,
            base_fn,
        )
        Trajectory._safe_remove_file(rm_fn)

    @staticmethod
    def remove_files_in_save_dir(data_dir, est_type, base_fn):
        rm_fn = os.path.join(data_dir, Trajectory.saved_res_dir_nm, est_type, base_fn)
        Trajectory._safe_remove_file(rm_fn)

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

        factor = 10.0**decimals
        return math.trunc(number * factor) / factor

    def compute_boxplot_distances(self):
        self.preset_boxplot_distances = [
            self.truncate(pct * self.traj_length, 2) for pct in self.boxplot_pcts
        ]

    def align_trajectory(self):
        if self.data_aligned:
            return
        n = int(self.align_num_frames)
        if n < 0.0:
            n = len(self.p_es)
        else:
            print("To align trajectory using " + str(n) + " frames.")

        self.trans = np.zeros((3,))
        self.rot = np.eye(3)
        self.scale = 1.0
        if self.align_type == "none":
            pass
        else:
            self.scale, self.rot, self.trans = au.alignTrajectory(
                self.p_es,
                self.p_gt,
                self.q_es,
                self.q_gt,
                self.align_type,
                self.align_num_frames,
            )

        self.p_es_aligned = np.zeros(np.shape(self.p_es))
        self.q_es_aligned = np.zeros(np.shape(self.q_es))
        for i in range(np.shape(self.p_es)[0]):
            self.p_es_aligned[i, :] = (
                self.scale * self.rot.dot(self.p_es[i, :]) + self.trans
            )
            q_es_R = self.rot.dot(tf.quaternion_matrix(self.q_es[i, :])[0:3, 0:3])
            q_es_T = np.identity(4)
            q_es_T[0:3, 0:3] = q_es_R
            self.q_es_aligned[i, :] = tf.quaternion_from_matrix(q_es_T)

        self.data_aligned = True

    def compute_absolute_error(self):
        if self.abs_errors:
            print("Absolute errors already calculated")
        else:
            self.align_trajectory()
            (
                e_trans,
                e_trans_vec,
                e_rot,
                e_ypr,
                e_scale_perc,
            ) = traj_err.compute_absolute_error(
                self.p_es_aligned, self.q_es_aligned, self.p_gt, self.q_gt
            )
            stats_trans = res_writer.compute_statistics(e_trans)
            stats_rot = res_writer.compute_statistics(e_rot)
            stats_scale = res_writer.compute_statistics(e_scale_perc)

            self.abs_errors["abs_e_trans"] = e_trans
            self.abs_errors["abs_e_trans_stats"] = stats_trans

            self.abs_errors["abs_e_trans_vec"] = e_trans_vec

            self.abs_errors["abs_e_rot"] = e_rot
            self.abs_errors["abs_e_rot_stats"] = stats_rot

            self.abs_errors["abs_e_ypr"] = e_ypr

            self.abs_errors["abs_e_scale_perc"] = e_scale_perc
            self.abs_errors["abs_e_scale_stats"] = stats_scale
        return

    def write_errors_to_yaml(self):
        self.abs_err_stats_fn = os.path.join(
            self.saved_results_dir,
            "absolute_err_statistics"
            + "_"
            + self.align_str
            + self.suffix_str
            + ".yaml",
        )
        res_writer.update_and_save_stats(
            self.abs_errors["abs_e_trans_stats"], "trans", self.abs_err_stats_fn
        )
        res_writer.update_and_save_stats(
            self.abs_errors["abs_e_rot_stats"], "rot", self.abs_err_stats_fn
        )
        res_writer.update_and_save_stats(
            self.abs_errors["abs_e_scale_stats"], "scale", self.abs_err_stats_fn
        )

        self.rel_error_stats_fns = []
        for dist in self.rel_errors:
            cur_err = self.rel_errors[dist]
            dist_str = "{:3.1f}".format(dist).replace(".", "_")
            dist_fn = os.path.join(
                self.saved_results_dir,
                Trajectory.rel_error_prefix + dist_str + self.suffix_str + ".yaml",
            )
            for et, label in zip(kRelMetrics, kRelMetricLables):
                res_writer.update_and_save_stats(cur_err[et + "_stats"], label, dist_fn)

            self.rel_error_stats_fns.append(dist_fn)

    def compute_relative_error_at_subtraj_len(self, subtraj_len, max_dist_diff=-1):
        if max_dist_diff < 0:
            max_dist_diff = 0.2 * subtraj_len

        if self.rel_errors and (subtraj_len in self.rel_errors):
            pass
        else:
            Tcm = np.identity(4)
            (
                _,
                e_trans,
                e_trans_perc,
                e_yaw,
                e_gravity,
                e_rot,
                e_rot_deg_per_m,
            ) = traj_err.compute_relative_error(
                self.p_es,
                self.q_es,
                self.p_gt,
                self.q_gt,
                Tcm,
                subtraj_len,
                max_dist_diff,
                self.accum_distances,
                self.scale,
            )
            dist_rel_err = {
                "rel_trans": e_trans,
                "rel_trans_stats": res_writer.compute_statistics(e_trans),
                "rel_trans_perc": e_trans_perc,
                "rel_trans_perc_stats": res_writer.compute_statistics(e_trans_perc),
                "rel_rot": e_rot,
                "rel_rot_stats": res_writer.compute_statistics(e_rot),
                "rel_yaw": e_yaw,
                "rel_yaw_stats": res_writer.compute_statistics(e_yaw),
                "rel_gravity": e_gravity,
                "rel_gravity_stats": res_writer.compute_statistics(e_gravity),
                "rel_rot_deg_per_m": e_rot_deg_per_m,
                "rel_rot_deg_per_m_stats": res_writer.compute_statistics(
                    e_rot_deg_per_m
                ),
            }
            self.rel_errors[subtraj_len] = dist_rel_err
        return True

    def compute_relative_errors(self, subtraj_lengths=None):
        if subtraj_lengths is None:
            subtraj_lengths = []

        suc = True
        if subtraj_lengths:
            for length in subtraj_lengths:
                suc = suc and self.compute_relative_error_at_subtraj_len(length)
        else:
            for length in self.preset_boxplot_distances:
                suc = suc and self.compute_relative_error_at_subtraj_len(length)
        self.success = suc

    def get_relative_errors_and_distances(self, error_types=None):
        if error_types is None:
            error_types = ["rel_trans", "rel_trans_perc", "rel_yaw"]

        rel_errors = {}
        for err_i in error_types:
            assert err_i in kRelMetrics
            rel_errors[err_i] = [
                [self.rel_errors[d][err_i] for d in self.preset_boxplot_distances]
            ]
        return rel_errors, self.preset_boxplot_distances

    def get_relative_errors_and_distance_no_ignore(self, error_types=None):
        if error_types is None:
            error_types = ["rel_trans", "rel_trans_perc", "rel_yaw"]

        rel_errors = {}
        for err_i in error_types:
            rel_errors[err_i] = [
                [self.rel_errors[d][err_i] for d in self.preset_boxplot_distances]
            ]
        return rel_errors, self.preset_boxplot_distances
