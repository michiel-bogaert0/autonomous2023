
import os
import argparse

import numpy as np
import matplotlib.pyplot as plt
import plot_utils as pu
from matplotlib import rc
import matplotlib
from colorama import init, Fore

from trajectory import Trajectory
import plot_utils as pu
from fn_constants import kNsToEstFnMapping, kNsToMatchFnMapping, kFnExt
from multiple_traj_errors import MulTrajError

FORMAT = '.pdf'

class analyze_while_running:
    platform=''
    alg_name=''
    dataset_name=''
    align_type='sim3'
    align_num_frames=-1
    suffix=''
    est_type='traj_est'
    nm_gt='stamped_groundtruth.txt'
    nm_est='stamped_traj_estimate.txt'
    nm_matches='stamped_est_gt_matches.txt'
    preset_boxplot_distances=[]
    preset_boxplot_percentages=[]
    
    def __init__(self, results_dir):
        assert os.path.exists(results_dir), "directory{0} doesnt exist".format(results_dir)
        self.result_dir = results_dir

    
    
    def printDataToPdf(self):
        #there is onlyy on path for this so the for loop doesn't do a thing
        #for est_type_i, plot_dir_i in zip(args.est_types, results_dir):
        est_type_i, plot_dir_i = zip(self.est_type, self.result_dir)
        print(Fore.RED +
              "#### Processing error type {0} ####".format(est_type_i))
        mt_error = MulTrajError()
        traj_list, mt_error = analyze_multiple_trials(
            args.result_dir, est_type_i, n_trials, args.recalculate_errors)
        if traj_list:
            plot_traj = traj_list[args.mul_plot_idx[0]]
        else:
            print("No success runs, not plotting.")

        if n_trials > 1:
            print(">>> Save results for multiple runs in {0}...".format(
                mt_error.save_results_dir))
            mt_error.saveErrors()
            mt_error.cache_current_error()

        if not args.plot:
            print("#### Skip plotting and go to next error type.")
            return

        print(Fore.MAGENTA +
              ">>> Plotting absolute error for one trajectory...")
        fig = plt.figure(figsize=(6, 5.5))
        ax = fig.add_subplot(111, aspect='equal',
                             xlabel='x [m]', ylabel='y [m]')
        pu.plot_trajectory_top(ax, plot_traj.p_es_aligned, 'b', 'Estimate')
        pu.plot_trajectory_top(ax, plot_traj.p_gt, 'm', 'Groundtruth')
        pu.plot_aligned_top(ax, plot_traj.p_es_aligned, plot_traj.p_gt,
                            plot_traj.align_num_frames)
        plt.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.)
        fig.tight_layout()
        fig.savefig(plot_dir_i+'/trajectory_top' + '_' + plot_traj.align_str +
                    FORMAT, bbox_inches="tight")

        fig = plt.figure(figsize=(6, 5.5))
        ax = fig.add_subplot(111, aspect='equal',
                             xlabel='x [m]', ylabel='z [m]')
        pu.plot_trajectory_side(ax, plot_traj.p_es_aligned, 'b', 'Estimate')
        pu.plot_trajectory_side(ax, plot_traj.p_gt, 'm', 'Groundtruth')
        plt.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.)
        fig.tight_layout()
        fig.savefig(plot_dir_i+'/trajectory_side' + '_' + plot_traj.align_str +
                    FORMAT, bbox_inches="tight")

        fig = plt.figure(figsize=(8, 2.5))
        ax = fig.add_subplot(
            111, xlabel='Distance [m]', ylabel='Position Drift [mm]',
            xlim=[0, plot_traj.accum_distances[-1]])
        pu.plot_error_n_dim(ax, plot_traj.accum_distances,
                            plot_traj.abs_errors['abs_e_trans_vec']*1000,
                            plot_dir_i)
        ax.legend()
        fig.tight_layout()
        fig.savefig(plot_dir_i+'/translation_error' + '_' + plot_traj.align_str
                    + FORMAT, bbox_inches="tight")

        fig = plt.figure(figsize=(8, 2.5))
        ax = fig.add_subplot(
            111, xlabel='Distance [m]', ylabel='Orient. err. [deg]',
            xlim=[0, plot_traj.accum_distances[-1]])
        pu.plot_error_n_dim(
            ax, plot_traj.accum_distances,
            plot_traj.abs_errors['abs_e_ypr']*180.0/np.pi, plot_dir_i,
            labels=['yaw', 'pitch', 'roll'])
        ax.legend()
        fig.tight_layout()
        fig.savefig(plot_dir_i+'/rotation_error'+'_'+plot_traj.align_str +
                    FORMAT, bbox_inches='tight')

        fig = plt.figure(figsize=(8, 2.5))
        ax = fig.add_subplot(
            111, xlabel='Distance [m]', ylabel='Scale Drift [\%]',
            xlim=[0, plot_traj.accum_distances[-1]])
        pu.plot_error_n_dim(
            ax, plot_traj.accum_distances,
            np.reshape(plot_traj.abs_errors['abs_e_scale_perc'], (-1, 1)),
            plot_dir_i, colors=['b'], labels=['scale'])
        ax.legend()
        fig.tight_layout()
        fig.savefig(plot_dir_i+'/scale_error'+'_'+plot_traj.align_str+FORMAT,
                    bbox_inches='tight')

        if args.plot_scale_traj:
            fig = plt.figure(figsize=(6, 12))
            ax_top = fig.add_subplot(211, aspect='equal',
                                     xlabel='x [m]', ylabel='y [m]',
                                     title='Top')
            ax_top.grid(ls='--', color='0.7')
            ax_side = fig.add_subplot(212, aspect='equal',
                                      xlabel='x [m]', ylabel='z [m]',
                                      title='Side')
            ax_side.grid(ls='--', color='0.7')
            abs_scale_e = np.abs(
                np.reshape(plot_traj.abs_errors['abs_e_scale_perc'], (-1, 1)),)
            color_idx =\
                (abs_scale_e-np.min(abs_scale_e))/(
                    np.max(abs_scale_e)-np.min(abs_scale_e))
            for idx, val in enumerate(color_idx[:-1]):
                c = matplotlib.cm.jet(val).flatten()
                ax_top.plot(plot_traj.p_gt[idx:idx+2, 0],
                            plot_traj.p_gt[idx:idx+2, 1], color=c)
                ax_side.plot(plot_traj.p_gt[idx:idx+2, 0],
                             plot_traj.p_gt[idx:idx+2, 2], color=c)
            fig.tight_layout()
            fig.savefig(plot_dir_i+'/scale_error_traj' + '_' +
                        plot_traj.align_str + FORMAT, bbox_inches="tight")

        print(Fore.MAGENTA+">>> Plotting relative (odometry) error...")
        suffix = ''
        if n_trials > 1:
            suffix = '_mt'

        plot_types = ['rel_trans', 'rel_trans_perc', 'rel_yaw']
        rel_errors, distances = mt_error.get_relative_errors_and_distances(
            error_types=plot_types)

        labels = ['Estimate']
        colors = ['b']

        fig = plt.figure(figsize=(6, 2.5))
        ax = fig.add_subplot(
            111, xlabel='Distance traveled [m]',
            ylabel='Translation error [m]')
        pu.boxplot_compare(ax, distances, rel_errors['rel_trans'],
                           labels, colors)
        fig.tight_layout()
        fig.savefig(plot_dir_i+'/rel_translation_error' + suffix + FORMAT,
                    bbox_inches="tight")
        plt.close(fig)

        fig = plt.figure(figsize=(6, 2.5))
        ax = fig.add_subplot(
            111, xlabel='Distance traveled [m]',
            ylabel='Translation error [\%]')
        pu.boxplot_compare(
            ax, distances, rel_errors['rel_trans_perc'], labels, colors)
        fig.tight_layout()
        fig.savefig(plot_dir_i+'/rel_translation_error_perc'+suffix+FORMAT,
                    bbox_inches="tight")
        plt.close(fig)

        fig = plt.figure(figsize=(6, 2.5))
        ax = fig.add_subplot(
            111, xlabel='Distance traveled [m]',
            ylabel='Yaw error [deg]')
        pu.boxplot_compare(ax, distances, rel_errors['rel_yaw'],
                           labels, colors)
        fig.tight_layout()
        fig.savefig(plot_dir_i+'/rel_yaw_error' + suffix + FORMAT,
                    bbox_inches="tight")
        plt.close(fig)

        print(Fore.GREEN +
              "#### Done processing error type {0} ####".format(est_type_i))
    import subprocess as s
    s.call(['notify-send', 'rpg_trajectory_evaluation finished',
            'results in: {0}'.format(os.path.abspath(args.result_dir))])

