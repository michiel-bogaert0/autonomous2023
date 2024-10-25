import datetime
import os

import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import rospy

########################################################################################################################
# PLOTTING FUNCTIONS:
# - make_legend_arrow
# - create_plot_name_with_timestamp
# - save_plot_to_folder
# - clear_folder
# - normalize_color

########################################################################################################################


def make_legend_arrow(legend, orig_handle, xdescent, ydescent, width, height, fontsize):
    """
    author:
    Kwinten Mortier

    .. description:
    This function is used to create a custom legend handler for matplotlib. It creates an arrow in the legend.

    .. inputs:
    :param width:       The width of the arrow.
    :type width:        float
    :param height:      The height of the arrow.
    :type height:       float

    .. output:
    :return p:          The arrow patch.
    :rtype p:           matplotlib.patches.FancyArrow

    .. notes:
    The arrow is created using the matplotlib.patches.FancyArrow class. This function can be further generalized if needed.
    For now, this is fine.
    """

    p = mpatches.FancyArrow(
        0, 0.5 * height, width, 0, length_includes_head=True, head_width=0.75 * height
    )
    return p


def create_plot_name_with_timestamp(
    plot_name: str,
) -> str:
    """
    author:
    Kwinten Mortier

    .. description::
    Creates a plot name with a timestamp. The timestamp layout is 'YYYYMMDD_HHMMSS'.

    .. inputs::
    :param plot_name:               name of the plot.
    :type plot_name:                str

    .. outputs::
    :return plot_name_t_stamp:      name of the plot with a timestamp.
    :rtype plot_name_t_stamp:       str
    """

    # Get current date and time
    t_stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S").lower()

    # Create plot name with timestamp
    plot_name_t_stamp = f"{plot_name}_{t_stamp}.png"

    return plot_name_t_stamp


def save_plot_to_folder(
    plot: plt.figure,
    folder_path: str,
    plot_filename: str = "plot",
):
    """
    author:
    Kwinten Mortier

    .. description::
    Save a plot to a specified folder.

    .. inputs::
    :param plot:                plot to save.
    :type plot:                 plt.figure.Figure
    :param folder_path:         factor for smoothing the trajectory.
    :type folder_path:          float
    :param plot_filename:       factor for smoothing the trajectory.
    :type plot_filename:        float

    .. outputs::
    None

    .. notes::
    This function saves the plot, no outputs!
    """

    # Create folder if it does not exist
    if not os.path.exists(folder_path):
        os.makedirs(folder_path)

        rospy.loginfo(f"Folder '{folder_path}' was created.")

    # Create plot name with timestamp
    plot_filename_t_stamp = create_plot_name_with_timestamp(plot_filename)

    # Save plot to folder
    plot.savefig(os.path.join(folder_path, plot_filename_t_stamp), dpi=1200)


def clear_folder(folder_path: str):
    """
    author:
    Kwinten Mortier

    .. description::
    Clear the contents of a folder.

    .. inputs::
    :param folder_path:         path to the folder to clear.
    :type folder_path:          str

    .. outputs::
    None

    .. notes::
    This function clears the contents of the folder, no outputs!
    """

    # Check if folder exists
    if os.path.exists(folder_path):
        # Iterate over the files in the folder
        for filename in os.listdir(folder_path):
            file_path = os.path.join(folder_path, filename)
            # Check if the path is a file
            if os.path.isfile(file_path):
                # Remove the file
                os.remove(file_path)

        rospy.loginfo(f"Folder '{folder_path}' was cleared.")

    else:
        rospy.loginfo(f"Folder '{folder_path}' does not exist.")


def normalize_color(color: tuple) -> tuple:
    """
    Created by:
    Kwinten Mortier

    .. description::
    Normalize a tuple representing a RGB color to be used in matplotlib.

    .. inputs::
    :param color:                   tuple containing the RGB values of a color
    :type color:                    tuple

    .. outputs::
    :return color_normalized:       tuple containing the normalized RGB values of a color
    :rtype color_normalized:        tuple
    """

    # normalize the color
    color_normalized = (color[0] / 255, color[1] / 255, color[2] / 255)

    return color_normalized
