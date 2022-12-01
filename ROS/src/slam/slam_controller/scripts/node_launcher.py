#! /usr/bin/python3

import roslaunch
import rospkg
from pathlib import Path
import rospy


class NodeLauncher:
    def __init__(self) -> None:
        self.active: roslaunch.parent.ROSLaunchParent = None

        # Initialize uuid
        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        # Needeed according to documentation (http://docs.ros.org/en/kinetic/api/roslaunch/html/roslaunch-module.html)
        roslaunch.configure_logging(self.uuid)

    def launch_node(self, package: str, launchfile: str) -> None:
        # Convert str to Path
        launchfile = Path(launchfile)

        # Get path of package to launch from
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path(package)

        # Construct path of launch file
        rel_path = pkg_path / rel_path
        # Check if roslaunch file is syntactic correct
        rospy.logdebug(roslaunch.rlutil.check_roslaunch(str(rel_path.absolute())))

        if self.active is None:
            # Shutdown currently active launch
            self.active.shutdown()
            self.active = None

        # Start launch file
        self.active = roslaunch.parent.ROSLaunchParent(
            self.uuid, [str(rel_path.absolute())]
        )
        self.active.start()

    def shutdown(self) -> None:
        # Shutdown active launch
        if self.active:
            self.active.shutdown()

        # Clear active
        self.active = None
