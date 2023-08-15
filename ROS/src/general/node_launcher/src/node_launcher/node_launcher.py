#! /usr/bin/python3

import roslaunch
import rospkg
from pathlib import Path
import rospy


class NodeLauncher:
    def __init__(self) -> None:
        """Initialize NodeLauncher"""
        self.active: roslaunch.parent.ROSLaunchParent = None

        # Initialize uuid
        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        # Needeed according to documentation (http://docs.ros.org/en/kinetic/api/roslaunch/html/roslaunch-module.html)
        roslaunch.configure_logging(self.uuid)

        self.action_to_execute = None

    def run(self):
        
        if self.action_to_execute == 'start':
            self.active.start()

        elif self.action_to_execute == 'shutdown':
            self.shutdown()

        self.action_to_execute = ''


    def launch_node(self, package: str, launchfile: str) -> None:
        """Launch roslaunch file

        Args:
            package: Package of which launch file should be taken
            launchfile: Path of the roslaunch file relative to package location

        """
        # Convert str to Path
        launchfile = Path(launchfile)

        # Get path of package
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path(package)

        # Construct path of launch file
        rel_path = pkg_path / launchfile
        # Check if roslaunch file is syntactic correct
        rospy.logdebug(roslaunch.rlutil.check_roslaunch(str(rel_path.absolute())))

        if self.active:
            # Shutdown currently active launch
            self.active.shutdown()
            self.active = None

        # Start launch file
        self.active = roslaunch.parent.ROSLaunchParent(
            self.uuid, [str(rel_path.absolute())]
        )

        self.action_to_execute = "start"

    def shutdown(self) -> None:
        """Terminate active launchfile"""
        # Shutdown active launch
        if self.active:
            self.active.shutdown()

        # Clear active
        self.active = None
