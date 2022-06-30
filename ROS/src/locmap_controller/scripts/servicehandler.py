#!/usr/bin/env python3

#!WIP

from locmap_controller.srv import *
import rospy
import roslaunch

from collections import deque


class ServiceHandler:
    """
    This class handles the service calls from locmap_controller
    """

    ID = 0
    launch_pointers = {}
    launch_uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    tasks_que = deque([])

    def __init__(self) -> None:

        rospy.init_node("locmap_controller_servicehandler")

        roslaunch.configure_logging(ServiceHandler.launch_uuid)

        # Setup services
        rospy.Service("locmap_controller/Launch", Launch, ServiceHandler.handle_launch)
        rospy.Service("locmap_controller/Stop", Stop, ServiceHandler.handle_stop)

        rospy.loginfo("LocMap Controller Servicehandler is ready to serve!")

        while not rospy.is_shutdown():
            self.handle_tasks()
            rospy.sleep(0.2)

    def handle_tasks(self):
        """
        This function is responsible for handling tasks that should be done in the main thread,
        instead of in the service call threads.

        It basically handles all tasks in the task_que
        """

        while len(ServiceHandler.tasks_que) != 0:
            taskname, data = ServiceHandler.tasks_que.popleft()

            if taskname == "start":
                ServiceHandler.launch_pointers[data].start()
                rospy.logdebug(f"Started node with id '{data}'")
            elif taskname == "stop":
                ServiceHandler.launch_pointers[data].shutdown()
                rospy.logdebug(f"Stopped node with id '{data}'")
            else:
                rospy.logerr(f"Task '{taskname}' not recognized")

    @staticmethod
    def handle_stop(req: StopRequest):
        """
        Handles a stoppage of a launch.
        It keeps the object and ID so that it can be restarted later (with the same config of course)
        """
        if req.id not in ServiceHandler.launch_pointers:
            return StopResponse(success=False)

        ServiceHandler.tasks_que.append(("stop", req.id))

        return StopResponse(success=True)

    @staticmethod
    def handle_launch(req: LaunchRequest):
        """
        Handles the Launch service of locmap_controller
        Basically exposes roslaunch functionality as a service

        Inputs:
            req: LaunchRequest - the request message

        Returns: LaunchResponse
        """

        filename = req.filename.strip()
        filename = filename if filename.endswith(".launch") else f"{filename}.launch"

        cli_args = ["locmap_controller", filename]

        for arg in req.args.split(" "):
            if arg != "":
                cli_args.append(arg)

        roslaunch_args = cli_args[2:]
        roslaunch_file = [
            (
                roslaunch.rlutil.resolve_launch_arguments(cli_args)[0],
                roslaunch_args,
            )
        ]

        launch_pointer = roslaunch.parent.ROSLaunchParent(
            ServiceHandler.launch_uuid, roslaunch_file
        )

        ServiceHandler.launch_pointers[ServiceHandler.ID] = launch_pointer
        ServiceHandler.ID += 1

        ServiceHandler.tasks_que.append(("start", ServiceHandler.ID - 1))

        return LaunchResponse(id=ServiceHandler.ID - 1)


node = ServiceHandler()
