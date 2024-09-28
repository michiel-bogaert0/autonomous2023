#!/usr/bin/env python3


import rospy
import tf2_ros as tf2
from geometry_msgs.msg import Point, TransformStamped, Vector3
from node_fixture.managed_node import ManagedNode
from std_msgs.msg import Bool, UInt16


class lap_counter(ManagedNode):
    def __init__(self):
        super().__init__("lap_counter")
        self.spin()

    def doConfigure(self):
        self.finished = False
        self.newLap = False
        self.checkFinishRange = True
        self.range = 3  # range finish zone
        self.laps = 0

        self.max_laps = rospy.get_param("~laps")
        pointx = rospy.get_param("~finishpoint_x")
        pointy = rospy.get_param("~finishpoint_y")
        pointz = rospy.get_param("~finishpoint_z")
        self.finishPoint = Point(pointx, pointy, pointz)
        self.distanceAfterFinish = rospy.get_param("~distance_after")
        self.car_pose = TransformStamped()
        self.world_frame = rospy.get_param("~world_frame", "ugr/map")
        self.base_link_frame = rospy.get_param("~base_link_frame", "ugr/car_base_link")

        self.tfBuffer = tf2.Buffer()
        self.listener = tf2.TransformListener(self.tfBuffer)

        self.lap_publisher = rospy.Publisher(
            "/lap_counter/laps_complete", UInt16, queue_size=10
        )
        self.finished_publisher = rospy.Publisher(
            "/lap_counter/finished", Bool, queue_size=1
        )

    def active(self):
        try:
            self.car_pose = self.tfBuffer.lookup_transform(
                self.world_frame, self.base_link_frame, rospy.Time(0)
            )
        except (
            tf2.LookupException,
            tf2.ConnectivityException,
            tf2.ExtrapolationException,
        ):
            rospy.logerr("error")

        carToFinishVector = Vector3()
        carToFinishVector.x = self.finishPoint.x - self.car_pose.transform.translation.x
        carToFinishVector.y = self.finishPoint.y - self.car_pose.transform.translation.y
        carToFinishVector.z = self.finishPoint.z - self.car_pose.transform.translation.z

        if self.newLap and not self.finished:
            if self.checkFinishRange and self.inFinishRange(
                carToFinishVector
            ):  # Car has entered Finishzone
                self.carEntryVector = carToFinishVector
                self.checkFinishRange = False
            # In Finish range so now we wait till it passes the FinishPoint
            elif not self.checkFinishRange and self.passedFinish(carToFinishVector):
                self.laps += 1
                self.newLap = False
                self.lap_publisher.publish(UInt16(self.laps))
            elif not self.checkFinishRange and not self.inFinishRange(
                carToFinishVector
            ):
                self.checkFinishRange = True  # if it goes out of the range again reset

        elif not self.finished and not self.inFinishRange(
            carToFinishVector
        ):  # Wait till it leaves the Finishzone
            self.newLap = True
            self.checkFinishRange = True

        if (
            self.laps >= self.max_laps
            and DotProduct(carToFinishVector, carToFinishVector)
            > self.distanceAfterFinish * self.distanceAfterFinish
        ):
            self.finished = True
            self.finished_publisher.publish(Bool(self.finished))
            self.reset()

    def passedFinish(self, carToFinishVector):
        if DotProduct(carToFinishVector, self.carEntryVector) < 0:
            return True
        return False

    def inFinishRange(self, carToFinishVector):
        if DotProduct(carToFinishVector, carToFinishVector) < self.range**2:
            return True
        return False

    def reset(self):
        self.finished = False
        self.newLap = False
        self.checkFinishRange = True
        self.laps = 0


def DotProduct(Vector1, Vector2):
    return Vector1.x * Vector2.x + Vector1.y * Vector2.y + Vector1.z * Vector2.z


node = lap_counter()
