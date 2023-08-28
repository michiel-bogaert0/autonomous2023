#!/usr/bin/env python3

import datetime
import socket
import time
from math import atan2, pi

import rospy
import serial
import utm
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import (PoseWithCovarianceStamped,
                               TwistWithCovarianceStamped)
from nav_msgs.msg import Odometry
from node_fixture.node_fixture import create_diagnostic_message
from sensor_msgs.msg import Imu, NavSatFix, TimeReference
from tf.transformations import quaternion_from_euler
from ubxtranslator.core import Parser
from ubxtranslator.predefined import ACK_CLS, NAV_CLS


class ParseUblox:
    def __init__(self):
        """
        This node parses UBX messages from a source

        Args:
            source: can be a port or a raw binary file
            baud: baudrate in case port is a source
            is_serial: set to True if using a port as source, False if using a file
            gps_frame_id: the frame to attach to the ROS messages. Should be the frame of the antenna
            fixed_only_mode: Set this to True to only publish messages that are free of any ambiguity.
                             If False, it will publish those, but with an unvalid fix status (0)
            use_ntrip: set to True if you want to stream RTCM data (via a socket based ntrip caster) to the device
            ntrip/source: If use_ntrip is True, this address is used to fetch RTCM data
            ntrip/port: If use_ntrip is True, this port is used to fetch RTCM data
        """
        # ros initialization
        rospy.init_node("parse_ubx_msgs")
        self.diag_publisher = rospy.Publisher(
            "/diagnostics", DiagnosticArray, queue_size=10
        )
        self.pvtfix_publisher = rospy.Publisher(
            "/output/pvt/fix", NavSatFix, queue_size=10
        )
        self.pvtodom_publisher = rospy.Publisher(
            "/output/pvt/odom", Odometry, queue_size=10
        )
        self.relposodom_publisher = rospy.Publisher(
            "/output/relposned/odom", Odometry, queue_size=10
        )

        # Arguments
        self.source_name = rospy.get_param("~source", "/dev/ttyUSB0")
        self.baud = rospy.get_param("~baud", "460800")
        self.use_serial = rospy.get_param("~is_serial", True)
        self.frame_id = rospy.get_param("~gps_frame_id", "ugr/car_base_link/gps0")
        self.fixed_only_mode = rospy.get_param("~fixed_only_mode", False)
        self.use_ntrip = rospy.get_param("~use_ntrip", False)
        self.socket_addr = rospy.get_param("~ntrip/source", "192.168.50.36")
        self.socket_port = rospy.get_param("~ntrip/port", 50010)

        self.started = False

        # Setup serial communication
        if self.use_serial:
            self.source = serial.Serial(self.source_name, baudrate=self.baud)
        else:
            self.source = open(self.source_name, "br")

        if self.use_ntrip:
            # Create a TCP/IP socket to receive the datastream
            # Try to connect to it before moving on
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.setblocking(False)

            sock_connected = False

            while not sock_connected:
                try:
                    SERVER_ADDRESS = (
                        rospy.get_param("~ntrip/source", "192.168.50.36"),
                        rospy.get_param("~ntrip/port", 50000),
                    )

                    # Connect the socket to the server address and port
                    self.sock.connect(SERVER_ADDRESS)
                    sock_connected = True

                    rospy.loginfo("Connected to {} port {}".format(*SERVER_ADDRESS))

                except Exception as e:
                    self.diag_publisher.publish(
                        create_diagnostic_message(
                            level=DiagnosticStatus.ERROR,
                            name=f"[GPS {self.source_name}] NTRIP connection failed",
                            message=str(e),
                        )
                    )
                    time.sleep(1)

        # Parser
        self.parser = Parser([NAV_CLS, ACK_CLS])

        print("UBLOX parser (", self.source_name, ") ready...")

        while not rospy.is_shutdown():
            self.parse()

            if self.use_ntrip:
                try:
                    data = self.sock.recv(1024)
                    self.diag_publisher.publish(
                        create_diagnostic_message(
                            level=DiagnosticStatus.OK,
                            name=f"[GPS {self.source_name}] NTRIP data received length",
                            message=f"{len(data)}",
                        )
                    )
                    self.source.write(data)
                except Exception as e:
                    self.diag_publisher.publish(
                        create_diagnostic_message(
                            level=DiagnosticStatus.WARN,
                            name=f"[GPS {self.source_name}] Could not receive NTRIP data. Is the NTRIP caster still online?",
                            message=str(e),
                        )
                    )

        self.source.close()

    def parse(self):
        """
        Parses message from source and publishes corresponding messages.
        Should be called periodically

        Also see:
            - Sharepoint
            - https://content.u-blox.com/sites/default/files/products/documents/u-blox8-M8_ReceiverDescrProtSpec_UBX-13003221.pdf
        """

        try:
            msg = self.parser.receive_from(self.source)

            if not self.started:
                print(self.source_name, " started!")
                self.started = True

            # Convert to ROS msg and send!
            klasse, subclass, data = msg

            # https://content.u-blox.com/sites/default/files/products/documents/u-blox8-M8_ReceiverDescrProtSpec_UBX-13003221.pdf#page=390&zoom=100,0,0
            if subclass == "PVT":  # Simple PVT solution
                if data.flags.gnssFixOK != 1:
                    self.diag_publisher.publish(
                        create_diagnostic_message(
                            level=DiagnosticStatus.ERROR,
                            name=f"[GPS {self.source_name}] UBX-NAV-PVT",
                            message=f"GNSS solution invalid",
                        )
                    )

                    return

                if data.flags.carrSoln != 2:
                    self.diag_publisher.publish(
                        create_diagnostic_message(
                            level=DiagnosticStatus.WARN,
                            name=f"[GPS {self.source_name}] UBX-NAV-PVT",
                            message=f"Solution mode: RTK FLOAT",
                        )
                    )

                    if self.fixed_only_mode:
                        return

                else:
                    self.diag_publisher.publish(
                        create_diagnostic_message(
                            level=DiagnosticStatus.OK,
                            name=f"[GPS {self.source_name}] UBX-NAV-PVT",
                            message=f"Solution mode: RTK FIX",
                        )
                    )

                ## PVT (Position, Velocity, Time)
                # Publishes
                # NavSatFix:
                # - long/lat

                rosmsg = NavSatFix()
                rosmsg.header.frame_id = self.frame_id

                rosmsg.latitude = data.lat / 10**7
                rosmsg.longitude = data.lon / 10**7
                rosmsg.altitude = data.height / 1000

                rosmsg.status.status = 0 if data.flags.carrSoln == 2 else -1

                self.pvtfix_publisher.publish(rosmsg)

                # Odometry:
                # - velocity
                # - position in utm
                # - heading (estimated using motion)

                x, y, _, _ = utm.from_latlon(rosmsg.latitude, rosmsg.longitude)
                rosmsg = Odometry()
                rosmsg.header.frame_id = self.frame_id
                rosmsg.pose.pose.position.x = x
                rosmsg.pose.pose.position.y = y

                # Velocity
                rosmsg.twist.twist.linear.x = data.velE / 1000
                rosmsg.twist.twist.linear.y = data.velN / 1000
                rosmsg.twist.twist.linear.z = -1 * data.velD / 1000

                # Heading estimation headMot
                q = quaternion_from_euler(
                    0,
                    0,
                    data.headMot * 2 * pi / 360,
                )
                rosmsg.pose.pose.orientation.x = q[0]
                rosmsg.pose.pose.orientation.y = q[1]
                rosmsg.pose.pose.orientation.z = q[2]
                rosmsg.pose.pose.orientation.w = q[3]

                self.pvtodom_publisher.publish(rosmsg)

            # https://content.u-blox.com/sites/default/files/products/documents/u-blox8-M8_ReceiverDescrProtSpec_UBX-13003221.pdf#page=390&zoom=100,0,0
            elif subclass == "RELPOSNED":  # Relative positioning (in NED frame)]]
                if data.flags.gnssFixOK != 1:
                    self.diag_publisher.publish(
                        create_diagnostic_message(
                            level=DiagnosticStatus.ERROR,
                            name=f"[GPS {self.source_name}] UBX-NAV-RELPOSNED",
                            message=f"GNSS solution invalid",
                        ),
                    )

                    return

                if data.flags.carrSoln != 2:
                    self.diag_publisher.publish(
                        create_diagnostic_message(
                            level=DiagnosticStatus.WARN,
                            name=f"[GPS {self.source_name}] UBX-NAV-RELPOSNED",
                            message=f"Solution mode: FLOAT",
                        )
                    )

                    if self.fixed_only_mode:
                        return
                else:
                    self.diag_publisher.publish(
                        create_diagnostic_message(
                            level=DiagnosticStatus.OK,
                            name=f"[GPS {self.source_name}] UBX-NAV-RELPOSNED",
                            message=f"Solution mode: FIX",
                        )
                    )

                ## RELPOSNED
                # Odometry: (relative to base station)
                # - position
                # - heading

                # We should publish in ENU frame (see REP103/REP105)
                rosmsg = Odometry()
                rosmsg.header.frame_id = self.frame_id

                # Position
                rosmsg.pose.pose.position.y = (
                    data.relPosN + data.relPosHPN / 100
                ) / 100
                rosmsg.pose.pose.position.x = (
                    data.relPosE + data.relPosHPE / 100
                ) / 100
                rosmsg.pose.pose.position.z = (
                    -1 * (data.relPosD + data.relPosHPD / 100) / 100
                )

                # Heading
                q = quaternion_from_euler(
                    0,
                    0,
                    atan2(rosmsg.pose.pose.position.y, rosmsg.pose.pose.position.x),
                )
                rosmsg.pose.pose.orientation.x = q[0]
                rosmsg.pose.pose.orientation.y = q[1]
                rosmsg.pose.pose.orientation.z = q[2]
                rosmsg.pose.pose.orientation.w = q[3]

                self.relposodom_publisher.publish(rosmsg)

        except (ValueError, IOError) as err:
            pass


if __name__ == "__main__":
    try:
        parser = ParseUblox()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
