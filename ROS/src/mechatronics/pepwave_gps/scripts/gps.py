#!/usr/bin/env python3

import datetime
import math
import socket
import time
from time import time

import rospy
from dateutil import tz
from sensor_msgs.msg import NavSatFix, NavSatStatus


class GPSPublisher:
    def __init__(self):
        # ros initialization
        rospy.init_node("pep_wave_gps")
        self.publisher = rospy.Publisher("/output/gps", NavSatFix, queue_size=10)

        self.ip = rospy.get_param("~ip_address", "192.168.50.1")
        self.port = rospy.get_param("~port", 60660)
        self.gps_antenna_frame = rospy.get_param("~gps_frame", "ugr/car_base_link/gps")
        self.use_gps_time = rospy.get_param("~use_gps_time", False)

        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.connect((self.ip, self.port))

        while True:
            # Check for external shutdown
            if rospy.is_shutdown():
                return

            # Reference for this code : https://forum.peplink.com/t/accessing-gps-info-from-pepwave-max-routers/8262/1

            data = self.client_socket.recv(1024)
            if len(data) > 0:
                lines = data.decode("ascii").splitlines(1)

                for line in lines:
                    gpsstring = line.split(",")

                    # See https://docs.novatel.com/OEM7/Content/Logs/GPRMC.htm for format of GPRMC message
                    if gpsstring[0] == "$GPRMC":
                        # Get the Lat and Long and put it into a sensor_msgs/NavSatFix
                        navsatfix_msg = NavSatFix()

                        # Calculate unix from UTC timestmamp & date
                        if self.use_gps_time:
                            timestr = gpsstring[1]
                            datestr = gpsstring[9]

                            utc = datetime.datetime(
                                int(datestr[-2:]) + 2000,
                                int(datestr[2:4]),
                                int(datestr[:2]),
                                int(timestr[:2]),
                                int(timestr[2:4]),
                                int(timestr[4:6]),
                            )

                            utc = utc.replace(tzinfo=tz.tzutc())
                            utc = utc.astimezone(tz.tzlocal())

                            navsatfix_msg.header.stamp = rospy.Time.from_sec(
                                time.mktime(utc.timetuple())
                            )
                        else:
                            navsatfix_msg.header.stamp = rospy.Time.now()

                        navsatfix_msg.header.frame_id = self.gps_antenna_frame

                        navsatfix_msg.status.service = NavSatStatus.SERVICE_GPS
                        navsatfix_msg.status.status = (
                            NavSatStatus.STATUS_SBAS_FIX
                            if gpsstring[2] == "A"
                            else NavSatStatus.STATUS_NO_FIX
                        )

                        navsatfix_msg.latitude = float(gpsstring[3][:2]) + float(
                            gpsstring[3][2:]
                        ) / 60 * (1 if gpsstring[4] == "N" else -1)
                        navsatfix_msg.longitude = float(gpsstring[5][:3]) + float(
                            gpsstring[5][3:]
                        ) / 60 * (1 if gpsstring[6] == "E" else -1)

                        navsatfix_msg.altitude = math.nan

                        navsatfix_msg.position_covariance_type = (
                            NavSatFix.COVARIANCE_TYPE_UNKNOWN
                        )

                        self.publisher.publish(navsatfix_msg)


if __name__ == "__main__":
    try:
        cp = GPSPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
