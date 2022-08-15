#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

#define NR_OF_ARGS 8

extern "C"
{
#include "ntripclient.c"
}


/**
 * Little ROScpp wrapper around the C ntripclient
 */
int main(int argc, char **argv)
{

    ros::init(argc, argv, "ublox_gps_rtcm_streamer");

    ros::NodeHandle n("~");

    std::string server, mountpoint, device, user, password, port, baudrate;

    // Get parameters to feed to ntripclient
    n.param<std::string>("server", server, "rtk2go.com");
    n.param<std::string>("port", port, "2101");
    n.param<std::string>("mountpoint", mountpoint, "bie001");

    n.param<std::string>("user", user, "");
    n.param<std::string>("password", password, "");

    n.param<std::string>("baudrate", baudrate, "460800");
    n.param<std::string>("device", device, "/dev/ttyUSB0");

    std::string str_args[NR_OF_ARGS] = {
        "ntripclient",
        "-s" + server,
        "-u " + user,
        "-p " + password,
        "-r " + port,
        "-m " + mountpoint,
        "-B " + baudrate,
        "-D " + device};

    char *args[NR_OF_ARGS + 1];

    for (int i = 0; i < NR_OF_ARGS; i++)
    {
        args[i] = (char *)str_args[i].c_str();
    }
    args[NR_OF_ARGS] = NULL;

    ntripclient_main(NR_OF_ARGS, args);

    return 0;
}