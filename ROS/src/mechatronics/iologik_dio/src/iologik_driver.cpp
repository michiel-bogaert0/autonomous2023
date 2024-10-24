#include "iologik_driver.hpp"
#include <mxio.h>

#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)                                                   \
  ((byte)&0x80 ? '1' : '0'), ((byte)&0x40 ? '1' : '0'),                        \
      ((byte)&0x20 ? '1' : '0'), ((byte)&0x10 ? '1' : '0'),                    \
      ((byte)&0x08 ? '1' : '0'), ((byte)&0x04 ? '1' : '0'),                    \
      ((byte)&0x02 ? '1' : '0'), ((byte)&0x01 ? '1' : '0')

iologik::iologik(ros::NodeHandle &n) : ManagedNode(n, "iologik_dio"), n_(n) {}

void iologik::doConfigure() {
  input0_pub_ = n_.advertise<std_msgs::Bool>("/input0", 10);
  input1_pub_ = n_.advertise<std_msgs::Bool>("/input1", 10);
  input2_pub_ = n_.advertise<std_msgs::Bool>("/input2", 10);
  input3_pub_ = n_.advertise<std_msgs::Bool>("/input3", 10);
  input4_pub_ = n_.advertise<std_msgs::Bool>("/input4", 10);
  input5_pub_ = n_.advertise<std_msgs::Bool>("/input5", 10);
  input6_pub_ = n_.advertise<std_msgs::Bool>("/input6", 10);
  input7_pub_ = n_.advertise<std_msgs::Bool>("/input7", 10);
  input8_pub_ = n_.advertise<std_msgs::Bool>("/input8", 10);

  output0_sub_ = n_.subscribe("/output0", 10, &iologik::output0Callback, this);
  output1_sub_ = n_.subscribe("/output1", 10, &iologik::output1Callback, this);
  output2_sub_ = n_.subscribe("/output2", 10, &iologik::output2Callback, this);
  output3_sub_ = n_.subscribe("/output3", 10, &iologik::output3Callback, this);
  output4_sub_ = n_.subscribe("/output4", 10, &iologik::output4Callback, this);
  output5_sub_ = n_.subscribe("/output5", 10, &iologik::output5Callback, this);
}

/**
 * @brief The callbacks for the output topics, these also check whether the
 * published value is within the right range and set the health status to warn
 * if it is not
 *
 * @arg msg: the float message containing the published output value
 */
void iologik::output0Callback(std_msgs::Bool msg) {
  iRet = DO_Write(iHandle, 0, 0, msg.data);
  CheckErr(iHandle, iRet, (char *)"DO0_Writes");
}
void iologik::output1Callback(std_msgs::Bool msg) {
  iRet = DO_Write(iHandle, 0, 1, msg.data);
  CheckErr(iHandle, iRet, (char *)"DO1_Writes");
}
void iologik::output2Callback(std_msgs::Bool msg) {
  iRet = DO_Write(iHandle, 0, 2, !msg.data);
  CheckErr(iHandle, iRet, (char *)"DO2_Writes");
}
void iologik::output3Callback(std_msgs::Bool msg) {
  iRet = DO_Write(iHandle, 0, 3, !msg.data);
  CheckErr(iHandle, iRet, (char *)"DO3_Writes");
}
void iologik::output4Callback(std_msgs::Bool msg) {
  iRet = DO_Write(iHandle, 0, 4, !msg.data);
  CheckErr(iHandle, iRet, (char *)"DO4_Writes");
}
void iologik::output5Callback(std_msgs::Bool msg) {
  iRet = DO_Write(iHandle, 0, 5, !msg.data);
  CheckErr(iHandle, iRet, (char *)"DO5_Writes");
}

void iologik::doActivate() { this->open(); }
void iologik::doDeactivate() { this->close(); }
void iologik::open() {
  ROS_INFO("connecting");
  iRet = MXEIO_Connect(const_cast<char *>(ip), port, timeout, &iHandle);

  CheckErr(iHandle, iRet, (char *)"MXEIO_Connect");
}

void iologik::close() {
  MXEIO_Disconnect(iHandle);
  CheckErr(iHandle, iRet, (char *)"MXEIO_Disconnect");
}

/**
 * @brief The main loop which will read in all enabled inputs and write the
 * enabled outputs if those have been changed
 *
 */
void iologik::active() {
  // Read the input registers
  DWORD dwValue = 0;

  iRet = DI_Reads(iHandle, 0, 1, 9, &dwValue);
  CheckErr(iHandle, iRet, (char *)"DI_Reads");

  // ROS_INFO(BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(dwValue));
  // ROS_INFO(BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(dwValue >> 8));
  // ROS_INFO("-----");

  std_msgs::Bool msg;
  for (int i = 0; i < 9; ++i) {
    msg.data = !((dwValue >> i) & 0x01);
    switch (i) {
    case 0:
      input0_pub_.publish(msg);
      break;
    case 1:
      input1_pub_.publish(msg);
      break;
    case 2:
      input2_pub_.publish(msg);
      break;
    case 3:
      input3_pub_.publish(msg);
      break;
    case 4:
      input4_pub_.publish(msg);
      break;
    case 5:
      input5_pub_.publish(msg);
      break;
    case 6:
      input6_pub_.publish(msg);
      break;
    case 7:
      input7_pub_.publish(msg);
      break;
    case 8:
      input8_pub_.publish(msg);
      break;
    }
  }
}

/**
 * @brief Checks for errors after each MXIO function call. If the function call
 * fails, the node will go into error and exit
 *
 * @arg iHandle: the handle for the connection
 *
 * @arg iRet: the return value of the function call
 *
 * @arg szFunctionName: the name of the function that was called
 */
void iologik::CheckErr(int iHandle, int iRet, char *szFunctionName) {
  if (iRet != MXIO_OK) {
    const char *szErrMsg;

    switch (iRet) {
    case ILLEGAL_FUNCTION:
      szErrMsg = "ILLEGAL_FUNCTION";
      break;
    case ILLEGAL_DATA_ADDRESS:
      szErrMsg = "ILLEGAL_DATA_ADDRESS";
      break;
    case ILLEGAL_DATA_VALUE:
      szErrMsg = "ILLEGAL_DATA_VALUE";
      break;
    case SLAVE_DEVICE_FAILURE:
      szErrMsg = "SLAVE_DEVICE_FAILURE";
      break;
    case SLAVE_DEVICE_BUSY:
      szErrMsg = "SLAVE_DEVICE_BUSY";
      break;
    case EIO_TIME_OUT:
      szErrMsg = "EIO_TIME_OUT";
      break;
    case EIO_INIT_SOCKETS_FAIL:
      szErrMsg = "EIO_INIT_SOCKETS_FAIL";
      break;
    case EIO_CREATING_SOCKET_ERROR:
      szErrMsg = "EIO_CREATING_SOCKET_ERROR";
      break;
    case EIO_RESPONSE_BAD:
      szErrMsg = "EIO_RESPONSE_BAD";
      break;
    case EIO_SOCKET_DISCONNECT:
      szErrMsg = "EIO_SOCKET_DISCONNECT";
      break;
    case PROTOCOL_TYPE_ERROR:
      szErrMsg = "PROTOCOL_TYPE_ERROR";
      break;
    case SIO_OPEN_FAIL:
      szErrMsg = "SIO_OPEN_FAIL";
      break;
    case SIO_TIME_OUT:
      szErrMsg = "SIO_TIME_OUT";
      break;
    case SIO_CLOSE_FAIL:
      szErrMsg = "SIO_CLOSE_FAIL";
      break;
    case SIO_PURGE_COMM_FAIL:
      szErrMsg = "SIO_PURGE_COMM_FAIL";
      break;
    case SIO_FLUSH_FILE_BUFFERS_FAIL:
      szErrMsg = "SIO_FLUSH_FILE_BUFFERS_FAIL";
      break;
    case SIO_GET_COMM_STATE_FAIL:
      szErrMsg = "SIO_GET_COMM_STATE_FAIL";
      break;
    case SIO_SET_COMM_STATE_FAIL:
      szErrMsg = "SIO_SET_COMM_STATE_FAIL";
      break;
    case SIO_SETUP_COMM_FAIL:
      szErrMsg = "SIO_SETUP_COMM_FAIL";
      break;
    case SIO_SET_COMM_TIME_OUT_FAIL:
      szErrMsg = "SIO_SET_COMM_TIME_OUT_FAIL";
      break;
    case SIO_CLEAR_COMM_FAIL:
      szErrMsg = "SIO_CLEAR_COMM_FAIL";
      break;
    case SIO_RESPONSE_BAD:
      szErrMsg = "SIO_RESPONSE_BAD";
      break;
    case SIO_TRANSMISSION_MODE_ERROR:
      szErrMsg = "SIO_TRANSMISSION_MODE_ERROR";
      break;
    case PRODUCT_NOT_SUPPORT:
      szErrMsg = "PRODUCT_NOT_SUPPORT";
      break;
    case HANDLE_ERROR:
      szErrMsg = "HANDLE_ERROR";
      break;
    case SLOT_OUT_OF_RANGE:
      szErrMsg = "SLOT_OUT_OF_RANGE";
      break;
    case CHANNEL_OUT_OF_RANGE:
      szErrMsg = "CHANNEL_OUT_OF_RANGE";
      break;
    case COIL_TYPE_ERROR:
      szErrMsg = "COIL_TYPE_ERROR";
      break;
    case REGISTER_TYPE_ERROR:
      szErrMsg = "REGISTER_TYPE_ERROR";
      break;
    case FUNCTION_NOT_SUPPORT:
      szErrMsg = "FUNCTION_NOT_SUPPORT";
      break;
    case OUTPUT_VALUE_OUT_OF_RANGE: // this can't happen (see callback for
                                    // output topics)
      szErrMsg = "OUTPUT_VALUE_OUT_OF_RANGE";
      break;
    case INPUT_VALUE_OUT_OF_RANGE: // could mean there is a broken cable/sensor
      szErrMsg = "INPUT_VALUE_OUT_OF_RANGE";
      break;
    }
    ROS_ERROR("Function \"%s\" execution Fail. Error Message : %s",
              szFunctionName, szErrMsg);
    this->setHealthStatus(diagnostic_msgs::DiagnosticStatus::ERROR, szErrMsg,
                          {});
    MXEIO_Disconnect(iHandle); // for now don't try to reconnect
    exit(1);
  }
}