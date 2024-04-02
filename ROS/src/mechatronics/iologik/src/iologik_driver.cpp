#include "iologik_driver.hpp"
#include <mxio.h>

iologik::iologik(ros::NodeHandle &n)
    : ManagedNode(n, "iologik_driver"), n_(n) {}

void iologik::doConfigure() {
  input0_pub_ = n_.advertise<std_msgs::Float64>("/input0", 10);
  input1_pub_ = n_.advertise<std_msgs::Float64>("/input1", 10);
  input2_pub_ = n_.advertise<std_msgs::Float64>("/input2", 10);
  input3_pub_ = n_.advertise<std_msgs::Float64>("/input3", 10);
  input4_pub_ = n_.advertise<std_msgs::Float64>("/input4", 10);
  input5_pub_ = n_.advertise<std_msgs::Float64>("/input5", 10);
  input6_pub_ = n_.advertise<std_msgs::Float64>("/input6", 10);
  input7_pub_ = n_.advertise<std_msgs::Float64>("/input7", 10);
  output0_sub_ = n_.subscribe("/output0", 10, &iologik::output0Callback, this);
  output1_sub_ = n_.subscribe("/output1", 10, &iologik::output1Callback, this);
  n_.param<bool>("enable_i0", enable_i0, false);
  n_.param<bool>("enable_i1", enable_i1, false);
  n_.param<bool>("enable_i2", enable_i2, false);
  n_.param<bool>("enable_i3", enable_i3, false);
  n_.param<bool>("enable_i4", enable_i4, false);
  n_.param<bool>("enable_i5", enable_i5, false);
  n_.param<bool>("enable_i6", enable_i6, false);
  n_.param<bool>("enable_i7", enable_i7, false);
  n_.param<bool>("enable_o0", enable_o0, false);
  n_.param<bool>("enable_o1", enable_o1, false);
  n_.param<double>("minimum_output_current", minimum_output_current, 4);
  n_.param<double>("maximum_output_current", maximum_output_current, 20);
  n_.param<double>("minimum_input_current", minimum_input_current, 4);
  n_.param<double>("maximum_input_current", maximum_input_current, 20);
}
void iologik::output0Callback(std_msgs::Float64 msg) {
  if (msg.data < minimum_output_current || msg.data > maximum_output_current) {
    ROS_WARN("Value given to output0 of iologik is not within the right range");
    this->setHealthStatus(diagnostic_msgs::DiagnosticStatus::WARN,
                          "OUTPUT_VALUE_OUT_OF_RANGE", {});
    return; // don't change output to incorrect values
  }
  output0 = msg.data;
}
void iologik::output1Callback(std_msgs::Float64 msg) {
  if (msg.data < minimum_output_current || msg.data > maximum_output_current) {
    ROS_WARN("Value given to output1 of iologik is not within the right range");
    this->setHealthStatus(diagnostic_msgs::DiagnosticStatus::WARN,
                          "OUTPUT_VALUE_OUT_OF_RANGE", {});
    return; // don't change output to incorrect values
  }
  output1 = msg.data;
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

void iologik::active() {
  // Read the input registers
  double dwValues[8] = {0};
  iRet = AI_Reads(iHandle,   // the handle for a connection
                  0,         // unused
                  0,         // starting channel
                  8,         // read channel count
                  dwValues); // DI reading value
  CheckErr(iHandle, iRet, (char *)"DI_Reads");
  std_msgs::Float64 msg;
  for (int i = 0; i < 8; ++i) {
    msg.data = dwValues[i];
    switch (i) {
    case 0:
      if (enable_i0) {
        CheckInput(0, msg.data);
        input0_pub_.publish(msg);
      }
      break;
    case 1:
      if (enable_i1) {
        CheckInput(1, msg.data);
        input1_pub_.publish(msg);
      }
      break;
    case 2:
      if (enable_i2) {
        CheckInput(2, msg.data);
        input2_pub_.publish(msg);
      }
      break;
    case 3:
      if (enable_i3) {
        CheckInput(3, msg.data);
        input3_pub_.publish(msg);
      }
      break;
    case 4:
      if (enable_i4) {
        CheckInput(4, msg.data);
        input4_pub_.publish(msg);
      }
      break;
    case 5:
      if (enable_i5) {
        CheckInput(5, msg.data);
        input5_pub_.publish(msg);
      }
      break;
    case 6:
      if (enable_i6) {
        CheckInput(6, msg.data);
        input6_pub_.publish(msg);
      }
      break;
    case 7:
      if (enable_i7) {
        CheckInput(7, msg.data);
        input7_pub_.publish(msg);
      }
      break;
    }
  }
  // Write the output register
  if (enable_o0) {
    iRet = AO_Write(iHandle, 0,
                    0,      // Selected channel
                    output0 // Value
    );
    CheckErr(iHandle, iRet, (char *)"AO0_Writes");
  }

  if (enable_o1) {
    iRet = AO_Write(iHandle, 0,
                    1,      // Selected channel
                    output1 // Value
    );
    CheckErr(iHandle, iRet, (char *)"AO1_Writes");
  }
}

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
    case OUTPUT_VALUE_OUT_OF_RANGE: // can't happen (see callback for output
                                    // topics)
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
void iologik::CheckInput(int channel, double input) {

  if ((input < minimum_input_current) || (input > maximum_input_current)) {
    ROS_INFO("%f and %f and %f", minimum_input_current, maximum_input_current,
             input);
    ROS_ERROR("Input value of %f on channel %d not within the right range",
              input, channel);
    this->setHealthStatus(diagnostic_msgs::DiagnosticStatus::ERROR,
                          "INPUT_VALUE_OUT_OF_RANGE", {});
    MXEIO_Disconnect(iHandle); // for now don't try to reconnect
    exit(1);
  }
}
