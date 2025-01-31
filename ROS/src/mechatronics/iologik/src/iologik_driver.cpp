#include "iologik_driver.hpp"
#include <mxio.h>

iologik::iologik(ros::NodeHandle &n) : ManagedNode(n, "iologik_aio"), n_(n) {}

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
  n_.param<bool>("enable_i0", enable_inputs[0], false);
  n_.param<bool>("enable_i1", enable_inputs[1], false);
  n_.param<bool>("enable_i2", enable_inputs[2], false);
  n_.param<bool>("enable_i3", enable_inputs[3], false);
  n_.param<bool>("enable_i4", enable_inputs[4], false);
  n_.param<bool>("enable_i5", enable_inputs[5], false);
  n_.param<bool>("enable_i6", enable_inputs[6], false);
  n_.param<bool>("enable_i7", enable_inputs[7], false);

  n_.param<float>("scaling_i0", scaling_inputs[0], 1.0);
  n_.param<float>("scaling_i1", scaling_inputs[1], 1.0);
  n_.param<float>("scaling_i2", scaling_inputs[2], 1.0);
  n_.param<float>("scaling_i3", scaling_inputs[3], 1.0);
  n_.param<float>("scaling_i4", scaling_inputs[4], 1.0);
  n_.param<float>("scaling_i5", scaling_inputs[5], 1.0);
  n_.param<float>("scaling_i6", scaling_inputs[6], 1.0);
  n_.param<float>("scaling_i7", scaling_inputs[7], 1.0);

  std::vector<bool> enables = {
      enable_inputs[0], enable_inputs[1], enable_inputs[2], enable_inputs[3],
      enable_inputs[4], enable_inputs[5], enable_inputs[6], enable_inputs[7]};
  for (int i = 0; i < enables.size(); i++) { // find the index of the lowest
                                             // enabled input
    if (enables[i]) {
      if (start_channel == -1) {
        start_channel = i;
      }
      end_channel = i;
    }
  }
  enabled_channels = end_channel - start_channel + 1;
  n_.param<bool>("enable_o0", enable_o0, false);
  n_.param<bool>("enable_o1", enable_o1, false);
  n_.param<double>("minimum_output_current", minimum_output_current, 4);
  n_.param<double>("maximum_output_current", maximum_output_current, 20);
  n_.param<double>("minimum_input_current", minimum_input_current, 4);
  n_.param<double>("maximum_input_current", maximum_input_current, 20);
}

/**
 * @brief The callbacks for the output topics, these also check whether the
 * published value is within the right range and set the health status to warn
 * if it is not
 *
 * @arg msg: the float message containing the published output value
 */
void iologik::output0Callback(std_msgs::Float64 msg) {
  if (msg.data < minimum_output_current || msg.data > maximum_output_current) {
    ROS_WARN("Value given to output0 of iologik is not within the right range");
    this->setHealthStatus(diagnostic_msgs::DiagnosticStatus::WARN,
                          "OUTPUT_VALUE_OUT_OF_RANGE", {});
    return; // don't change output to incorrect values
  }
  this->setHealthStatus(diagnostic_msgs::DiagnosticStatus::OK, "OK", {});
  output0 = msg.data;
  o0_changed = true;
}
void iologik::output1Callback(std_msgs::Float64 msg) {
  if (msg.data < minimum_output_current || msg.data > maximum_output_current) {
    ROS_WARN("Value given to output1 of iologik is not within the right range");
    this->setHealthStatus(diagnostic_msgs::DiagnosticStatus::WARN,
                          "OUTPUT_VALUE_OUT_OF_RANGE", {});
    return; // don't change output to incorrect values
  }
  this->setHealthStatus(diagnostic_msgs::DiagnosticStatus::OK, "OK", {});
  output1 = msg.data;
  o1_changed = true;
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
  double dwValues[8] = {0};
  if (start_channel != -1) {
    iRet = AI_Reads(iHandle,          // the handle for a connection
                    0,                // unused
                    start_channel,    // starting channel
                    enabled_channels, // read channel count
                    dwValues); // DI reading value, make sure the inputs get
                               // written at the correct index
    CheckErr(iHandle, iRet, (char *)"DI_Reads");
    std_msgs::Float64 msg;
    uint8_t pointer = 0;

    // ROS_INFO("%d", enabled_channels);

    for (int i = 0; i < 8; ++i) {
      msg.data = dwValues[i - start_channel];

      if (enable_inputs[i]) {

        if (i != 1 && i != 4) {
          CheckInput(i, msg.data);

          msg.data -= 4;
          msg.data /= 16;
          msg.data *= scaling_inputs[i];
        }
        switch (i) {
        case 0:
          pointer++;
          input0_pub_.publish(msg);
          break;
        case 1:
          pointer++;
          input1_pub_.publish(msg);
          break;
        case 2:
          pointer++;
          input2_pub_.publish(msg);
          break;
        case 3:
          pointer++;
          input3_pub_.publish(msg);
          break;
        case 4:
          pointer++;
          input4_pub_.publish(msg);
          break;
        case 5:
          pointer++;
          input5_pub_.publish(msg);
          break;
        case 6:
          pointer++;
          input6_pub_.publish(msg);
          break;
        case 7:
          pointer++;
          input7_pub_.publish(msg);
          break;
        }
      }
    }
  }
  // Write the output register
  if (enable_o0 && o0_changed) {
    iRet = AO_Write(iHandle, 0,
                    0,      // Selected channel
                    output0 // Value
    );
    CheckErr(iHandle, iRet, (char *)"AO0_Writes");
    o0_changed = false;
  }

  if (enable_o1 && o1_changed) {
    iRet = AO_Write(iHandle, 0,
                    1,      // Selected channel
                    output1 // Value
    );
    CheckErr(iHandle, iRet, (char *)"AO1_Writes");
    o1_changed = false;
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

/**
 * @brief Checks whether the input values are within the right range. If this is
 * not the case, the node will go into error and exit
 *
 * @arg channel: the index of the input channel that is being checked
 *
 * @arg input: the input value that is being checked
 */
void iologik::CheckInput(int channel, double input) {

  if ((input < minimum_input_current) || (input > maximum_input_current)) {
    // go into error when the input values are not within the right range
    ROS_ERROR("Input value of %f on channel %d not within the right range",
              input, channel);
    this->setHealthStatus(diagnostic_msgs::DiagnosticStatus::ERROR,
                          "INPUT_VALUE_OUT_OF_RANGE", {});
    MXEIO_Disconnect(iHandle); // for now don't try to reconnect
    exit(1);
  }
}
