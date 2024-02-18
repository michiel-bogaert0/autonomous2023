import rospy
from std_msgs.msg import Float32, String

messages = {
    "<class 'float'>": lambda msg: Float32(msg),
    # "<class 'cantools.database.can.signal.NamedSignalValue'>" : lambda msg: String(msg)
    "<class 'cantools.database.can.signal.NamedSignalValue'>": lambda msg: str(msg),
}

publishers = {
    "Actual_ERPM": rospy.Publisher("/processed/actual_erpm", Float32, queue_size=10),
    "Actual_Duty": rospy.Publisher("/processed/actual_duty", Float32, queue_size=10),
    "Actual_InputVoltage": rospy.Publisher(
        "/processed/actual_input_voltage", Float32, queue_size=10
    ),
    "Actual_ACCurrent": rospy.Publisher(
        "/processed/actual_ac_current", Float32, queue_size=10
    ),
    "Actual_DCCurrent": rospy.Publisher(
        "/processed/actual_dc_current", Float32, queue_size=10
    ),
    "Actual_TempController": rospy.Publisher(
        "/processed/actual_temp_controller", Float32, queue_size=10
    ),
    "Actual_TempMotor": rospy.Publisher(
        "/processed/actual_temp_motor", Float32, queue_size=10
    ),
    "Actual_FaultCode": rospy.Publisher(
        "/processed/actual_fault_code", String, queue_size=10
    ),
    "Actual_FOC_id": rospy.Publisher(
        "/processed/actual_foc_id", Float32, queue_size=10
    ),
    "Actual_FOC_iq": rospy.Publisher(
        "/processed/actual_foc_iq", Float32, queue_size=10
    ),
    "Actual_Throttle": rospy.Publisher(
        "/processed/actual_throttle", Float32, queue_size=10
    ),
    "Actual_Brake": rospy.Publisher("/processed/actual_brake", Float32, queue_size=10),
    "Digital_input_1": rospy.Publisher(
        "/processed/digital_input_1", Float32, queue_size=10
    ),
    "Digital_input_2": rospy.Publisher(
        "/processed/digital_input_2", Float32, queue_size=10
    ),
    "Digital_input_3": rospy.Publisher(
        "/processed/digital_input_3", Float32, queue_size=10
    ),
    "Digital_input_4": rospy.Publisher(
        "/processed/digital_input_4", Float32, queue_size=10
    ),
    "Digital_output_1": rospy.Publisher(
        "/processed/digital_output_1", Float32, queue_size=10
    ),
    "Digital_output_2": rospy.Publisher(
        "/processed/digital_output_2", Float32, queue_size=10
    ),
    "Digital_output_3": rospy.Publisher(
        "/processed/digital_output_3", Float32, queue_size=10
    ),
    "Digital_output_4": rospy.Publisher(
        "/processed/digital_output_4", Float32, queue_size=10
    ),
    "Drive_enable": rospy.Publisher("/processed/drive_enable", Float32, queue_size=10),
    "Capacitor_temp_limit": rospy.Publisher(
        "/processed/capacitor_temp_limit", Float32, queue_size=10
    ),
    "DC_current_limit": rospy.Publisher(
        "/processed/dc_current_limit", Float32, queue_size=10
    ),
    "Drive_enable_limit": rospy.Publisher(
        "/processed/drive_enable_limit", Float32, queue_size=10
    ),
    "IGBT_accel_limit": rospy.Publisher(
        "/processed/igbt_accel_limit", Float32, queue_size=10
    ),
    "IGBT_temp_limit": rospy.Publisher(
        "/processed/igbt_temp_limit", Float32, queue_size=10
    ),
    "Input_voltage_limit": rospy.Publisher(
        "/processed/input_voltage_limit", Float32, queue_size=10
    ),
    "Motor_accel_limit": rospy.Publisher(
        "/processed/motor_accel_limit", Float32, queue_size=10
    ),
    "Motor_temp_limit": rospy.Publisher(
        "/processed/motor_temp_limit", Float32, queue_size=10
    ),
    "RPM_min_limit": rospy.Publisher(
        "/processed/rpm_min_limit", Float32, queue_size=10
    ),
    "RPM_max_limit": rospy.Publisher(
        "/processed/rpm_max_limit", Float32, queue_size=10
    ),
    "Power_limit": rospy.Publisher("/processed/power_limit", Float32, queue_size=10),
    "CAN_map_version": rospy.Publisher(
        "/processed/can_map_version", Float32, queue_size=10
    ),
    "CMD_TargetAcCurrent": rospy.Publisher(
        "/processed/cmd_target_ac_current", Float32, queue_size=10
    ),
    "CMD_TargetSpeed": rospy.Publisher(
        "/processed/cmd_target_speed", Float32, queue_size=10
    ),
    "CMD_TargetPosition": rospy.Publisher(
        "/processed/cmd_target_position", Float32, queue_size=10
    ),
    "CMD_TargetRelativeCurrent": rospy.Publisher(
        "/processed/cmd_target_relative_current", Float32, queue_size=10
    ),
    "CMD_TargeRelativeBrakeCurrent": rospy.Publisher(
        "/processed/cmd_target_relative_brake_current", Float32, queue_size=10
    ),
    "CMD_MaxAcCurrent": rospy.Publisher(
        "/processed/cmd_max_ac_current", Float32, queue_size=10
    ),
    "CMD_MaxAcBrakeCurrent": rospy.Publisher(
        "/processed/cmd_max_ac_brake_current", Float32, queue_size=10
    ),
    "CMD_MaxDcCurrent": rospy.Publisher(
        "/processed/cmd_max_dc_current", Float32, queue_size=10
    ),
    "CMD_MaxDcBrakeCurrent": rospy.Publisher(
        "/processed/cmd_max_dc_brake_current", Float32, queue_size=10
    ),
    "CMD_DriveEnable": rospy.Publisher(
        "/processed/cmd_drive_enable", Float32, queue_size=10
    ),
}
