from std_msgs.msg import Bool, Float64, Int64

# Dict of messages and their respective ROS message types for conversion
messages = {
    "<class 'float'>": lambda msg: Float64(msg),
    "<class 'int'>": lambda msg: Int64(msg),
    "<class 'bool": lambda msg: Bool(msg),
}

publishers = {
    # Empty by design. Will be filled with publishers on the fly
}
