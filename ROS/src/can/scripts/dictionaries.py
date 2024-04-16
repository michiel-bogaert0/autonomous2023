from std_msgs.msg import Float32, Int64, String

messages = {
    "<class 'float'>": lambda msg: Float32(msg),
    # "<class 'cantools.database.can.signal.NamedSignalValue'>" : lambda msg: String(msg)
    "<class 'cantools.database.can.signal.NamedSignalValue'>": lambda msg: String(msg),
    "<class 'int'>": lambda msg: Int64(msg),
}

publishers = {
    # Empty by design. Will be filled with publishers on the fly
}
