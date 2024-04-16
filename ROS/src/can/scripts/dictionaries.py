from std_msgs.msg import Float64, Int64, String, Bool

messages = {
    "<class 'float'>": lambda msg: Float64(msg),
    "<class 'cantools.database.can.signal.NamedSignalValue'>": lambda msg: String(msg),
    "<class 'int'>": lambda msg: Int64(msg),
    "<class 'bool": lambda msg: Bool(msg),

}


publishers = {
    # Empty by design. Will be filled with publishers on the fly
}
