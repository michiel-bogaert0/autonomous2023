from aggregation.aggregation import Aggregation
from std_msgs.msg import Float32

class plus_aggr(Aggregation):
    def __init__(self):
        super().__init__("plus")

    def aggregate_data(self,x: Float32,y:Float32):
        return Float32(data=x.data+y.data)
    
    def get_output_type(self):
        return Float32
        