import inspect
import rospy

class Aggregation:
    def do_plus(self,x=None,y=None):
        return x + y
    def do_divide(self,x,y):
        return x/y
    
    def get_attrs(self,func):
        do = f"do_{func}"
        
        if hasattr(self, do) and callable(func := getattr(self, do)):
            rospy.loginfo(f"found func {do}")
            return inspect.getargspec(getattr(self,do))
        
    def apply(self,func,kwargs):
        do = f"do_{func}"
        
        if hasattr(self, do) and callable(func := getattr(self, do)):
            return getattr(self,do)(**kwargs)