#!/usr/bin/env python3





import rospy
from geometry_msgs.msg import Vector3,Point,TransformStamped
from std_msgs.msg import UInt32
from std_msgs.msg import Bool
import tf2_ros as tf2

class lap_counter:
    def __init__(self):
        rospy.init_node("lap_counter")
        
        
        self.finished=False
        self.newLap=False
        self.checkFinishRange=True
        self.max_laps=4
        self.range=3
        self.finishPoint=Point(0,0,0)
        self.distanceAfterFinish=4
        car_pose=TransformStamped()
        self.world_frame = rospy.get_param("~world_frame", "ugr/map")
        self.base_link_frame = rospy.get_param("~base_link_frame", "ugr/car_base_link")
        self.tfBuffer=tf2.Buffer()
        self.listener=tf2.TransformListener(self.tfBuffer)
        self.lap_publisher = rospy.Publisher(
            "/lap_counter/laps_complete",
            UInt32,
            queue_size=10
        )
        self.finished_publisher = rospy.Publisher(
            "/lap_counter/finished",
            Bool,
            queue_size=1
        )
        self.laps=0
       
        
        
        rate = rospy.Rate(50) # 10hz
        while not rospy.is_shutdown():
    

            try:
                car_pose=self.tfBuffer.lookup_transform(self.world_frame,self.base_link_frame,rospy.Time(0))
            except (tf2.LookupException,tf2.ConnectivityException,tf2.ExtrapolationException):
                rospy.logerr("error")
            carToFinishVector=Vector3()
            carToFinishVector.x=self.finishPoint.x-car_pose.transform.translation.x
            carToFinishVector.y=self.finishPoint.y-car_pose.transform.translation.y
            carToFinishVector.z=self.finishPoint.z-car_pose.transform.translation.z

            if self.newLap and not self.finished:
            
                if self.checkFinishRange and self.inFinishRange(carToFinishVector): #Car has entered Finishzone
                    
                    self.carEntryVector=carToFinishVector
                    self.checkFinishRange=False
                #In Finish range so now we wait till it passes the FinishPoint
                elif not self.checkFinishRange and self.passedFinish(carToFinishVector):
                    self.laps+=1
                    self.newLap=False
                    self.lap_publisher.publish(UInt32(self.laps))
                elif not self.checkFinishRange and not self.inFinishRange(carToFinishVector):
                    self.checkFinishRange=True #if it goes out of the range again reset

            elif not self.finished and not self.inFinishRange(carToFinishVector): #Wait till it leaves the Finishzone
                self.newLap=True
                self.checkFinishRange=True


            if self.laps>=self.max_laps:
                self.finished=True 
                self.finished_publisher.publish(Bool(self.finished))
            if self.finished and self.DotProduct(carToFinishVector,carToFinishVector)>self.distanceAfterFinish * self.distanceAfterFinish :
                self.lap_publisher.publish(UInt32(24))

            rate.sleep()
    def passedFinish(self,carToFinishVector):
        if self.DotProduct(carToFinishVector,self.carEntryVector)<0:
            return True
        return False

    def inFinishRange(self,carToFinishVector):
        if self.DotProduct(carToFinishVector,carToFinishVector)<self.range**2:
            return True
        return False

    
    def DotProduct(self,Vector1,Vector2):
        return Vector1.x*Vector2.x+Vector1.y*Vector2.y+Vector1.z*Vector2.z


    


node = lap_counter()
rospy.spin()