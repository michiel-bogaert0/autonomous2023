
#! WIP

from locmap_controller.msg import ServiceRequest, ServiceResponse
from node_fixture.node_fixture import ROSNode, AddSubscriber

class ServiceCaller(ROSNode):
    
    def __init__(self) -> None:
        super().__init__('servicecaller')

    @AddSubscriber('/servicecaller/in')
    def handleServiceCall(msg: ServiceRequest):
        
        command = msg.command

        raise "TODO"
        