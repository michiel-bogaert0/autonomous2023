import rospy
from car_state import CarState, CarStateEnum
from node_fixture import AutonomousStatesEnum


class SimulationState(CarState):

    """
    Example implementation for simulation

    Everything is just OK, no emergencies can be initiated
    The car will auto transition from ASReady to ASDrive after being
    5 seconds in R2D
    """

    def __init__(self) -> None:
        self.state = {
            "TS": CarStateEnum.UNKOWN,
            "ASMS": CarStateEnum.UNKOWN,
            "R2D": CarStateEnum.UNKOWN,
            "ASB": CarStateEnum.UNKOWN,
            "EBS": CarStateEnum.UNKOWN,
        }

        self.start_t = rospy.Time.now().to_sec()

        self.as_state = AutonomousStatesEnum.ASOFF
        self.ebs_state = CarStateEnum.ON

    def update(self, state: AutonomousStatesEnum):
        self.as_state = state

    def activate_EBS(self):
        self.ebs_state = CarStateEnum.ACTIVATED

    def get_state(self):
        """
        Returns:
            object with the (physical) state of the car systems,
            like EBS and ASSI. See general docs for info about this state
        """

        # R2D
        if (
            self.as_state == AutonomousStatesEnum.ASREADY
            and rospy.Time.now().to_sec() - self.start_t > 5.0
        ):
            self.state["R2D"] = CarStateEnum.ACTIVATED
        elif self.as_state != AutonomousStatesEnum.ASDRIVE:
            self.state["R2D"] = CarStateEnum.OFF

        # TS and ASB
        self.state["TS"] = CarStateEnum.ON
        self.state["ASB"] = (
            CarStateEnum.ACTIVATED
            if self.state["R2D"] == CarStateEnum.OFF
            else CarStateEnum.ON
        )

        # ASMS
        self.state["ASMS"] = CarStateEnum.ON

        # EBS
        self.state["EBS"] = self.ebs_state

        return self.state
