import rospy

from car_state import carStateEnum, CarState
from node_fixture import AutonomousStatesEnum


class SimulationState(CarState):

    """
    Example implementation for simulation

    Everything is just OK, no emergencies can be initiated
    The car will auto transition from ASReady to ASDrive after being
    5 seconds in R2D
    """

    def __init__(self) -> None:

        self.as_ready_time = rospy.Time.now().to_sec()

        self.state = {
            "TS": carStateEnum.UNKOWN,
            "ASMS": carStateEnum.UNKOWN,
            "R2D": carStateEnum.UNKOWN,
            "ASB": carStateEnum.UNKOWN,
            "EBS": carStateEnum.UNKOWN,
        }

        self.as_state = AutonomousStatesEnum.ASOFF

    def update(self, state: AutonomousStatesEnum):

        # On a state transition, start 5 second timeout
        if (
            state == AutonomousStatesEnum.ASREADY
            and self.as_state != AutonomousStatesEnum.ASREADY
        ):
            self.as_ready_time = rospy.Time.now().to_sec()

        self.as_state = state

    def get_state(self):
        """
        Returns:
            object with the (physical) state of the car systems,
            like EBS and ASSI. See general docs for info about this state
        """

        t = rospy.Time.now().to_sec()

        # R2D
        if t - self.as_ready_time > 5.0:
            self.state["R2D"] = carStateEnum.ACTIVATED
        elif self.as_state != AutonomousStatesEnum.ASDRIVE:
            self.state["R2D"] = carStateEnum.OFF

        # TS and ASB
        self.state["TS"] = carStateEnum.ON
        self.state["ASB"] = (
            carStateEnum.ACTIVATED
            if self.state["R2D"] == carStateEnum.OFF
            else carStateEnum.ON
        )

        # ASMS
        self.as_state["ASMS"] = carStateEnum.ON

        # EBS
        self.state["EBS"] = carStateEnum.ON

        return self.state
