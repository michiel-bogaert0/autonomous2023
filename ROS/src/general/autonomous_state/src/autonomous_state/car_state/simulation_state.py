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

        self.state = {
            "TS": carStateEnum.UNKOWN,
            "ASMS": carStateEnum.UNKOWN,
            "R2D": carStateEnum.UNKOWN,
            "ASB": carStateEnum.UNKOWN,
            "EBS": carStateEnum.UNKOWN,
        }

        self.as_state = AutonomousStatesEnum.ASOFF
        self.ebs_state = carStateEnum.ON

    def update(self, state: AutonomousStatesEnum):
        self.as_state = state

    def activate_EBS(self):
        self.ebs_state = carStateEnum.ACTIVATED

    def get_state(self):
        """
        Returns:
            object with the (physical) state of the car systems,
            like EBS and ASSI. See general docs for info about this state
        """

        # R2D
        if self.as_state == AutonomousStatesEnum.ASREADY:
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
        self.state["ASMS"] = carStateEnum.ON

        # EBS
        self.state["EBS"] = self.ebs_state

        return self.state
