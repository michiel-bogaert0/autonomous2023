import enum
from abc import ABC, abstractmethod
from itertools import product

import rospy
import yaml
from param import Param


class TunerModes(str, enum.Enum):
    NONE = "none"
    INCREMENT = "increment"
    LIST = "list"
    MULTI = "multi"


def getTuner(config_file: str):
    """
    Returns a tuner object based on the mode specified in the configuration file.

    Args:
        config_file (str): The path to the configuration file.

    Returns:
        Tuner: An instance of the appropriate tuner based on the mode specified in the configuration file.
    """
    with open(config_file, "r") as f:
        data = yaml.safe_load(f)
    mode = data["mode"]

    if mode == TunerModes.INCREMENT:
        return IncrementTuner(data["param"], data[mode])
    elif mode == TunerModes.LIST:
        return ListTuner(data["param"], data[mode])
    elif mode == TunerModes.MULTI:
        return MultiTuner(data["param"], data[mode])
    return DefaultTuner(data["param"])


class Tuner(ABC):
    """
    Abstract base class for parameter tuners.
    """

    def __init__(self) -> None:
        pass

    @abstractmethod
    def change(self):
        pass

    @abstractmethod
    def simulation_finished(self):
        pass

    @abstractmethod
    def reset(self):
        pass


class SingleTuner(Tuner):
    """
    A class representing a single tuner.
    """

    def __init__(self, param_config) -> None:
        self.param = Param(param_config)

    def change(self):
        return [self.param.set_parameter(self.change_tuner())]

    @abstractmethod
    def change_tuner(self):
        return self.param.previous

    @abstractmethod
    def simulation_finished(self):
        pass

    @abstractmethod
    def reset(self):
        self.param.reset()


class MultiTuner(Tuner):
    """
    A class that represents a multi-parameter tuner.

    Attributes:
        params (list): A list of Param objects representing the parameters to be tuned.
        values (list): A list of lists containing the possible values for each parameter.
        combinations (list): A list of tuples representing all possible combinations of parameter values.
        counter (int): A counter to keep track of the current combination being used.

    Methods:
        __init__(self, p, multi_config): Initializes the MultiTuner object.
        change(self): Changes the parameter values to the next combination.
        simulation_finished(self): Checks if all combinations have been used.
        reset(self): Resets the tuner to the initial state.
    """

    def __init__(self, p, multi_config) -> None:
        Tuner.__init__(self)
        self.params = []
        self.values = []

        for param_config in multi_config:
            self.params.append(Param(param_config["param"]))
            self.values.append(param_config["values"])

        self.combinations = list(product(*self.values))
        rospy.loginfo(f"Combinations: {self.combinations}")
        self.counter = 0

    def change(self):
        changes = []
        for i, param in enumerate(self.params):
            changes.append(param.set_parameter(self.combinations[self.counter][i]))
        self.counter += 1

        return changes

    def simulation_finished(self):
        return self.counter >= len(self.combinations)

    def reset(self):
        for param in self.params:
            param.reset()
        self.counter = 0


class IncrementTuner(SingleTuner):
    def __init__(self, param_config, increment_config) -> None:
        """
        Initializes a IncrementTuner object.

        Args:
            param_config (dict): Configuration parameters for the SingleTuner base class.
            increment_config (dict): Configuration parameters for the IncrementTuner.

        Attributes:
            incrementer (int): The value to be added to the parameter.
            simulation_count (int): The number of simulations to be performed.
            counter (int): The counter to keep track of the number of simulations performed.
        """
        SingleTuner.__init__(self, param_config)
        self.incrementer = increment_config["add_value"]
        self.simulation_count = increment_config["simulation_count"]
        self.counter = 0

    def change_tuner(self):
        """
        Changes the tuner by adding the sum value to the parameter.

        Returns:
            int: The updated parameter value.
        """
        self.counter += 1
        return self.param.get_parameter() + self.incrementer

    def simulation_finished(self):
        """
        Checks if the number of simulations performed has reached the simulation count.

        Returns:
            bool: True if the number of simulations is equal to or greater than the simulation count, False otherwise.
        """
        return self.counter >= self.simulation_count

    def reset(self):
        """
        Resets the tuner by calling the reset method of the base class and resetting the counter.
        """
        super().reset()
        self.counter = 0


class ListTuner(SingleTuner):
    """
    A tuner that iterates through a list of values for a parameter.

    Args:
        param_config (dict): Configuration for the parameter being tuned.
        list_config (dict): Configuration for the list tuner.

    Attributes:
        values (list): List of values for the parameter.
        counter (int): Counter to keep track of the current index in the list.

    Methods:
        change_tuner: Returns the next value from the list.
        simulation_finished: Checks if all values in the list have been iterated.
        reset: Resets the tuner to its initial state.
    """

    def __init__(self, param_config, list_config) -> None:
        SingleTuner.__init__(self, param_config)
        self.values = list_config["values"]
        self.counter = 0

    def change_tuner(self):
        param = self.values[self.counter]
        self.counter += 1
        return param

    def simulation_finished(self):
        return self.counter >= len(self.values)

    def reset(self):
        super().reset()
        self.counter = 0


class DefaultTuner(SingleTuner):
    def __init__(self, param_config) -> None:
        SingleTuner.__init__(self, param_config)

    def change_tuner(self):
        return self.param.previous

    def simulation_finished(self):
        return True

    def reset(self):
        super().reset()
