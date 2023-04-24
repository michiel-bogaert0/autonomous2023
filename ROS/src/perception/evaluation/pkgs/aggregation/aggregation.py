import inspect
from abc import ABC, abstractmethod
from typing import Any, get_type_hints, List


class Aggregation(ABC):
    """
    Class to include your aggregation functions
    these should be of the form do_{agg_string}
    """

    def __init__(self, name) -> None:
        self.name = name
        self.state = None

    @abstractmethod
    def aggregate_data(self) -> Any:
        """
        function that computes the outcome value, should have the right arguments
        """
        pass

    @abstractmethod
    def get_output_type(self) -> Any:
        """
        Needed for the publisher
        """
        pass

    @abstractmethod
    def plot(self, values: List[Any], savedir: str) -> None:
        """
        Custom plot function
        args:
            values: a list of aggregate values that are kept in the plottingnode
            savedir: the file to save the plot to
        """
        pass

    @abstractmethod
    def update_state(self, input: Any) -> Any:
        """
        Update internal state if needed
        """
        return self.state

    def get_attrs(self, func: str) -> dict:
        """
        Get the argument names of a function
        Args:
            func: name of the aggregate function
        Returns:
            the argument specifications of that function
        """
        # get the "hidden" name of the function inside this class
        do = f"aggregate_data"

        # check if string is valid
        if hasattr(self, do) and callable(func := getattr(self, do)):

            return get_type_hints(getattr(self, do))

    def apply(self, func: str, kwargs: dict) -> Any:
        """
        applies the aggregation function
        Args:
            funct: name of the aggregation function
            kwargs: keyword arguments of the aggregation function with corresponding values
        Returns:
            the return value of the aggregation function
        """

        # get the "hidden" name of the function inside this class
        do = f"aggregate_data"
        # check if string is valid
        if hasattr(self, do) and callable(func := getattr(self, do)):
            value = getattr(self, do)(**kwargs)
            self.update_state(value)
            return value
