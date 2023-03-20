import inspect
from abc  import ABC,abstractmethod
from typing import Any,get_type_hints
class Aggregation(ABC):
    """
    Class to include your aggregation functions
    these should be of the form do_{agg_string}
    """
    def __init__(self,name) -> None:
        self.name = name
    @abstractmethod
    def aggregate_data(self):
        pass

    @abstractmethod
    def get_output_type(self):
        pass

    def get_attrs(self,func:str) -> dict:
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
            
            return get_type_hints(getattr(self,do))
        
    def apply(self,func:str,kwargs:dict)-> Any:
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
            return getattr(self,do)(**kwargs)