import inspect
from typing import Any
class Aggregation:
    """
    Class to include your aggregation functions
    these should be of the form do_{agg_string}
    """
    def __init__(self) -> None:
        return self
    
    def do_plus(self,x:float=None,y:float=None)->float:
        """
        Example aggregation function: addition
        Args:
            x
            y
        Returns:
            x+y
        """
        return x + y
    
    def do_divide(self,x:float=None,y:float=None)->float:
        """
        Example aggregation function: division
        Args:
            x
            y
        Returns:
            x/y
        """
        return x/y
    
    def get_attrs(self,func:str)->inspect.Argspec:
        """
        Get the argument names of a function
        Args:
            func: name of the aggregate function
        Returns:
            the argument specifications of that function
        """
        # get the "hidden" name of the function inside this class
        do = f"do_{func}"

        # check if string is valid
        if hasattr(self, do) and callable(func := getattr(self, do)):
            
            return inspect.getargspec(getattr(self,do))
        
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
        do = f"do_{func}"
        # check if string is valid
        if hasattr(self, do) and callable(func := getattr(self, do)):
            return getattr(self,do)(**kwargs)