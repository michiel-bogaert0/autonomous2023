import time
from inspect import signature


class Timer(object):
    def __init__(self, description=None, verbose=True):
        self.time = None
        self.verbose = verbose
        if description is None:
            self.description = "execution took"
        else:
            self.description = description

    def __enter__(self):
        self.start = time.perf_counter()

    def __exit__(self, type, value, traceback):
        self.end = time.perf_counter()
        self.time = self.end - self.start
        if self.verbose:
            print(f"{self.description}: {self.time} s")


def get_nr_args(fun) -> int:
    fun_args = signature(fun).parameters
    return len(fun_args)


if __name__ == "__main__":
    with Timer("List Comprehension Example"):
        s = [x for x in range(10_000_000)]
