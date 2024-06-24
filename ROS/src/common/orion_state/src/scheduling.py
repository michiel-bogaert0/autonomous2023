#!/usr/bin/env python3
import heapq
import time
from typing import Callable, Optional, Tuple

import rospy


class Job:
    def __init__(
        self,
        exec_time: float,
        func: Callable,
        args: Tuple = (),
        tag: Optional[str] = None,
    ):
        self.exec_time = exec_time
        self.func = func
        self.args = args
        self.tag = tag

    def __lt__(self, other):
        return self.exec_time < other.exec_time


class JobScheduler:
    def __init__(self):
        self.jobs = []
        self.tags = set()

    def add_job(
        self,
        exec_time: float,
        func: Callable,
        args: Tuple = (),
        tag: Optional[str] = None,
    ):
        if tag and tag in self.tags:
            print(f"Job with tag '{tag}' is already scheduled.")
            return

        job = Job(exec_time, func, args, tag)
        heapq.heappush(self.jobs, job)

        if tag:
            self.tags.add(tag)

    def add_job_relative(
        self, delay: float, func: Callable, args: Tuple = (), tag: Optional[str] = None
    ):
        self.add_job(rospy.Time.now().to_sec() + delay, func, args, tag)

    def tag_exists(self, tag: str) -> bool:
        return tag in self.tags

    def remove_job_by_tag(self, tag: str):
        self.jobs = [job for job in self.jobs if job.tag != tag]
        self.tags.remove(tag)

    def step(self):
        current_time = rospy.Time.now().to_sec()
        while self.jobs and self.jobs[0].exec_time <= current_time:
            job = heapq.heappop(self.jobs)
            job.func(*job.args)
            if job.tag:
                self.tags.remove(job.tag)


# Example usage
if __name__ == "__main__":

    def example_task(arg1, arg2):
        print(f"Task executed with arguments: {arg1}, {arg2}")

    scheduler = JobScheduler()

    # Schedule a job to run 5 seconds from now with a tag 'task1'
    scheduler.add_job(
        rospy.Time.now().to_sec() + 5,
        example_task,
        args=("hello", "world"),
        tag="task1",
    )

    # Attempt to schedule another job with the same tag 'task1' (won't be added)
    scheduler.add_job(
        rospy.Time.now().to_sec() + 10,
        example_task,
        args=("hello again", "world"),
        tag="task1",
    )

    # Schedule another job with a different tag 'task2'
    scheduler.add_job(
        rospy.Time.now().to_sec() + 10,
        example_task,
        args=("goodbye", "world"),
        tag="task2",
    )

    # Main loop
    t = 0
    try:
        while True:
            scheduler.step()
            time.sleep(0.1)  # Sleep for a short period to prevent busy-waiting
            t += 1

            print("t: ", t * 0.1)

            if t == 60:
                scheduler.add_job(
                    rospy.Time.now().to_sec() + 5,
                    example_task,
                    args=("hello", "world"),
                    tag="task1",
                )

    except KeyboardInterrupt:
        print("Scheduler stopped.")
