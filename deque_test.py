#! /usr/bin/env python3

from collections import deque

class SchedulerConflict(Exception):
    """Raised upon trying to schedule a function that
    is already in the MovementScheduler"""
    def __init__(self, value):
        self.value = value
    def __str__(self):
        return repr(self.value)

class QueueItemError(Exception):
    """Raised if an item to be schedule with the MovementScheduler
    has the wrong format"""
    def __init__(self, value):
        self.value = value
    def __str__(self):
        return repr(self.value) 


class MovementQueue(deque):
    """A queue for scheduling robot movements.
    It guarantees that each function is scheduled only once at a time
    to prevent conflicts with the potential of deadlocking the bot"""

    def append(self, newitem):
        # check proper format of item first
        if not type(newitem) == list:
            raise QueueItemError("MovementQueue items must be lists.")
        elif not len(newitem) == 2:
            raise QueueItemError("MovementQueue items must be of format: [function, [args]].")
        elif not callable(newitem[0]):
            raise QueueItemError("First item in list must be a function.")
        elif not type(newitem[1]) == list:
            raise QueueItemError("MovementQueue items must be of format: [function, [args]].")

        for item in self:
            if item[0] == newitem[0]:
                raise SchedulerConflict('The function "{}" is already in the queue.'.format(item[0]))

        super().append(newitem)
