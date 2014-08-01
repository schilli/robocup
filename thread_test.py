#! /usr/bin/env python3

import threading
import time

class Counter(object):

    def __init__(self, n):
        self.n = n
        self.i = 0

    def count(self):
        self.i = 0
        while self.i < self.n:
            self.i += 1
            time.sleep(1)

    def get_i(self):
        return self.i


