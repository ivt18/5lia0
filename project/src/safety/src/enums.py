#!/usr/env/bin python2

from enum import Enum


class StopGo(Enum):
    STOP = False 
    GO = True


class StopSign(Enum):
    GO = 0
    STOP = 1
    LEAVING = 2

