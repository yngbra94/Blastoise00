"""
    commands.py

    Defines an Enum for valid commands to be sent over cmd/ and state/ topics

    Using an Enum whose values are equal to the strings we expect is a
    safer method than using the strings themselves. With string alone,
    typos won't result in a error but our program won't work. Using Enums,
    a typo will result in an error which will be easier to fix.

    Created: 2020/02/05
    Author: Brendan Halloran
"""

from enum import Enum

class  Commands(Enum):
    START = 'start'
    STOP = 'stop'


class RobotCommand(Enum):
    EXPLORE = 'explore'
    RETURN = 'return'
    PAUSE = 'pause'

class RobotState(Enum):
    WAITING_TO_START = 'waiting_to_start'
    PAUSED = 'pause'
    EXPLORING = 'exploring'
    RETURNING = 'returning'
    DONE = 'done'