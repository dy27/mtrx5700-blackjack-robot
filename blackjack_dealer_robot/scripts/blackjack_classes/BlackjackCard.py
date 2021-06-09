"""
MTRX5700 - Experimental Robotics
Major Project - Blackjack Robot
Year: 2021
Group 5 - Curry Shop

File:
Info:
.
.
"""

class BlackjackCard:
    def __init__(self, value, x=0, y=0, detected=False):
        self.detected = detected
        self.value = value
        self.coords = [x,y]
        
