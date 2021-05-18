"""
MTRX5700 - Experimental Robotics
Major Project - Blackjack Robot
Year: 2021
Group 5 - Curry Shop

File:
Info:
"""

# Imports


# Class declaration
class Blackjack_Card:
    
    def __init__(self, suit, number, value, x=0, y=0):

        self.suit = suit
        self.number = number
        self.value = value
        self.name = self.suit + self.number
        self.coords = (x,y)

    def get_card_name(self):

        # 
        string = self.suit + self.number
        
        # 
        return string