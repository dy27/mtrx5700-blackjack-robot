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
    
    def __init__(self, suit, number, value):

        self.suit = suit
        self.number = number
        self.value = value

    def get_card_name(self):

        # 
        string = self.suit + self.number
        
        # 
        return string