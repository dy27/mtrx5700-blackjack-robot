"""
MTRX5700 - Experimental Robotics
Major Project - Blackjack Robot
Year: 2021
Group 5 - Curry Shop

File:
Info:
"""

# Imports
from classes.Blackjack_Card import Blackjack_Card

# Class declaration
class Blackjack_Player:

    # 
    def __init__(self, name, player_ID, money=100):
        
        # 
        self.name = name
        # 
        self.ID = player_ID
        # 
        self.wallet = money
        # 
        self.hand = []
        # 
        self.value = 0
        # 
        self.can_play = True

        # Print message
        print("[INFO]: Player", self.name, "created.")

    # 
    def add_card_to_hand(self, card):

        # 
        self.hand.append(card)
        # 
        self.update_hand_value()

    #
    def update_hand_value(self):

        # 
        total = 0

        # 
        for card in self.hand:
            
            # 
            total = total + card.value

    # 
    def print_hand(self):

        print("[{}]:".format(self.name), "{}|{}".format(self.hand[0].get_card_name(), self.hand[1].get_card_name()))
