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

    # Class constructor
    def __init__(self, name, player_ID, money=100):
        
        # Set name
        self.name = name
        # Set player ID
        self.ID = player_ID
        # Set budget of player
        self.wallet = money
        # Initialise empty hand
        self.hand = []
        # Initialise value of hand
        self.value = 0
        # Flag to track if player actions are eligible
        self.can_play = True

        # Print message
        print("[INFO]: Player, {}, created.".format(self.name))

    # Add a card to player's hand
    def add_card_to_hand(self, card):

        # Add card to hand
        self.hand.append(card)
        # Update value of hand
        self.update_hand_value()

    # Update the value of cards held
    def update_hand_value(self):

        # Initialise total
        total = 0

        # Iterate through each card in hand
        for card in self.hand:
            
            # Sum value of cards in hand
            total = total + card.value

    # Print cards held in hand
    def print_hand(self):
        
        # Get list of card names
        cards_list = [card.name for card in self.hand]
        # Print list
        print("[{}]: {}; Value:{}".format(self.name, cards_list, self.value))
