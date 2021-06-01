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
class Blackjack_Dealer:

    # Class constructor
    def __init__(self, name="Dealer"):
        
        # Set name
        self.name = name
        # Initialise empty hand
        self.hand = []
        # Initialise value of hand
        self.value = 0
        # Flag to track if player actions are eligible
        self.action = True
        # Flag to track if player actions are eligible
        self.bust = False

        # Print message
        print("[INFO]: Dealer, {}, created.".format(self.name))

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
            
            # Cumulatively add value of cards in hand
            total = total + card.value

        # Save value
        self.value = total

    # Print cards held in hand
    def print_hand(self):
        
        # Get list of card names
        cards_list = [card.name for card in self.hand]
        # Print list
        print("[{}]: Hand: {}; Value:{}".format(self.name, cards_list, self.value))

    # Reveal first card
    def print_first_card(self):

        # Print first card
        print("[{}]: Hand: {}".format(self.name, self.hand[0].name))

    # Print message if dealer gets blackjack
    def print_blackjack_msg(self):

        # Print message
        print("[{}]: Blackjack!".format(self.name))
