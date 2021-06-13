"""
MTRX5700 - Experimental Robotics
Major Project - Blackjack Robot
Year: 2021
Group 5 - Curry Shop

File:
Info:
.
"""
# Imports
from blackjack_classes.BlackjackCard import BlackjackCard

# Class declaration
class BlackjackDealer:

    # Class constructor
    def __init__(self):
        
        # Initialise empty hand
        self.hand = []
        self.player_id = -1
        # Initialise value of hand
        self.value = 0
        # Flag to track if player actions are eligible
        self.action = True
        # Flag to track if player actions are eligible
        self.bust = False

    def reset(self):
        self.hand = []
        self.value = 0
        self.action = True
        self.bust = False

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
        ace_count = 0
        # Iterate through each card in hand
        for card in self.hand:
            
            # Cumulatively add value of cards in hand
            if card.value == 11:
                ace_count += 1
            total = total + card.value

        # Save value
        self.value = total
        if total > 21:
            for i in range(ace_count):
                self.value -= 10
                if self.value < 21:
                    break
        
        if self.value <= 21 and len(self.hand) >= 6:
            self.value = 21

    # Print cards held in hand
    def print_hand(self):
        
        # Get list of card names
        cards_list = [str(card.value) for card in self.hand]
        # Print list
        print("[{}]: Hand: {}; Value:{}".format(str(self.player_id), cards_list, self.value))

    # Reveal first card
    def print_first_card(self):

        # Print first card
        print("[{}]: Hand: {}".format(str(self.player_id), self.hand[0].name))

    # Print message if dealer gets blackjack
    def print_blackjack_msg(self):

        # Print message
        print("[{}]: Blackjack!".format(str(self.player_id)))
