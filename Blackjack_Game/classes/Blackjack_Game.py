"""
MTRX5700 - Experimental Robotics
Major Project - Blackjack Robot
Year: 2021
Group 5 - Curry Shop

File:
Info:
"""

# Imports
from classes.Blackjack_Deck import Blackjack_Deck
from classes.Blackjack_Player import Blackjack_Player
from classes.Blackjack_Dealer import Blackjack_Dealer

# Class declaration
class Blackjack_Game:

    def __init__(self, dealer, player_list, deck):

        # 
        self.dealer = dealer
        # 
        self.player_list = player_list
        # 
        self.deck = deck
        
        # 
        self.num_players = len(self.player_list)

        # Print message
        print("[INFO]: Game initialised.")

    # 
    def deal_initial_cards(self):
        
        # Deal 1st card to dealer
        self.dealer.add_card_to_hand(self.deck.deck[0])
        self.deck.deck.pop(0)
        # Deal 1st card to player
        self.player_list[0].add_card_to_hand(self.deck.deck[0])
        self.deck.deck.pop(0)
        
        # Deal 2nd card to dealer
        self.dealer.add_card_to_hand(self.deck.deck[0])
        self.deck.deck.pop(0)
        # Deal 2nd card  to player
        self.player_list[0].add_card_to_hand(self.deck.deck[0])
        self.deck.deck.pop(0)

        self.player_list[0].print_hand()
        self.dealer.print_hand()
        
        print("{} cards in deck".format(len(self.deck.deck)))

        pass


