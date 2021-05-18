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

    # Class constructor
    def __init__(self, dealer, player_list, deck):

        # Set dealer of game
        self.dealer = dealer
        # Set players in game
        self.player_list = player_list
        # Set deck
        self.deck = deck
        # Get number of players
        self.num_players = len(self.player_list)

        # Print message
        print("[INFO]: Game initialised.")

    # Deal 2 cards to dealer and each player
    def deal_initial_cards(self):
        
        # Deal 2 cards to dealer
        self.deal_to_person(self.dealer, 2)

        # Iterate through each player
        for player in self.player_list:
            # Deal each player 2 cards
            self.deal_to_person(player, 2)

        # Print hands
        self.player_list[0].print_hand()
        self.dealer.print_hand()
        # Print deck size
        self.deck.print_deck_size()

    # Deal to a person (dealer or player)
    def deal_to_person(self, person, num_cards=1):

        # Iterate for number of cards to be dealt
        for _ in range(num_cards):
            # Add card person's hand
            person.add_card_to_hand(self.deck.deck[0])
            # Remove card from deck
            self.deck.deck.pop(0)


