"""
MTRX5700 - Experimental Robotics
Major Project - Blackjack Robot
Year: 2021
Group 5 - Curry Shop

File:
Info:
"""

# Imports
from Blackjack_Card import Blackjack_Card


# Class declaration
class Blackjack_Game:

    def __init__(self, num_players=1, num_decks=4):

        # 
        self.num_players = num_players
        # 
        self.num_decks = num_decks

        # 
        self.create_decks()

        # Print message
        print("[INFO]: Game initialised.")

    # 
    def create_decks(self):
        
        # 
        suits = ["Clubs", "Diamonds", "Hearts", "Spades"]
        # 
        numbers = ["A", "2", "3", "4", "5", "6", "7", "8", "9", "10", "J", "Q", "K"]
        # 
        values = {"A":11, "2":2, "3":3, "4":4, "5":5, "6":6, "7":7, "8":8, "9":9, "10":10, "J":10, "Q":10, "K":10}

        # 
        self.deck = []

        # 
        for deck in range(self.num_decks):
            
            # 
            for suit in suits:
                
                # 
                for number in numbers:

                    # 
                    self.deck.append(Blackjack_Card(suit, number, values[number]))

        # Print message
        print("[INFO]: Decks initialised.")
        print("[INFO]:", len(self.deck), " cards created.")

    # 
    # def deal_initial_cards(self)


