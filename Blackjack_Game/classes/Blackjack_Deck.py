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
import random

# Class declaration
class Blackjack_Deck:

    # 
    def __init__(self, num_decks):
        
        # 
        # suits = ["Clubs", "Diamonds", "Hearts", "Spades"]
        suits = ["C", "D", "H", "S"]
        # 
        numbers = ["A", "2", "3", "4", "5", "6", "7", "8", "9", "10", "J", "Q", "K"]
        # 
        values = {"A":11, "2":2, "3":3, "4":4, "5":5, "6":6, "7":7, "8":8, "9":9, "10":10, "J":10, "Q":10, "K":10}

        # 
        self.deck = []

        # 
        for _ in range(num_decks):
            
            # 
            for suit in suits:
                
                # 
                for number in numbers:

                    # 
                    self.deck.append(Blackjack_Card(suit, number, values[number]))

        # Print message
        # print("[INFO]: Decks initialised.")
        # print("[INFO]:", len(self.deck), "cards created.")

        # Shuffle deck
        self.shuffle_deck()
        # print("[INFO]: Deck shuffled.")

        # 
    def shuffle_deck(self):

        # 
        random.shuffle(self.deck)