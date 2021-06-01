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
from classes.Blackjack_Game import Blackjack_Game
from classes.Blackjack_Player import Blackjack_Player
from classes.Blackjack_Dealer import Blackjack_Dealer

if __name__ == "__main__":

    # Create a dealer
    Dealer = Blackjack_Dealer()
    
    # Create a player
    Player1 = Blackjack_Player("Bob", 1)
    # Append to player list
    player_list = [Player1]

    # Create a deck
    Deck = Blackjack_Deck(1)
    
    # Create game
    Game = Blackjack_Game(Dealer, player_list, Deck)

    # Play the game
    Game.play_game()