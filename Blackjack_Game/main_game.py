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

    # 
    Dealer = Blackjack_Dealer()
    
    # 
    Player1 = Blackjack_Player("Bob", 1)
    # 
    player_list = [Player1]

    # 
    Deck = Blackjack_Deck(1)
    
    # 
    Game = Blackjack_Game(Dealer, player_list, Deck)

    # 
    Game.play_game()