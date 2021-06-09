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
from blackjack_classes.BlackjackDeck import BlackjackDeck
from blackjack_classes.BlackjackGame import BlackjackGame
from blackjack_classes.BlackjackPlayer import BlackjackPlayer
from blackjack_classes.BlackjackDealer import BlackjackDealer

if __name__ == "__main__":

    # Create a deck
    deck = BlackjackDeck(1)
    
    # Create game
    game = BlackjackGame(max_players=3, deck=deck)
    game.update_players()

    # Play the game
    game.play_game()

