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
        # 
        self.blackjack_value = 21
        # 
        self.play_flag = True

        # Print message
        print("[INFO]: Game initialised.")
        print("--------------------------------------------------")

    # Deal 2 cards to dealer and each player
    def deal_initial_cards(self):
        
        # Deal 2 cards to dealer
        self.deal_to_person(self.dealer, 2)
        self.dealer.print_first_card()

        # Iterate through each player
        for player in self.player_list:
            # Deal each player 2 cards
            self.deal_to_person(player, 2)
            # player.print_hand()
        

    # Deal to a person (dealer or player)
    def deal_to_person(self, person, num_cards=1):

        # Iterate for number of cards to be dealt
        for _ in range(num_cards):
            # Add card person's hand
            person.add_card_to_hand(self.deck.deck[0])
            # Remove card from deck
            self.deck.deck.pop(0)

    # 
    def check_values(self):

        blackjack_flag = False

        # Iterate through each player
        for player in self.player_list:
            
            # 
            if player.value == self.blackjack_value:
                # 
                player.print_blackjack_msg()
                # 
                blackjack_flag = True

            # 
            elif player.value > self.blackjack_value:
                player.action = "s"        

        # 
        return blackjack_flag

    # 
    def get_player_input(self):

        # 
        for player in self.player_list:
            
            # 
            if player.action != "s":
                # 
                player.take_input()
                print(player.action)

    # 
    def execute_player_action(self):
        # 
        count_play_flag = 0

        # 
        for player in self.player_list:

            # 
            if player.action.lower() == "h":

                # 
                self.deal_to_person(player)
            
            # 
            else:
                # 
                count_play_flag = count_play_flag + 1

        # 
        if count_play_flag == len(self.player_list):
            self.play_flag = False

    # 
    def final_reveal(self):

        # 
        print("-------------------FINAL REVEAL-------------------")
        # 
        self.dealer.print_hand()

        # 
        for player in self.player_list:
            
            #  
            player.print_hand()

    # 
    def play_game(self):
        
        # 
        self.deal_initial_cards()

        # 
        while self.play_flag:

            # 
            continue_flag = self.check_values()

            # 
            if continue_flag:
                # 
                self.play_flag = False
                # 
                break
            else:
                self.get_player_input()
                self.execute_player_action()

        # 
        self.final_reveal()

        # 
        print("[GAME]: Game over.")




