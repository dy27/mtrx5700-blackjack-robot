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
        # Set value for blackjack
        self.blackjack_value = 21
        # Set flag to continue playing
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

    # Deal to a person (dealer or player)
    def deal_to_person(self, person, num_cards=1):

        # Iterate for number of cards to be dealt
        for _ in range(num_cards):
            # Add card person's hand
            person.add_card_to_hand(self.deck.deck[0])
            # Remove card from deck
            self.deck.deck.pop(0)

    # Check hand of all players
    def check_values(self):

        # Set flag at start
        continue_flag = True
        # Initialise counter
        count = 0

        # Iterate through each player
        for player in self.player_list:
            
            # If player has 21 or higher
            if player.value >= self.blackjack_value:
                # Force action as "stay"
                player.action = "s"
                # Set bust flag
                player.bust = True
            
            # Check if player action is "s"
            if player.action == "s":
                # Increment count
                count = count + 1

        # Check if everyone's action is "s"
        if count == len(self.player_list):
            # Clear flag
            continue_flag = False

        # Return flag
        return continue_flag

    # Receive player input
    def get_player_input(self):

        # Iterate through each player
        for player in self.player_list:
            
            # Ensure that player is not asked to stay
            if player.action != "s":
                
                # Take input from player
                player.take_input()
                # Print player action
                print(player.action)

    # Execute player action
    def execute_player_action(self):

        # Iterate through each player
        for player in self.player_list:

            # Check if player asked to hit
            if player.action.lower() == "h":

                # Deal a card to player
                self.deal_to_person(player)

    # Reveal dealer's cards
    def final_reveal(self):

        # Print message
        print("-------------------FINAL REVEAL-------------------")
        
        # Reveal delear's hand
        self.dealer.print_hand()

        # Iterate through every player
        for player in self.player_list:
            
            # Print player hand
            player.print_hand()

        # Determine winner
        self.choose_winner()

        

    # Check hand values and determine game winner
    def choose_winner(self):

        # Iterate through each player
        for player in self.player_list:
            
            # Check if dealer has busted
            if self.dealer.value >= self.blackjack_value:
                # 
                print("Dealer has busted.")

                # Check if player has busted
                if player.bust:
                    print("[{}]: has busted.".format(player.name))

                else:
                    # Player wins
                    player.print_win_msg()

            else:
                # Check if player has not busted and beats the dealer's value
                if (not player.bust) and (player.value >= self.dealer.value):

                    # Player wins
                    player.print_win_msg()

                else:
                    # Player loses
                    player.print_lose_msg()

    # Main loop to play game
    def play_game(self):
        
        # Deal initial cards
        self.deal_initial_cards()

        # While play can continue
        while self.play_flag:

            # Check players's hand values
            continue_flag = self.check_values()

            # If game can continue
            if continue_flag:
                
                # Get player input
                self.get_player_input()
                # Execute player action
                self.execute_player_action()

            else:
                # Clear flag
                self.play_flag = False
                # Break from loop
                break

        # Reveal dealer cards and determine winner
        self.final_reveal()

        # Print message
        print("[GAME]: Game over.")




