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
class BlackjackPlayer:

    # Class constructor
    def __init__(self, player_id, wallet, table_coordinate=()):
        
        # Set player ID
        self.player_id = player_id
        # Set budget of player
        self.wallet = wallet
        # Initialise empty hand
        self.hand = []
        # Initialise value of hand
        self.value = 0
        # Flag to track if player actions are eligible
        self.bust = False
        # Variable to track player action
        self.action = None
        # 
        self.winner = False

        #
        self.bet = 0

        self.table_coordinate = table_coordinate


        # Print message
        # print("[INFO]: Player, {}, created.\n".format(self.name))

    def reset(self):
        self.hand = []
        self.value = 0
        self.bust = False
        self.bet = 0
        self.action = None

    # def get_bet(self):
    #     while(True):
    #         bet_size = int(input(str(self.player_id) + " how much would you like to bet, you have ${} remaing: ".format(self.wallet)))
    #         if bet_size > self.wallet or bet_size <= 0:
    #             print("Invalid Bet, please bet a different amount")
    #         else:
    #             break
    #     self.wallet -= bet_size
    #     self.bet = bet_size
    # Add a card to player's hand
    
    def add_card_to_hand(self, card):

        # Add card to hand
        self.hand.append(card)
        # Update value of hand
        # self.update_hand_value()
        self.print_hand()

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

            if card.value is None:
                card.value = 0
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
        
        if self.value == 21:
            print("Black Jack!")
        

    # # Take player input
    # def take_input(self):
        
    #     # Receive input
    #     self.action = input("[GAME]: {}, would you like to HIT (h) or STAY (s)? ".format(self.name))


    # Print cards held in hand
    def print_hand(self):
        
        # Get list of card names
        cards_list = [str(card.value) for card in self.hand]
        # Print list
        print("[{}]: Hand: {}; Value:{}".format(self.player_id, cards_list, self.value))

    # Print message if blackjack is achieved
    def print_blackjack_msg(self):

        # Print message
        print("[{}]: Blackjack!".format(self.player_id))

    # Print message if player wins
    def print_win_msg(self):
        
        # Set win flag
        self.winner = True 
        # Print message
        print("[{}]: Wins!".format(self.player_id))

    # Print message if player loses
    def print_lose_msg(self):
        
        # Clear win flag
        self.winner = False
        # Print message
        print("[{}]: Loses!".format(self.player_id))

    