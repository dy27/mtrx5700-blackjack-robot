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
from actionlib import action_client
import rospy
import blackjack_dealer_robot.msg
import geometry_msgs.msg
from blackjack_classes.BlackjackDeck import BlackjackDeck
from blackjack_classes.BlackjackPlayer import BlackjackPlayer
from blackjack_classes.BlackjackDealer import BlackjackDealer


class GameStatus:
    WAITING_FOR_PLAYERS = 0
    WAITING_FOR_BETS = 1
    DEALING = 2
    WAITING_FOR_ACTION = 3
    WAITING_FOR_CARD_DETECTIONS = 4


class PlayerActions:
    JOIN_GAME = 0
    LEAVE_GAME = 1
    BET = 2
    HIT = 3
    STAND = 4


class ActionType:
    DEAL = 0
    CLEAR = 1
    FLIP = 2
    MOVE = 3


class Timer:
    def __init__(self, time_secs):
        self.start_time = None
        self.timer_time = time_secs

    def start(self):
        self.start_time = rospy.get_time()

    def is_finished(self):
        if rospy.get_time() >= self.start_time + self.timer_time:
            return True
        else:
            return False


# Class declaration
class BlackjackGame:

    def fake_card_update_callback(self):
        if self.game_status == GameStatus.WAITING_FOR_CARD_DETECTIONS:
            for player in self.player_list:
                if player is not None:
                    for card in player.hand:
                        if not card.detected:
                            card.detected = True
                            card.value = 2
                            player.update_hand_value()

        for card in self.dealer.hand:
            if not card.detected:
                card.detected = True
                card.value = 2
                self.dealer.update_hand_value()

    def fake_add_players(self):
        pass

    def fake_set_bets(self):
        for player in self.player_list:
            player.bet = 10

    # Class constructor
    def __init__(self, max_players, action_client=None, game_status_pub=None):

        self.max_players = max_players

        self.game_status = GameStatus.WAITING_FOR_PLAYERS

        # Set dealer of game
        self.dealer = BlackjackDealer()

        # Initialise number of seats for the game
        self.player_list = [None] * max_players

        # Set deck
        self.deck = BlackjackDeck(100)

        # Set value for blackjack
        self.blackjack_value = 21

        # Set flag to continue playing
        self.play_flag = True

        self.action_client = action_client
        self.game_status_pub = game_status_pub

        self.bet_timer = Timer(10)
        self.action_timer = Timer(10)

        self.card_shape = (0.064, 0.089)
        self.card_spacing = (0.015, 0.015)

        self.first_coord = (0.07, 0.07)
        self.board_size = (0.9, 0.6)

        self.player_coordinates = self.generate_player_coordinates()

        # max_players * 6 grid of target locations
        self.player_card_coordinates  = self.generate_card_locations()

        self.dealer_card_coordinates = [(0.7,0.4), (0.532,0.4), (0.522,0.4), (0.443,0.4), (0.364,0.4), (0.285,0.4)]

        # Print message
        print("[INFO]: Game initialised.")
        print("--------------------------------------------------\n")



    def update_card_values(self):
        """Go through recognised cards then update the hand values of players"""

        for i in range(len(photo_coords)):
            closestIndices = [0,0]
            closestDistance = 1
            card = photo_coords[i]

            for j in range(len(self.player_list)):
                if self.player_list[j] != None:
                    player_cards = self.player_card_coordinates[j]

                    for k in range(len(self.player_list[j])):
                        player_card = player_cards[k]

                        if self.player_list[j][k].detected == True:
                            continue    

                        if ((player_card[0] - card[0])**2 + (player_card[1] - card[1])**2)**0.5 < closestDistance:
                            closestDistance = ((player_card[0] - card[0])**2 + (player_card[1] - card[1])**2)**0.5
                            closestIndices = [j,k]
                        
            self.player_list[j].hand[k].value = recognised_cards[i]
            self.player_list[j].hand[k].coords = [card[0], card[1]]
            self.player_list[j].hand[k].detected = True
        
        for player in self.player_list:
            if player != None:
                player.update_hand_value()



    # Generate the locations of the player rect top left points
    def generate_player_coordinates(self):

        player_coords = [self.first_coord]

        width = self.board_size[0] + self.card_shape[0]/2 - (2 * self.first_coord[0])

        player_width  = (3*self.card_shape[0] + 2*self.card_spacing[0])
        player_spacing = (width - 3 * player_width)/2
        print(player_width)
        print(player_spacing)

        for i in range(1, self.max_players):
            x_displacement = i * (player_spacing + player_width)
            player_coords.append((self.first_coord[0] + x_displacement, self.first_coord[1]))

        return player_coords


    def generate_card_locations(self):
        
        card_locs = []
        
        for i in range(self.max_players):
            card_locs.append([[0,0],[0,0],[0,0],[0,0],[0,0],[0,0]])
            player_pos = self.player_coordinates[i]

            for j in range(6):
                    card_locs[i][j] = [player_pos[0] + (j % 3) * (self.card_shape[0] + self.card_spacing[0]),
                    player_pos[1] + (j // 3) * (self.card_shape[1] + self.card_spacing[1])]
        

    # Deal first cards of player, then dealer first cards face up, then players' second cards, then dealers 2nd face down 
    def deal_initial_cards(self):
        
        # Iterate through each player
        for _ in range(2):
            for player in self.player_list:
                if player != None:
                    self.deal_to_person(player)   
                    print()
            
            self.deal_to_person(self.dealer)
        
        # DEAL UNFLIPPED CARD TO DEALER
        # CARD GETS FLIPPED IN FINISH DEALER HAND FUNCTION


    # Deal to a person (dealer or player)
    def deal_to_person(self, person):

        flip = True

        # Location for robot to distribute card to 
        if person.id == -1:
            card_target_location = self.dealer_card_coordinates[len(person.hand)]

            # Dealer's second card should not be flipped
            if len(person.hand) == 1:
                flip = False
        else:
            card_target_location = self.player_card_coordinates[person.player_id][len(person.hand)]

        if self.action_client is not None:
            goal = blackjack_dealer_robot.msg.DealerGoal()
            goal.type = 0
            point_msg = geometry_msgs.msg.Point()
            point_msg.x, point_msg.y, point_msg.z = card_target_location[0], card_target_location[1], 0.0
            goal.use_runway_list = [True]
            goal.flip_card_list = [True]
            self.action_client.send_goal(goal)
            self.action_client.wait_for_result()

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
        countActive = 0
        # Iterate through each player
        for player in self.player_list:
            
            if player != None:

                countActive += 1

                # If player has 21 or higher
                if player.value > self.blackjack_value:
                    # Force action as "stay"
                    player.action = "s"
                    # Set bust flag
                    player.bust = True

                elif player.value == self.blackjack_value:
                    player.action = "s"
                
                # Check if player action is "s"
                if player.action == "s":
                    # Increment count
                    count = count + 1

        # Check if everyone's action is "s"
        if count == countActive:
            # Clear flag
            continue_flag = False

        # Return flag
        return continue_flag


    # Receive player input
    def get_player_input(self):

        self.game_status = GameStatus.WAITING_FOR_ACTION

        # Iterate through each player
        for player in self.player_list:
            
            if player != None:
                # Ensure that player is not asked to stay
                if player.action != "s":
                    
                    while player.action is None:
                        rospy.sleep(0.5)

                    # # Take input from player
                    # player.take_input()
                    # # Print player action
                    # print(player.action)

                    # Execute player action
                    self.execute_player_action(player)

                    while not self.all_cards_detected():
                        rospy.sleep(0.5)
                        self.fake_card_update_callback()


    # Execute player action
    def execute_player_action(self, player):

        # Check if player asked to hit
        if player.action == "h":
            # Deal a card to player
            self.deal_to_person(player)
            print()
        else:
            player.print_hand()
            print()
 

    # Reveal dealer's cards
    def final_reveal(self):

        # Print message
        print("\n-------------------FINAL REVEAL-------------------\n")
        
        # Reveal delear's hand
        self.dealer.print_hand()

        # Iterate through every player
        for player in self.player_list:
            
            if player != None:
                # Print player hand
                player.print_hand()

        print()

        # Determine winner
        self.choose_winner()


    def finish_dealer_hand(self):
        # FLIP UNFLIPPED CARD AND UPDATE VALUES
        goal = blackjack_dealer_robot.msg.DealerGoal()
        goal.type = 2
        points = [[0.46, 0.4, 0]]
        self.action_client.send_goal(goal)
        self.action_client.wait_for_result()
        result = self.action_client.get_result()
        print("RESULT RECEIVED", result)


        while self.dealer.value < 17:
            self.deal_to_person(self.dealer)
            while not self.all_cards_detected():
                rospy.sleep(0.5)
                self.fake_card_update_callback()
            # CALL UPDATE CARD VALUES AFTER EACH CARD IS DEALT


    # Check hand values and determine game winner
    def choose_winner(self):

        # Iterate through each player   
        for player in self.player_list:
            
            if player != None:
                if player.bust:
                    print("{}, you have gone bust. You lose your bet of ${}.".format(player.id, player.bet))
                    player.bet = 0

                elif player.value > self.dealer.value or self.dealer.value > self.blackjack_value:
                    if player.value == 21:
                        print("Congrats {}, you got blackjack. You win a 3:2 payout of ${}.".format(player.id, 1.5 * player.bet))
                        player.wallet += player.bet * 2.5 
                    else:
                        print("Congrats {}, you beat the dealer. You win a 1:1 payout of ${}.".format(player.id, player.bet))
                        player.wallet += player.bet * 2
                    player.bet = 0

                elif player.value < self.dealer.value:
                    print("Unlucky {}, the dealer beat you. You lose your bet of ${}.".format(player.id, player.bet))
                    player.bet = 0

                else:
                    print("{}, you tied with the dealer. You get your bet back.".format(player.id))
                    player.wallet += player.bet
                    player.bet = 0
                    
                print("You now have a bankroll of ${}.\n".format(player.wallet))
                if player.wallet == 0:
                    print(player.id + ", you no longer have any money left in your bankroll, you will be removed from the table\n")
                    
        new_player_list = [x for x in self.player_list if (x != None and x.wallet > 0)]
        self.player_list = new_player_list


    def add_player(self, player_index, player_balance):
        if self.game_status == GameStatus.WAITING_FOR_PLAYERS or self.game_status == GameStatus.WAITING_FOR_BETS:

            if self.player_list[player_index] is not None:
                return False
            
            self.player_list[player_index] = BlackjackPlayer(player_index, player_balance, table_coordinate=self.player_coordinates[player_index])
            return True

        else:
            return False


    def get_player_count(self):
        count = 0
        for player in self.player_list:
            if player is not None:
                count += 1
        return count


    def wait_for_bets(self):
        self.game_status = GameStatus.WAITING_FOR_BETS
        # self.bet_timer.start()
        # while not self.bet_timer.is_finished() and not self.all_bets_placed():
        #     rospy.sleep(0.5)
        print("Waiting for bets...")
        while not self.all_bets_placed():
            rospy.sleep(0.5)
        

    def all_bets_placed(self):
        """Returns true if all players in the game have placed their bets"""
        for player in self.player_list:
            if player is not None and player.bet is None:
                return False
        return True


    def all_cards_detected(self):
        for player in self.player_list:
            if player is not None:
                for card in player.hand:
                    if not card.detected:
                        return False
        return True


    def play_game(self):
        while True:
            if self.get_player_count() == 0:
                rospy.sleep(0.5)
                continue

            self.wait_for_bets()

            self.deal_initial_cards()

            while self.play_flag:

                while not self.all_cards_detected():
                    rospy.sleep(0.5)
                    self.fake_card_update_callback()

                # Check players's hand values
                continue_flag = self.check_values()

                # If game can continue
                if continue_flag:
                    # WILL NEED TO CHANGE
                    # Get player input
                    self.get_player_input()
                else:
                    # Clear flag
                    self.play_flag = False
                    # Break from loop
                    break

            # REMEMBER TO FLIP FACE DOWN CARD FIRST IN IT
            self.finish_dealer_hand()

            # Reveal dealer cards and determine winner
            self.final_reveal()
            
            # Process players leaving

            # Kick 
            for i in range(len(self.player_list)):
                if self.player_list[i] != None:
                    print(self.player_list[i].name + ": " + str(self.player_list[i].wallet))
                    if self.player_list[i].wallet <= 0:
                        self.player_list[i] = None
                    else:
                        self.player_list[i].reset()
            self.dealer.reset()
            self.play_flag = True

            # active_players = max(self.update_players(),active_players)

        # active_players = 0
        # print("Players on table:")
        # for player in self.player_list:
        #     if player != None:
        #         if player.wallet > 0:
        #             print(str(player.id) + ": " + str(player.wallet))
        #             active_players += 1

        # while active_players > 0:
        #     self.get_wagers()
        #     print()

        #     # Deal initial cards
        #     self.deal_initial_cards()
        #     print()

        #     # While play can continue
        #     while self.play_flag:

        #         # Check players's hand values
        #         continue_flag = self.check_values()

        #         # If game can continue
        #         if continue_flag:
                    
        #             # WILL NEED TO CHANGE
        #             # Get player input
        #             self.get_player_input()


        #         else:
        #             # Clear flag
        #             self.play_flag = False
        #             # Break from loop
        #             break

        #     # REMEMBER TO FLIP FACE DOWN CARD FIRST IN IT
        #     self.finish_dealer_hand()

        #     # Reveal dealer cards and determine winner
        #     self.final_reveal()

        #     print("New round\n")

        #     # DAVIDS PLAYER UPATE STUFF
        #     active_players = 0
        #     print("Players on table:")
        #     for i in range(len(self.player_list)):
        #         if self.player_list[i] != None:
        #             print(self.player_list[i].name + ": " + str(self.player_list[i].wallet))
        #             if self.player_list[i].wallet > 0:
        #                 active_players += 1
        #             else:
        #                 self.player_list[i] = None
        #             player.reset()

        #     self.dealer.reset()

        #     print()
        #     self.play_flag = True

        #     active_players = max(self.update_players(),active_players)
            

        # # Print message
        # print("[GAME]: Game over.")




