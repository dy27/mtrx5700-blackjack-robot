import rospy
import math
import numpy as np

import actionlib
import blackjack_dealer_robot.msg

class Timer:
    def __init__(self, time_secs):
        self.start_time = None
        self.timer_time = time_secs

    def start():
        self.start_time = rospy.get_time()

    def is_finished():
        if rospy.get_time() >= self.start_time + self.timer_time:
            return True
        else:
            return False


class GameStatus:
    WAITING_FOR_PLAYERS = 0
    WAITING_FOR_BETS = 1
    DEALING = 2
    WAITING_FOR_ACTION = 3


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


class BlackjackGameManager:
    def __init__(self):
        # Initialise the ROS node
        rospy.init_node('blackjack_game')

        self.max_players = 4

        # Player action subscriber
        self.sub_player_actions = rospy.Subscriber("/player_actions", blackjack_dealer_robot.msg.PlayerAction, self.player_action_callback)

        # Card detection subscriber
        self.sub_card_detections = rospy.Subscriber("/card_detections", Image, self.depth_callback)

        # 

        # 
        self.pub_card_move = rospy.Publisher("/card_move", Landmarks_Msg, queue_size=10)
        # self.marker_pub = rospy.Publisher("/landmark_markers_gt", MarkerArray, queue_size=10)


        self.card_shape = (0.064, 0.089)
        self.card_spacing = ()
        self.player_spacing = 0.05


        self.player_coordinates = [(0.08, 0.2), (0.4, 0.2), (0.72, 0.2), (1.04, 0.2)]
        self.player_card_space_shape = (2, 3)

        self.dealer_coordinate = (0.5, 0.5)
        self.dealer_card_space_shape = (1, 6)


        self.blackjack_game = BlackjackGame(max_players=4)


        # self.player_join_queue = []
        self.player_leave_queue = []
        # self.player_move_queue = []
        

        self.players = [None] * self.max_players
        self.player_bets = [None] * self.max_players
        self.player_moves = [None] * self.max_players
        self.player_balances = [None] * self.max_players

        self.game_status = GameStatus.WAITING_FOR_PLAYERS



        self.action_client = actionlib.SimpleActionClient('dealer_action', blackjack_dealer_robot.msg.DealerAction)
        self.action_client.wait_for_server()

        self.robot_start()

            
        # rospy.spin()

    def robot_start(self):
        """Robot operation loop."""

        while True:

            self.game_status = GameStatus.WAITING_FOR_PLAYERS

            # Process player leaves

            # Process player joins

            # Wait for players
            while len(self.players) == 0:

                rospy.sleep(0.5)

            
            self.game_status = GameStatus.WAITING_FOR_BETS
            self.blackjack_game.wait_for_bets()

            bet_timer = Timer(15)
            bet_timer.start()
            while not bet_timer.is_finished() and not self.all_bets_placed():
                rospy.sleep(0.5)


            # Remove players which didn't place bets
            for i in range(len(players)):
                if players[i] is not None and player_bets[i] is None:
                    players[i] = None
                


            self.game_status = GameStatus.DEALING
            self.blackjack_game.initial_deal()

            goal = blackjack_dealer_robot.msg.DealerGoal()
            goal.type = int(sys.argv[1])
            goal.pose = geometry_msgs.msg.Pose()
            goal.pose.position.x = 0.0
            goal.pose.position.y = 0.5
            goal.pose.position.z = 0.05
            goal.pose.orientation.x = -0.5
            goal.pose.orientation.y = 0.5
            goal.pose.orientation.z = 0.5
            goal.pose.orientation.w = 0.5
            # Sends the goal to the action server.
            client.send_goal(goal)

            # Waits for the server to finish performing the action.
            client.wait_for_result()

            # Prints out the result of executing the action
            result = client.get_result()







            



    def all_bets_placed(self):
        """Returns true if all players in the game have placed their bets"""
        for player, bet in zip(self.players, self.player_bets):
            if player is not None and bet is None:
                return False
        return True


    def all_moves_placed(self):
        """Returns true if all players in the game have selected their move for the round"""
        for player, move in zip(self.players, self.player_moves):
            if player is not None and move is None:
                return False
        return True


    def player_action_callback(self, msg):
        if msg.action_type == PlayerActions.JOIN_GAME:
            player_index = msg.value
            if self.players[player_index] is None:
                pass

        elif msg.action_type == PlayerActions.LEAVE_GAME:

        elif msg.action_type == PlayerActions.HIT:

        elif msg.action_type == PlayerActions.STAND:

        if self.game_status == GameStatus.WAITING_FOR_PLAYERS:

            blackjack_game.add_player()

            remove_player







class BlackjackGame:
    def __init__(self, max_players):
        self.max_players = max_players
        self.players = [False for i in range(max_players)]
        self.cards_remaining = 52
        self.turn = None

        self.player_hands = [[] for i in range(max_players)]
        self.dealer_hand = []

        # self.cards = []

    def add_player():
        pass

    def remove_player():
        pass

    def next_card(card):
        """Pass the value of the next card """
        pass

    def get_player_hand_value(player_index):
        pass

    def clear_board():
        pass


class Player:
    pass

class Card:
    def __init__(self, value, suit, pose):
        self.value = value
        self.suit = suit
        self.pose = pose

class GameState:
    pass


if __name__ == '__main__':
    BlackjackGameManager()