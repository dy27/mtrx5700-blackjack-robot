import rospy
import math
import numpy as np

import actionlib
import blackjack_dealer_robot.msg
from blackjack_dealer_robot.srv import BalanceUpdate, JoinLeaveGame, PlaceBet, PlayerHitStay
from std_msgs.msg import Int64

from blackjack_classes.BlackjackGame import BlackjackGame

import std_msgs

class Timer:
    def __init__(self, time_secs):
        self.start_time = None
        self.timer_time = time_secs

    def start(self):
        self.start_time = rospy.get_time()

    def is_finished(self):
        return True if rospy.get_time() >= self.start_time + self.timer_time else False


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
        rospy.init_node('game_manager')

        self.max_players = 3

        # Player action subscriber
        # self.sub_player_actions = rospy.Subscriber("/player_actions", blackjack_dealer_robot.msg.PlayerAction, self.player_action_callback)

        # Player join subscriber
        self.sub_join = rospy.Subscriber("/player_join", std_msgs.msg.String, self.join_callback)

        # Player bets subscriber
        self.sub_bet = rospy.Subscriber("/player_bets", std_msgs.msg.String, self.place_bet_callback)

        # Player bets subscriber
        self.sub_hit_stay = rospy.Subscriber("/player_hit_stay", std_msgs.msg.String, self.player_hit_stay_callback)

        # Action client for moving the dealer arm
        self.action_client = actionlib.SimpleActionClient('dealer_action', blackjack_dealer_robot.msg.DealerAction)
        # self.action_client.wait_for_server()

        # self.blackjack_game = BlackjackGame(max_players=3, action_client=self.action_client, game_status_pub=self.pub_game_status)
        self.blackjack_game = None
        self.blackjack_game = BlackjackGame(max_players=3)

        self.blackjack_game.play_game()


    def join_callback(self, msg):
        print(msg)
        try:
            args = msg.data.split(',')
            player_id = int(args[0])
            initial_balance = int(args[1])
        except:
            return
        self.blackjack_game.add_player(player_id, initial_balance)


    def balance_update_callback(self, msg):
        pass


    def place_bet_callback(self, msg):
        print(msg)
        try:
            args = msg.data.split(',')
            player_id = int(args[0])
            initial_balance = int(args[1])
        except:
            return


    def player_hit_stay_callback(self, msg):
        print(msg)
        if self.blackjack_game.game_status == GameStatus.WAITING_FOR_ACTION:

            try:
                args = msg.data.split(',')
                player_id = int(args[0])
                action = args[1]
            except:
                return

            player = self.blackjack_game.player_list[player_id]
            if player is not None:
                if action == 'hit':
                    player.action = 'h'

                elif action == 'stay':
                    player.action = 's'


if __name__ == '__main__':
    BlackjackGameManager()