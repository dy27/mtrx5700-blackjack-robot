"""
MTRX5700 - Experimental Robotics
Major Project - Blackjack Robot
Year: 2021
Group 5 - Curry Shop

File: 
Info:
"""

# Card image class
class Card_Img:

	# Class constructor
	def __init__(self):

		# 
		self.contour = []
		self.width = 0
		self.height = 0
		self.corners = []
		self.centre = []
		self.warp = []
		self.rank_img = []
		self.suit_img = []
		self.best_rank_match = "None"
		self.best_suit_match = "None"
		self.rank_diff = 0
		self.suit_diff = 0


