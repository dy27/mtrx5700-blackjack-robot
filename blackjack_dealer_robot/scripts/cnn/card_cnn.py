"""
MTRX5700 - Experimental Robotics
Major Project - Blackjack Robot
Year: 2021
Group 5 - Curry Shop

File: 
Info: 
"""

# Imports
import torch.nn as nn
from torch.nn.modules.activation import Sigmoid

# Class for CNN to detect cards
class CardCNN(nn.Module):

	# Class constructor
	def __init__(self):
		super(CardCNN, self).__init__()

		self.network = nn.Sequential(
            
            nn.Conv2d(1, 32, kernel_size = 3, padding = 1),
            nn.ReLU(),
            nn.Conv2d(32,64, kernel_size = 3, stride = 1, padding = 1),
            nn.ReLU(),
            nn.MaxPool2d(2,2),
        
            nn.Conv2d(64, 128, kernel_size = 3, stride = 1, padding = 1),
            nn.ReLU(),
			nn.BatchNorm2d(128),
            nn.Conv2d(128 ,128, kernel_size = 3, stride = 1, padding = 1),
            nn.ReLU(),
			nn.BatchNorm2d(128),
			nn.Conv2d(128 ,128, kernel_size = 3, stride = 1, padding = 1),
            nn.ReLU(),
			nn.BatchNorm2d(128),
            nn.MaxPool2d(2,2),
            
            nn.Conv2d(128, 256, kernel_size = 3, stride = 1, padding = 1),
            nn.ReLU(),
			nn.BatchNorm2d(256),
            nn.Conv2d(256,256, kernel_size = 3, stride = 1, padding = 1),
            nn.ReLU(),
			nn.BatchNorm2d(256),
			nn.Conv2d(256,256, kernel_size = 3, stride = 1, padding = 1),
            nn.ReLU(),
			nn.BatchNorm2d(256),
            nn.MaxPool2d(2,2),

			nn.Conv2d(256, 512, kernel_size = 3, stride = 1, padding = 1),
            nn.ReLU(),
			nn.BatchNorm2d(512),
            nn.Conv2d(512,512, kernel_size = 3, stride = 1, padding = 1),
            nn.ReLU(),
			nn.BatchNorm2d(512),
			nn.Conv2d(512,512, kernel_size = 3, stride = 1, padding = 1),
            nn.ReLU(),
			nn.BatchNorm2d(512),
            nn.MaxPool2d(2,2),
			nn.Dropout(p=0.2),
            
            nn.Flatten(),
            nn.Linear(41472,1024),
            nn.ReLU(),
			nn.Dropout(p=0.4),
            nn.Linear(1024, 512),
            nn.ReLU(),
			nn.Dropout(p=0.4),
            nn.Linear(512,2)
        )

	def forward(self, x):
		x = self.network(x)
		# print(x.size())
		return x