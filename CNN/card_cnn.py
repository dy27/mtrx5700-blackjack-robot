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

# Class for CNN to detect cards
class Card_CNN(nn.Module):

	# Class constructor
	def __init__(self):
		super(Card_CNN, self).__init__()

		# Convolution 1
		self.cnn1 = nn.Conv2d(in_channels=1, out_channels=32, kernel_size=3, stride=1, padding=0)
		self.relu1 = nn.ReLU()
		self.bn1 = nn.BatchNorm2d(32)

		# Convolution 2
		self.cnn2 = nn.Conv2d(in_channels=32, out_channels=32, kernel_size=3, stride=1, padding=0)
		self.relu2 = nn.ReLU()
		self.bn2 = nn.BatchNorm2d(32)

		# Convolution 3
		self.cnn3 = nn.Conv2d(in_channels=32, out_channels=32, kernel_size=5, stride=2, padding=0)
		self.relu3 = nn.ReLU()
		self.bn3 = nn.BatchNorm2d(32)

		self.dropout1 = nn.Dropout(p=0.4)

		# Convolution 4
		self.cnn4 = nn.Conv2d(in_channels=32, out_channels=64, kernel_size=3, stride=1, padding=0)
		self.relu4 = nn.ReLU()
		self.bn4 = nn.BatchNorm2d(64)

		# Convolution 5
		self.cnn5 = nn.Conv2d(in_channels=64, out_channels=64, kernel_size=3, stride=1, padding=0)
		self.relu5 = nn.ReLU()
		self.bn5 = nn.BatchNorm2d(64)

		# Convolution 6
		self.cnn6 = nn.Conv2d(in_channels=64, out_channels=64, kernel_size=5, stride=2, padding=0)
		self.relu6 = nn.ReLU()
		self.bn6 = nn.BatchNorm2d(64)

		# flatten

		# dropout
		self.dropout2 = nn.Dropout(p=0.4)

		# Fully connected 1
		# self.fc1 = nn.Linear(123904, 13)
		self.fc1 = nn.Linear(123904, 1200)
		self.fc2 = nn.Linear(1200, 84)
		self.fc3 = nn.Linear(84, 11)

	def forward(self, x):
		out = self.cnn1(x)
		out = self.relu1(out)
		out = self.bn1(out)

		out = self.cnn2(out)
		out = self.relu2(out)
		out = self.bn2(out)

		out = self.cnn3(out)
		out = self.relu3(out)
		out = self.bn3(out)

		out = self.dropout1(out)

		out = self.cnn4(out)
		out = self.relu4(out)
		out = self.bn4(out)

		out = self.cnn5(out)
		out = self.relu5(out)
		out = self.bn5(out)

		out = self.cnn6(out)    
		out = self.relu6(out)
		out = self.bn6(out)

		# Flatten
		out = out.view(out.size(0), -1)

		out = self.dropout2(out)
		# print(out.shape)
		# Dense
		out = self.fc1(out)
		out = self.fc2(out)
		out = self.fc3(out)

		return out