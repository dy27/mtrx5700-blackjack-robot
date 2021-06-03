"""
MTRX5700 - Experimental Robotics
Major Project - Blackjack Robot
Year: 2021
Group 5 - Curry Shop

File: 
Info: 
"""

# Imports
import os
import random
import time
import torch
import torchvision
import cv2 as cv
import numpy as np
from torch import nn, optim
from torch.utils.data import Dataset, DataLoader
from torchvision import transforms
from PIL import Image
from custom_dataset import Custom_Dataset
from card_cnn import Card_CNN

# Set paths
test_imgs = os.path.join("dataset_02", "test_imgs")
test_labs = os.path.join("dataset_02", "test.csv")

# Hyperparameters

# Transform to use when evaluating model
custom_transform = transforms.Compose([	transforms.ToPILImage(),
                                        transforms.CenterCrop(2000),
										transforms.Resize((200,200)),
										transforms.Grayscale(),
                                        transforms.ToTensor(),
                                        transforms.Normalize((0.5, ), (0.5, )),
                                       ])

# Define the dataloaders
# -----------------------
# Dataloaders help in managing large amounts of data efficiently

test_dataset = Custom_Dataset(test_imgs, test_labs, custom_transform)
test_loader = DataLoader(test_dataset, shuffle=True)

print("Datasets created.")

model = Card_CNN()
if torch.cuda.is_available():
    model.cuda()


#####################################################################################################
# Step 5: Evaluate the accuracy of your network on the test dataset
#--------------------------------------------------------------------
# Use the above trained model and predict labels for your test data - this is nothing but the
# forward pass.
# Check accuracy by comparing with the ground truth labels and report it.
# What are the cases where the accuracy is low?
# Can you visualize these cases?

# Note: If you are loading a saved model then you must call model.eval() before you do inference.
# model = torch.load(PATH)
# model.eval()
#####################################################################################################

model = Card_CNN()
if torch.cuda.is_available():
    model.cuda()
model.load_state_dict(torch.load("model-11"))

confusion_matrix = [[0 for j in range(52)] for i in range(52)]
incorrect_preds = []

total_pred_count = 0
incorrect_pred_count = 0

model.eval()
with torch.no_grad():
	for _, (x, y) in enumerate(test_loader):
		output = model(x)
		pred = np.argmax(output).numpy()
		label = y.item()
		# print("Prediction: {}; Actual: {}".format(pred, label))

		confusion_matrix[label][pred] += 1
		if pred != label:
			incorrect_preds.append((x, label, pred))
			incorrect_pred_count += 1

		total_pred_count += 1


print("Test Accuracy: {:2f}".format((1 - incorrect_pred_count/total_pred_count) * 100))

