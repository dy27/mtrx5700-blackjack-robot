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
train_imgs = os.path.join("dataset_02", "train_imgs")
train_labs = os.path.join("dataset_02", "train.csv")
valid_imgs = os.path.join("dataset_02", "valid_imgs")
valid_labs = os.path.join("dataset_02", "valid.csv")
# test_imgs = os.path.join("custom_dataset_01", "imgs")
# test_labs = os.path.join("custom_dataset_01", "labels.csv")

# Hyperparameters
BATCH_SIZE = 5
NUM_EPOCHS = 10
LEARNING_RATE = 0.00001

# Transform to use when evaluating model
custom_transform = transforms.Compose([	transforms.ToPILImage(),
                                        transforms.CenterCrop(2000),
										transforms.Resize((200,200)),
										transforms.Grayscale(),
                                        transforms.ToTensor(),
                                        transforms.Normalize((0.5, ), (0.5, )),
                                       ])

# Transform including data augmentation to use when training model
train_transform = transforms.Compose([	transforms.ToPILImage(),
                                        transforms.CenterCrop(2000),
                                        transforms.ColorJitter(brightness=0.5, contrast=0.5),
										transforms.RandomHorizontalFlip(),
										transforms.RandomVerticalFlip(),
										# transforms.RandomPerspective(),
										transforms.Resize((200,200)),
                                        transforms.Grayscale(),
                                        transforms.ToTensor(),
                                        transforms.Normalize((0.5, ), (0.5, )),
                                       ])

# Define the dataloaders
# -----------------------
# Dataloaders help in managing large amounts of data efficiently
train_dataset = Custom_Dataset(train_imgs, train_labs, train_transform)
train_loader = DataLoader(train_dataset, shuffle=True, batch_size=BATCH_SIZE)

valid_dataset = Custom_Dataset(valid_imgs, valid_labs, custom_transform)
valid_loader = DataLoader(valid_dataset, shuffle=True, batch_size=BATCH_SIZE)

print("Datasets created.")

model = Card_CNN()
if torch.cuda.is_available():
    model.cuda()


#####################################################################################################
# Step 3: Defining loss function and optimizer
#-----------------------------------------------
# Define your loss function and optimizer here - refer to lecture slides for help.
# Explore different loss functions e.g. torch.nn.CrossEntropyLoss(), torch.nn.MSELoss(), torch.nn.NLLLoss()
# Explore different optimizers e.g. torch.optim.SGD(), torch.optim.Adam()
# Explore different values of learning rates and momentum.
#####################################################################################################

loss_function = torch.nn.CrossEntropyLoss()
optimizer = torch.optim.Adam(model.parameters(), lr=LEARNING_RATE)
print("Optimiser created.")

#####################################################################################################
# Step 4: Main training loop
#-----------------------------------------------
# Refer to lecture slides for help.
# Loop over your entire training data in 'epochs'
# For each epoch, get one minibatch from the dataset, perform forward pass, compute loss,
# backpropagation, adjust weight through optimization step
#
#####################################################################################################
print("Starting model training...")

loss_list = []
iteration_list = []
accuracy_list = []
for epoch in range(NUM_EPOCHS):
	for i, (images, labels) in enumerate(train_loader):

		train_loss = 0.0
		valid_loss = 0.0

		train_total = 0
		val_total = 0

		train_correct = 0
		val_correct = 0

		model.train()
		if torch.cuda.is_available():
			images, labels = images.cuda(), labels.cuda()

		optimizer.zero_grad()
		output = model(images)
		loss = loss_function(output, labels)
		loss.backward()
		optimizer.step()


		# Calculate loss and accuracy
		train_loss += loss.item()*images.size(0)

		scores, predictions = torch.max(output.data, 1)
		train_total += labels.size(0)
		train_correct += int(sum(predictions == labels))
		acc = round((train_correct / train_total) * 100, 5)
        

	# Validate Model
	model.eval()
	for idx, (images, labels) in enumerate(valid_loader):
		if torch.cuda.is_available():
			images, labels = images.cuda(), labels.cuda()
		output = model(images)
		loss = loss_function(output, labels)
		valid_loss += loss.item()*images.size(0)
		scores, predictions = torch.max(output.data, 1)
		val_total += labels.size(0)
		val_correct += int(sum(predictions == labels))
		val_acc = round((val_correct / val_total) * 100, 5)
    

	train_loss = train_loss/len(train_loader.sampler)
	valid_loss = valid_loss/len(valid_loader.sampler)
	print("Epoch:{} \tTL:{} \tTA:{} \tVL:{} \tVA:{}".format(epoch, train_loss, acc, valid_loss, val_acc))
	# print("Epoch:{} \tTL:{} \tTA:{}".format(epoch, train_loss, acc))

torch.save(model.state_dict(), "model-11")


