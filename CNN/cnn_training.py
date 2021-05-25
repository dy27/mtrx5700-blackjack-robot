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
folder_path = "card_dataset"
train_imgs = os.path.join(folder_path, "training_data")
train_labs = os.path.join(folder_path, "training_data.csv")
test_imgs = os.path.join(folder_path, "testing_data")
test_labs = os.path.join(folder_path, "testing_data.csv")
valid_imgs = os.path.join(folder_path, "validation_data")
valid_labs = os.path.join(folder_path, "validation_data.csv")

# Hyperparameters
BATCH_SIZE = 32
NUM_EPOCHS = 10
LEARNING_RATE = 3e-4

# Transform to use when evaluating model
custom_transform = transforms.Compose([	transforms.ToPILImage(),
										transforms.Resize((150,100)),
										transforms.Grayscale(),
                                        transforms.ToTensor(),
                                        transforms.Normalize((0.5, ), (0.5, )),
                                       ])

# Transform including data augmentation to use when training model
train_transform = transforms.Compose([	transforms.ToPILImage(),
										transforms.Resize((150,100)),
										transforms.RandomRotation(5),
                                        transforms.RandomCrop(32), 
                                        transforms.ColorJitter(brightness=0.5, contrast=0.5, saturation=0.5, hue=0.5),
                                        transforms.Grayscale(),
                                        transforms.ToTensor(),
                                        transforms.Normalize((0.5, ), (0.5, )),
                                       ])

# Define the dataloaders
# -----------------------
# Dataloaders help in managing large amounts of data efficiently
train_dataset = Custom_Dataset(train_imgs, train_labs, train_transform)
train_loader = DataLoader(train_dataset, shuffle=True, batch_size=BATCH_SIZE)

valid_dataset = Custom_Dataset(valid_imgs, valid_labs, train_transform)
valid_loader = DataLoader(valid_dataset, shuffle=True, batch_size=BATCH_SIZE)

test_dataset = Custom_Dataset(test_imgs, test_labs, train_transform)
test_loader = DataLoader(test_dataset, shuffle=True)


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

#####################################################################################################
# Step 4: Main training loop
#-----------------------------------------------
# Refer to lecture slides for help.
# Loop over your entire training data in 'epochs'
# For each epoch, get one minibatch from the dataset, perform forward pass, compute loss,
# backpropagation, adjust weight through optimization step
#
#####################################################################################################

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
	valid_loss = valid_loss/len(test_loader.sampler)
	print("Epoch:{} \tTL:{} \tTA:{} \tVL:{} \tVA:{}".format(epoch, train_loss,
													acc, valid_loss, val_acc))

torch.save(model.state_dict(), "model")


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
model.load_state_dict(torch.load("model"))

confusion_matrix = [[0 for j in range(52)] for i in range(52)]
incorrect_preds = []

total_pred_count = 0
incorrect_pred_count = 0

model.eval()
with torch.no_grad():
	for _, (x, y) in enumerate(test_loader):
		# img = Image.fromarray(x)
		# img = custom_transform(img)
		# img = torch.unsqueeze(img, 0)
		output = model(x)
		pred = np.argmax(output).numpy()

		confusion_matrix[y][pred] += 1
		if pred != y:
			incorrect_preds.append((x, y, pred))
			incorrect_pred_count += 1

		total_pred_count += 1


print("Test Accuracy:", (1 - incorrect_pred_count/total_pred_count) * 100)

# for row in confusion_matrix:
#     print(row)

# plt.figure(figsize=(10,10))
# i = 1
# for img, label, pred in incorrect_preds:
#     ax = plt.subplot(3, 3, i)
#     plt.imshow(img)
#     plt.title('Label: ' + str(label) + ', Predicted: ' + str(pred))
#     plt.axis('off')
#     i += 1
#     if i > 9:
#         break

# plt.show()


#####################################################################################################
# Step 6: Evaluate the accuracy of your network on your own data
#--------------------------------------------------------------------
# Use images that are not part of the training dataset.
# We have provided a larger dataset from which you can pick images.
# What does the network predict and why?
#####################################################################################################
# extradata_file = "extradata.pkl"

# with open(extradata_file, mode='rb') as f:
#     extra_data = pickle.load(f)

# x_extra, y_extra = extra_data['features'], extra_data['labels']
# xy_shuffled = [(x,y) for x,y in zip(x_test, y_test)]
# random.shuffle(xy_shuffled)
# x_extra = [xy[0] for xy in xy_shuffled]
# y_extra = [xy[1] for xy in xy_shuffled]

# plt.figure(figsize=(10,3))
# i = 1
# with torch.no_grad():
#     for x, y in zip(x_extra, y_extra):
#         img = Image.fromarray(x)      
#         img = custom_transform(img)
#         img = torch.unsqueeze(img, 0)
#         output = model(img)
#         pred = np.argmax(output).numpy()

#         ax = plt.subplot(1, 3, i)
#         plt.imshow(x)
#         plt.title('Label: ' + str(y) + ', Predicted: ' + str(pred))
#         plt.axis('off')

#         i += 1

#         if i > 3:
#             break

# plt.savefig('examples.png')
# plt.show()

