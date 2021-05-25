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
import pandas as pd
from torchvision.io import read_image
from torch.utils.data import Dataset
from sklearn import preprocessing

# Class for custom image dataset
class Custom_Dataset(Dataset):

    # Constructor
    def __init__(self, img_dir, annotations_file, transform=None, target_transform=None):

        # 
        self.img_labels = pd.read_csv(annotations_file)
        # 
        self.img_labels.convert_dtypes().dtypes
        # 
        self.img_dir = img_dir
        # 
        self.transform = transform
        # 
        self.target_transform = target_transform

    # 
    def __len__(self):
        
        # 
        return len(self.img_labels)

    # 
    def __getitem__(self, idx):

        # 
        img_path = os.path.join(self.img_dir, self.img_labels.iloc[idx, 0])
        # 
        image = read_image(img_path)
        # 
        label = self.img_labels.iloc[idx, 1]
        
        # 
        if self.transform:
            # 
            image = self.transform(image)
        
        # 
        if self.target_transform:
            # 
            label = self.target_transform(label)
        
        # 
        # sample = {"image": image, "label": label}
        # 
        return image, label