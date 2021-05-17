"""
MTRX5700 - Experimental Robotics
Major Project - Blackjack Robot
Year: 2021
Group 5 - Curry Shop

card-labelling.py
File to generate dataset labels.
"""

# Imports
import csv
import os

# Card dictionary to assign integer value from 0 to 51 for cards
card_dict = {"C01" : 0,  "C02" : 1,  "C03" : 2,  "C04" : 3,  "C05" : 4,  "C06" : 5,  "C07" : 6,
             "C08" : 7,  "C09" : 8,  "C10" : 9,  "C11" : 10, "C12" : 11, "C13" : 12,

             "D01" : 13, "D02" : 14, "D03" : 15, "D04" : 16, "D05" : 17, "D06" : 18, "D07" : 19,
             "D08" : 20, "D09" : 21, "D10" : 22, "D11" : 23, "D12" : 24, "D13" : 25,
             
             "H01" : 26, "H02" : 27, "H03" : 28, "H04" : 29, "H05" : 30, "H06" : 31, "H07" : 32,
             "H08" : 33, "H09" : 34, "H10" : 35, "H11" : 36, "H12" : 37, "H13" : 38, 
             
             "S01" : 39, "S02" : 40, "S03" : 41, "S04" : 42, "S05" : 43, "S06" : 44, "S07" : 45,
             "S08" : 46, "S09" : 47, "S10" : 48, "S11" : 49, "S12" : 50, "S13" : 51}

# Function to create card labels
def create_labels(folder, file):
    # List of files in folder
    file_list = os.listdir(folder)
    # Use file name to open a csv file
    with open(file, "w", newline="") as csv_file:
        # Initialise writer
        csv_writer = csv.writer(csv_file)
        # Iterate through each file name
        for f in file_list:
            # Extract card value using part of file name as dictionary key
            card_num = card_dict[f[0:3]]
            # Create new list combining file name and card number
            row = [f, card_num]
            # Write row to csv file
            csv_writer.writerows([row])

# Main function
if __name__ == "__main__":
    # Set path to folder with card images
    folder_path = "Card_Dataset\\Card_Images"
    # Set file name for label file
    label_filename = "Card_Dataset\\card_labels.csv"
    # Create labels
    create_labels(folder_path, label_filename)