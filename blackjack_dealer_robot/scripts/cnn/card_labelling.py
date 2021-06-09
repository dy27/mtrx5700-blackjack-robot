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
import sys

# Card dictionary to assign integer value from 0 to 51 for cards
# card_dict = {"C01" : 0,  "C02" : 1,  "C03" : 2,  "C04" : 3,  "C05" : 4,  "C06" : 5,  "C07" : 6,
#              "C08" : 7,  "C09" : 8,  "C10" : 9,  "C11" : 10, "C12" : 11, "C13" : 12,

#              "D01" : 13, "D02" : 14, "D03" : 15, "D04" : 16, "D05" : 17, "D06" : 18, "D07" : 19,
#              "D08" : 20, "D09" : 21, "D10" : 22, "D11" : 23, "D12" : 24, "D13" : 25,
             
#              "H01" : 26, "H02" : 27, "H03" : 28, "H04" : 29, "H05" : 30, "H06" : 31, "H07" : 32,
#              "H08" : 33, "H09" : 34, "H10" : 35, "H11" : 36, "H12" : 37, "H13" : 38, 
             
#              "S01" : 39, "S02" : 40, "S03" : 41, "S04" : 42, "S05" : 43, "S06" : 44, "S07" : 45,
#              "S08" : 46, "S09" : 47, "S10" : 48, "S11" : 49, "S12" : 50, "S13" : 51}

# label_reduction = lambda label : label % 12

card_dict = {"C01" : 0,  "C02" : 0,  "C03" : 0,  "C04" : 0,  "C05" : 0,  "C06" : 0,  "C07" : 0,
             "C08" : 0,  "C09" : 0,  "C10" : 0,  "C11" : 1, "C12" : 1, "C13" : 1,

             "D01" : 0, "D02" : 0, "D03" : 0, "D04" : 0, "D05" : 0, "D06" : 0, "D07" : 0,
             "D08" : 0, "D09" : 0, "D10" : 0, "D11" : 1, "D12" : 1, "D13" : 1,
             
             "H01" : 0, "H02" : 0, "H03" : 0, "H04" : 0, "H05" : 0, "H06" : 0, "H07" : 0,
             "H08" : 0, "H09" : 0, "H10" : 0, "H11" : 1, "H12" : 1, "H13" : 1, 
             
             "S01" : 0, "S02" : 0, "S03" : 0, "S04" : 0, "S05" : 0, "S06" : 0, "S07" : 0,
             "S08" : 0, "S09" : 0, "S10" : 0, "S11" : 1, "S12" : 1, "S13" : 1}

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
            # card_num = label_reduction(card_num)
            # Create new list combining file name and card number
            row = [f, card_num]
            # Write row to csv file
            csv_writer.writerows([row])

# Main function
if __name__ == "__main__":
    # Set path to folder with card images
    folder_path = sys.argv[1]
    # Set file name for label file
    label_filename = sys.argv[2]
    # Create labels
    create_labels(folder_path, label_filename)