"""
MTRX5700 - Experimental Robotics
Major Project - Blackjack Robot
Year: 2021
Group 5 - Curry Shop

card-labelling.py
File to detect and extract cards in an image.
"""

# Imports
from Detector import Detector
import cv2 as cv
import sys

# Main function
if __name__ == "__main__":

    # If extra arguments are given
    if len(sys.argv) == 2:
        # Set path
        img_path = sys.argv[1]
    else:
        # Default image path
        img_path = "Input_Images\\test_01.jpg"

    # Create card detector
    detector = Detector()
    img = cv.imread(img_path)
    detector.detect_cards(img)
