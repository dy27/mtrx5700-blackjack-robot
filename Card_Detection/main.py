"""
MTRX5700 - Experimental Robotics
Major Project - Blackjack Robot
Year: 2021
Group 5 - Curry Shop

File: main.py
Info: Run the card detection algorithm.
"""

# Imports
from Detector import Detector
# from Card_Detector import Card_Detector as Detector
import cv2 as cv
import sys

# Function to scale image
def scale_img(src_img, scale):

    # Extract image dimensions
    height = src_img.shape[0]
    width = src_img.shape[1]
    
    # Scale dimensions
    new_height = int(height * scale/100)
    new_width = int(width * scale/100)
    
    # Resize image
    img = cv.resize(src_img, (new_width, new_height), interpolation = cv.INTER_AREA)
    
    # Return new image
    return img

# Main function
if __name__ == "__main__":

    # If extra arguments are given
    if len(sys.argv) == 2:
        # Set path
        img_path = sys.argv[1]
    else:
        # Default image path
        img_path = "Input_Images\\test_01.jpg"
    
    # 
    scale = 1
    
    # 
    img = cv.imread(img_path)
    # 
    img = scale_img(img, 100*scale)

    # Create card detector
    detector = Detector()
    detector.detect_cards(img)

    # Close all windows
    cv.destroyAllWindows()
