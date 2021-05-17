"""
MTRX5700 - Experimental Robotics
Major Project - Blackjack Robot
Year: 2021
Group 5 - Curry Shop

card-labelling.py
File to detect and extract cards in an image.
"""

# Imports
from Card_Detector import Card_Detector
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
    
    print(sys.argv)

    if len(sys.argv) == 2:
        img_path = sys.argv[1]
        print("custom")
    else:
        img_path = "Input_Images\\test_01.jpg"
        print("default")

    # Create card detector
    detector = Card_Detector()
    
    # Read image and scale down
    img1 = cv.imread(img_path)
    # cv.imshow("image1", img1)
    cv.imshow("image1", scale_img(img1, 20))

    # Preprocess image
    img2 = detector.preprocess(img1)
    # cv.imshow("image2", img2)
    cv.imshow("image2", scale_img(img2, 20))

    # Find contours of cards
    contours = detector.find_cards(img2)

    # Draw contours
    img3 = cv.drawContours(img1, contours, -1, (255,0,0), 10)
    # cv.imshow("image3", img3)
    cv.imshow("image3", scale_img(img3, 20))

    # Wait for keystroke
    cv.waitKey(0)

    # Close all windows
    cv.destroyAllWindows()