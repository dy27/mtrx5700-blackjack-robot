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
import cv2 as cv
import numpy as np

# Card detector class
class Detector:
    
    # Constructor
    def __init__(self):
        
        # Set value for threshold filter
        self.THRESH = 120

        # Set maximum and minimum card area
        self.CARD_AREA_MAX = 1000000
        self.CARD_AREA_MIN = 1000

    # Preprocess image
    def preprocess(self, image):
        
        # Convert to grayscale
        gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
        # Apply blur
        blur = cv.GaussianBlur(gray, (5,5), 0)
        
        # Extract level of background to compute an adaptive threshold
        # w, h = image.shape[0:2]
        # bkg_level = gray[int(h/100)][int(w/2)]
        bkg_level = gray[0][0]
        thresh_filter = bkg_level + self.THRESH

        # Filter with adaptive threshold
        _, thresh_img = cv.threshold(blur, thresh_filter, 255, cv.THRESH_BINARY)

        # Return thresholded image
        return thresh_img

    # Find cards by examining contours in image
    def find_cards(self, img):

        # Extract contours
        # contours, heir = cv.findContours(img, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        _, contours, _ = cv.findContours(img, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        # Sort contours
        contours = sorted(contours, key=cv.contourArea, reverse=True)

        # Empty list of contours
        card_contours = []

        # Iterate through contour found
        for i in contours:
            
            # Get size of contour
            size = cv.contourArea(i)
            # Get perimeter
            perimeter = cv.arcLength(i, True)
            # Get vertices of contour
            vertices = cv.approxPolyDP(i, 0.01*perimeter, True)

            # Check if area is within bounds and contour has 4 vertices
            if ((size < self.CARD_AREA_MAX) and (size > self.CARD_AREA_MIN) and (len(vertices) == 4)):
                # Append to new list
                card_contours.append(i)

        # Reprort number of cards found
        print(len(card_contours), "cards found.")

        # Return contours of cards
        return card_contours

    # Function to scale image
    def scale_img(self, src_img, scale):

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

    # 
    def detect_cards(self, src_img):

        # 
        img_copy1 = self.preprocess(src_img)
        # 
        contours = self.find_cards(img_copy1)
        #
        img_copy2 = cv.drawContours(src_img, contours, -1, (255,0,0), 10)
        
        cv.imshow("image3", self.scale_img(img_copy2, 50))

        cv.waitKey(0)

        
