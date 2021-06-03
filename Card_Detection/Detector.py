"""
MTRX5700 - Experimental Robotics
Major Project - Blackjack Robot
Year: 2021
Group 5 - Curry Shop

File: detector.py
Info: Class definition for Card Detector.
"""

# Imports
import os
import cv2 as cv
import numpy as np

# Card Detector class
class Detector:
    
    # Constructor
    def __init__(self):
        
        # Set maximum and minimum card area
        self.CARD_AREA_MAX = 1000000
        self.CARD_AREA_MIN = 80
        
        # Initialise Blob Detector parameters
        params = cv.SimpleBlobDetector_Params()
        # Change thresholds
        params.minThreshold = 10
        params.maxThreshold = 200
        # Filter by Area.
        params.filterByArea = True
        params.minArea = 500
        # Filter by Convexity
        params.filterByConvexity = True
        params.minConvexity = 0.40
        # Filter by Inertia
        # params.filterByInertia = True
        # params.minInertiaRatio = 0.01
        # Create Blob detector with parameters
        self.blob_detector = cv.SimpleBlobDetector_create(params)

    # Preprocess image
    def preprocess(self, image, thresh):
        
        # Convert to grayscale
        gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
        # Apply blur
        blur = cv.GaussianBlur(gray, (5,5), 0)

        # Filter with adaptive threshold
        _, thresh_img = cv.threshold(blur, thresh, 255, cv.THRESH_BINARY)

        # Return thresholded image
        return thresh_img

    # Find cards by examining contours in image
    def find_cards(self, img):

        # Extract contours
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
    def extract_cards(self, img, contours, idx):

        img_copy = img

        mask = np.zeros_like(img)
        cv.drawContours(mask, contours, idx, 255, -1)
        out = np.zeros_like(img)
        out[mask==255] = img[mask==255]

        (y, x, _) = np.where(mask == 255)
        (topy, topx) = (np.min(y), np.min(x))
        (bottomy, bottomx) = (np.max(y), np.max(x))
        card = img_copy[topy:bottomy+1, topx:bottomx+1]

        return card

    # 
    def blob_detection(self, img):

        # img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        # 
        keypoints = self.blob_detector.detect(img)
        # 
        return keypoints

    # 
    def find_contour_centroid(self, cnt):
        
        # Computer centre
        M = cv.moments(cnt)
        x_c = int(M["m10"] / M["m00"])
        y_c = int(M["m01"] / M["m00"])

        return (x_c, y_c)

    # Runs the helper functions to detect and extract the cards
    def detect_cards(self, src_img):

        thresh = 160

        img_copy = src_img.copy()
        # Perform image preprocessing
        img1 = self.preprocess(img_copy, thresh)
        # Detect the contours of cards
        contours = self.find_cards(img1)
        
        # Draw contours
        cv.drawContours(img_copy, contours, -1, (255,0,0), 10)
        # Show annotated image
        cv.imshow("contours", self.scale_img(img_copy, 50))
        cv.waitKey(0)

        # 
        for i in range(len(contours)):

            # Find centroid of contour
            cnt_centroid = self.find_contour_centroid(contours[i])
            # Draw centroid 
            cv.circle(img_copy, cnt_centroid, 15, (0, 0, 255), -1)
            cv.imshow("contours", self.scale_img(img_copy, 50))
            
            # Extract a single card from contours
            extracted_card = self.extract_cards(src_img, contours, i)
            # Scale image
            extracted_card = self.scale_img(extracted_card, 300)
            # Preprocess iamge
            img2 = self.preprocess(extracted_card, thresh)
            
            # Perform blob detection
            blobs = self.blob_detection(img2)
            # Print number of blobs detected
            print(len(blobs))

            # Draw blobs
            blob_img = cv.drawKeypoints(extracted_card.copy(), blobs, np.array([]), (0,0,255), cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

            # Refresh windows
            cv.destroyWindow("card")
            cv.destroyWindow("blobs")
            # Show images
            cv.imshow("card", img2)
            cv.imshow("blobs", blob_img)
            # Wait for keystroke
            cv.waitKey(0)




        
