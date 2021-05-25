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
from card_img import Card_Img
from train_img import Train_Img

# Card detector class
class Card_Detector:
    
    # Constructor
    def __init__(self):
        
        # Set value for threshold filter
        self.THRESH = 120

        # Set maximum and minimum card area
        self.CARD_AREA_MAX = 1000000
        self.CARD_AREA_MIN = 1000

        # 
        self.training_ranks = []
        self.training_suits = []

        self.load_train_imgs()

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

    def detect_cards(self, src_img):

        # 
        img2 = self.preprocess(src_img)
        # 
        contours = self.find_cards(img2)
        #

        if len(contours) != 0:


            cards_matched = []

            for cnt in contours:

                cards_matched.append(self.preprocess_card(cnt, src_img))

            for card in cards_matched:

                self.match_card(card)
                print(card.best_rank_match, card.best_suit_match)
        
        

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

    # 
    def load_train_imgs(self):
        folder_path = "Train_Imgs"

        train_ranks = []
        train_suits = []

        rank_list = ["Ace", "Two", "Three", "Four", "Five", "Six", "Seven", 
                     "Eight", "Nine", "Ten", "Jack", "Queen", "King"]
        suit_list = ["Spades", "Diamonds", "Clubs", "Hearts"]
        
        for Rank in rank_list:
            training_img = Train_Img(Rank)
            file_path = os.path.join(folder_path, Rank + ".jpg")
            training_img.img = cv.imread(file_path, cv.IMREAD_GRAYSCALE)
            train_ranks.append(training_img)

        for Suit in suit_list:
            training_img = Train_Img(Suit)
            file_path = os.path.join(folder_path, Suit + ".jpg")
            training_img.img = cv.imread(file_path, cv.IMREAD_GRAYSCALE)
            train_suits.append(training_img)

        # 
        self.training_ranks = train_ranks
        self.training_suits = train_suits

        print("Trained")


    # 
    def match_card(self, card_img):

        RANK_DIFF_MAX = 2000
        SUIT_DIFF_MAX = 700

        best_rank_match_diff = 10000
        best_suit_match_diff = 10000
        best_rank_match_name = "Unknown"
        best_suit_match_name = "Unknown"

        
        cv.imshow("image", card_img.rank_img)
        cv.waitKey(0)

        # i = 0

        # If no contours were found in query card in preprocess_card function,
        # the img size is zero, so skip the differencing process
        # (card will be left as Unknown)
        if (len(card_img.rank_img) != 0) and (len(card_img.suit_img) != 0):

            
            # Difference the query card rank image from each of the train rank images,
            # and store the result with the least difference
            for Trank in self.training_ranks:

                    diff_img = cv.absdiff(card_img.rank_img, Trank.img)
                    rank_diff = int(np.sum(diff_img)/255)
                    
                    if rank_diff < best_rank_match_diff:
                        # best_rank_diff_img = diff_img
                        best_rank_match_diff = rank_diff
                        best_rank_name = Trank.name

            # Same process with suit images
            for Tsuit in self.training_suits:
                    
                    diff_img = cv.absdiff(card_img.suit_img, Tsuit.img)
                    suit_diff = int(np.sum(diff_img)/255)
                    
                    if suit_diff < best_suit_match_diff:
                        # best_suit_diff_img = diff_img
                        best_suit_match_diff = suit_diff
                        best_suit_name = Tsuit.name

        # Combine best rank match and best suit match to get query card's identity.
        # If the best matches have too high of a difference value, card identity
        # is still Unknown
        if (best_rank_match_diff < RANK_DIFF_MAX):
            best_rank_match_name = best_rank_name

        if (best_suit_match_diff < SUIT_DIFF_MAX):
            best_suit_match_name = best_suit_name

        card_img.best_rank_match = best_rank_match_name
        card_img.best_suit_match = best_suit_match_name
        card_img.rank_diff = best_rank_match_diff
        card_img.suit_diff = best_suit_match_diff

    # 
    def preprocess_card(self, contour, src_img):

        # 
        card = Card_Img()
        # 
        card.contour = contour

        perimeter = cv.arcLength(contour, True)
        approx = cv.approxPolyDP(contour, 0.01*perimeter, True)
        pts = np.float32(approx)
        card.corners = pts

        # Find width and height of card's bounding rectangle
        _, _, w, h = cv.boundingRect(contour)
        card.width = w
        card.height = h

        # Find center point of card by taking x and y average of the four corners.
        average = np.sum(pts, axis=0)/len(pts)
        cent_x = int(average[0][0])
        cent_y = int(average[0][1])
        card.center = [cent_x, cent_y]

        # Warp card into 200x300 flattened image using perspective transform
        card.warp = self.img_flattener(src_img, pts, w, h)

        # Adaptive threshold levels
        CARD_THRESH = 30
        # Width and height of card corner, where rank and suit are
        CORNER_WIDTH = 32
        CORNER_HEIGHT = 84
        # Dimensions of rank train images
        RANK_WIDTH = 70
        RANK_HEIGHT = 125
        # Dimensions of suit train images
        SUIT_WIDTH = 70
        SUIT_HEIGHT = 100

        # Grab corner of warped card image and do a 4x zoom
        img_corner = card.warp[0:CORNER_HEIGHT, 0:CORNER_WIDTH]
        img_corner_zoom = cv.resize(img_corner, (0,0), fx=4, fy=4)

        # Sample known white pixel intensity to determine good threshold level
        white_level = img_corner_zoom[15,int((CORNER_WIDTH*4)/2)]
        thresh_level = white_level - CARD_THRESH
        
        if (thresh_level <= 0):
            thresh_level = 1
        
        _, query_thresh = cv.threshold(img_corner_zoom, thresh_level, 255, cv.THRESH_BINARY_INV)
        
        # Split in to top and bottom half (top shows rank, bottom shows suit)
        rank = query_thresh[20:185, 0:128]
        suit = query_thresh[186:336, 0:128]

        # Find rank contour and bounding rectangle, isolate and find largest contour
        dummy, rank_cnts, _ = cv.findContours(rank, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        rank_cnts = sorted(rank_cnts, key=cv.contourArea, reverse=True)

        # Find bounding rectangle for largest contour, use it to resize query rank
        # image to match dimensions of the train rank image
        if len(rank_cnts) != 0:
            x1,y1,w1,h1 = cv.boundingRect(rank_cnts[0])
            Qrank_roi = rank[y1:y1+h1, x1:x1+w1]
            Qrank_sized = cv.resize(Qrank_roi, (RANK_WIDTH,RANK_HEIGHT), 0, 0)
            card.rank_img = Qrank_sized

        # Find suit contour and bounding rectangle, isolate and find largest contour
        dummy, suit_cnts, _ = cv.findContours(suit, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        suit_cnts = sorted(suit_cnts, key=cv.contourArea, reverse=True)
        
        # Find bounding rectangle for largest contour, use it to resize query suit
        # image to match dimensions of the train suit image
        if len(suit_cnts) != 0:
            x2,y2,w2,h2 = cv.boundingRect(suit_cnts[0])
            Qsuit_roi = suit[y2:y2+h2, x2:x2+w2]
            Qsuit_sized = cv.resize(Qsuit_roi, (SUIT_WIDTH, SUIT_HEIGHT), 0, 0)
            card.suit_img = Qsuit_sized

        return card

    # 
    def img_flattener(self, image, pts, w, h):

        # 
        temp_rect = np.zeros((4,2), dtype = "float32")
        # 
        s = np.sum(pts, axis = 2)
        # 
        tl = pts[np.argmin(s)]
        br = pts[np.argmax(s)]
        # 
        diff = np.diff(pts, axis = -1)
        tr = pts[np.argmin(diff)]
        bl = pts[np.argmax(diff)]

        # Need to create an array listing points in order of
        # [top left, top right, bottom right, bottom left]
        # before doing the perspective transform

        if w <= 0.8*h: # If card is vertically oriented
            temp_rect[0] = tl
            temp_rect[1] = tr
            temp_rect[2] = br
            temp_rect[3] = bl

        if w >= 1.2*h: # If card is horizontally oriented
            temp_rect[0] = bl
            temp_rect[1] = tl
            temp_rect[2] = tr
            temp_rect[3] = br

        # If the card is 'diamond' oriented, a different algorithm
        # has to be used to identify which point is top left, top right
        # bottom left, and bottom right.
        
        if w > 0.8*h and w < 1.2*h: #If card is diamond oriented
            # If furthest left point is higher than furthest right point,
            # card is tilted to the left.
            if pts[1][0][1] <= pts[3][0][1]:
                # If card is titled to the left, approxPolyDP returns points
                # in this order: top right, top left, bottom left, bottom right
                temp_rect[0] = pts[1][0] # Top left
                temp_rect[1] = pts[0][0] # Top right
                temp_rect[2] = pts[3][0] # Bottom right
                temp_rect[3] = pts[2][0] # Bottom left

            # If furthest left point is lower than furthest right point,
            # card is tilted to the right
            if pts[1][0][1] > pts[3][0][1]:
                # If card is titled to the right, approxPolyDP returns points
                # in this order: top left, bottom left, bottom right, top right
                temp_rect[0] = pts[0][0] # Top left
                temp_rect[1] = pts[3][0] # Top right
                temp_rect[2] = pts[2][0] # Bottom right
                temp_rect[3] = pts[1][0] # Bottom left
                
        maxWidth = 200
        maxHeight = 300

        # Create destination array, calculate perspective transform matrix,
        # and warp card image
        dst = np.array([[0,0],[maxWidth-1,0],[maxWidth-1,maxHeight-1],[0, maxHeight-1]], np.float32)
        M = cv.getPerspectiveTransform(temp_rect,dst)
        warp = cv.warpPerspective(image, M, (maxWidth, maxHeight))
        warp = cv.cvtColor(warp,cv.COLOR_BGR2GRAY)

        return warp




        
