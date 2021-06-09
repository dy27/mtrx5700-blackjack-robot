#! /usr/bin/env python

import rospy
import numpy as np
from blackjack_dealer_robot.msg import CardDetection, CardDetectionArray
import cv2
from cnn.card_cnn import CardCNN
from cnn.cnn_training import custom_transform
import torch
from PIL import Image


class ImageProcessor:
    def __init__(self):
        # Initialise the ROS node
        rospy.init_node('image_processor')

        # Card detection publisher
        self.pub_game_status = rospy.Publisher("/card_detections", CardDetectionArray, queue_size=10)

        # Video capture stream from IP address
        self.video_capture = cv2.VideoCapture('http://10.19.106.211:8080/videofeed')

        # CNN binary classifier to predict whether a card is a royal or number
        self.card_classifier = CardCNN()
        if torch.cuda.is_available():
            self.card_classifier.cuda()
        self.card_classifier.load_state_dict(torch.load("model"))
        self.cnn_transform = custom_transform

        # Infinite processing loop
        self.process_image_loop()

        
    def process_image_loop(self):
        while True:
            _, frame = self.video_capture.read()

            card_images, card_coords = self.find_cards(frame)

            
            rospy.sleep(0.2)


    def find_cards(self, frame):
        pass


    def classify_card_value(self, card_image):
        if self.is_royal_card(card_image):
            return 10
        else:
            num_blobs = self.detect_blobs(card_image)
            return num_blobs


    def detect_blobs(self):
        pass


    def is_royal_card(self, card_image):
        with torch.no_grad():
            img = np.asarray(card_image)
            img = Image.fromarray(img)
            img = self.cnn_transform(img)
            img = torch.unsqueeze(img, 0)
            pred = self.cnn_model(img)
            pred_class = np.argmax(pred)

        return False if pred_class.numpy() == 0 else True


if __name__ == '__main__':
    ImageProcessor()