#! /usr/bin/env python

from cnn import card_cnn
import rospy
import numpy as np
from blackjack_dealer_robot.msg import DetectCardsAction, DetectCardsResult, CardDetection
import cv2
from cnn.card_cnn import CardCNN
# from cnn.cnn_training import custom_transform
from torchvision import transforms
import torch
from PIL import Image
import sensor_msgs
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import actionlib


class ImageProcessor:
    def __init__(self):
        # Initialise the ROS node
        rospy.init_node('image_processor')

        self.img_pub = rospy.Publisher("/test_image", sensor_msgs.msg.Image, queue_size=10)
        self.bridge = CvBridge()

        # CNN binary classifier to predict whether a card is a royal or number
        self.card_classifier = CardCNN()
        # if torch.cuda.is_available():
        #     self.card_classifier.cuda()
        self.card_classifier.load_state_dict(torch.load("cnn/cnn_model"))
        self.cnn_transform = transforms.Compose([	
										transforms.Resize((150,150)),
										transforms.Grayscale(),
                                        transforms.ToTensor(),
                                        transforms.Normalize((0.5, ), (0.5, )),
                                       ])

        # Initialise blob detector parameters
        params = cv2.SimpleBlobDetector_Params()
        params.minThreshold = 100
        params.maxThreshold = 150
        params.filterByArea = True
        params.minArea = 700
        params.filterByConvexity = True
        params.minConvexity = 0.50
        self.blob_detector = cv2.SimpleBlobDetector_create(params)

        # Set maximum and minimum card area for card detection
        self.CARD_AREA_MAX = 1000000
        self.CARD_AREA_MIN = 80

        # Set up action server
        self.action_name = 'card_detection'
        self.action_server = actionlib.SimpleActionServer(self.action_name,
                                                        DetectCardsAction,
                                                        execute_cb=self.process_image_callback,
                                                        auto_start=False)
        self.action_server.start()

        
    def process_image_callback(self, msg):
        """Execution callback for the action server, which returns card detections."""

        video_capture = cv2.VideoCapture('http://10.19.106.211:8080/videofeed')
        video_capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        _, frame = video_capture.read()
        frame = frame[345:1059, 387:1462]

        result = self.detect_cards(frame)

        self.action_server.set_succeeded(result)

        return result


    def detect_cards(self, src_img):
        """Detects and classifies cards in an image."""

        if src_img is None:
            return

        thresh = 160
        img_copy = src_img.copy()
        # Perform image preprocessing
        img1 = self.preprocess(img_copy, thresh)
        # Detect the contours of cards
        contours = self.find_cards(img1)

        result = DetectCardsResult()

        for i in range(len(contours)):

            detection = CardDetection()

            # Find centroid of contour
            centroid = self.find_contour_centroid(contours[i])            
            
            # Extract a single card from contours
            extracted_card = self.extract_cards(src_img, contours, i)

            # Scale image
            extracted_card = self.scale_img(extracted_card, 300)

            # Preprocess image
            img2 = self.preprocess(extracted_card, thresh)
            
            card_val = self.classify_card_value(extracted_card)
            if card_val != -1:

                print("card details:", card_val, centroid)

                detection.card_point.x, detection.card_point.y, detection.card_point.z = (centroid[0])*(0.6/714), (centroid[1])*(0.9/1075), 0
                detection.value = card_val

                result.detections.append(detection)

        return result


    def preprocess(self, image, thresh):
        
        # Convert to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # Apply blur
        blur = cv2.GaussianBlur(gray, (5,5), 0)

        # Filter with adaptive threshold
        _, thresh_img = cv2.threshold(blur, thresh, 255, cv2.THRESH_BINARY)

        # Return thresholded image
        return thresh_img

    
    def find_cards(self, img):

        # Extract contours
        contours, _ = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # _, contours, _ = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # Sort contours
        contours = sorted(contours, key=cv2.contourArea, reverse=True)

        # Empty list of contours
        card_contours = []

        # Iterate through contour found
        for i in contours:
            
            # Get size of contour
            size = cv2.contourArea(i)
            # Get perimeter
            perimeter = cv2.arcLength(i, True)
            # Get vertices of contour
            vertices = cv2.approxPolyDP(i, 0.01*perimeter, True)


            # Check if area is within bounds and contour has 4 vertices
            if ((size < self.CARD_AREA_MAX) and (size > self.CARD_AREA_MIN) and (len(vertices) == 4)):

                # Append to new list
                card_contours.append(i)
        
        # Reprort number of cards found
        print(len(card_contours), "cards found.")

        # Return contours of cards
        return card_contours


    def extract_cards(self, img, contours, idx):
        """Extract image of a card from the camera frame image."""

        img_copy = img

        mask = np.zeros_like(img)
        cv2.drawContours(mask, contours, idx, 255, -1)
        out = np.zeros_like(img)
        out[mask==255] = img[mask==255]

        (y, x, _) = np.where(mask == 255)
        (topy, topx) = (np.min(y), np.min(x))
        (bottomy, bottomx) = (np.max(y), np.max(x))
        card = img_copy[topy:bottomy+1, topx:bottomx+1]

        return card


    def scale_img(self, src_img, scale):
        """Returns a scaled version of an image."""

        # Extract image dimensions
        height = src_img.shape[0]
        width = src_img.shape[1]
        
        # Scale dimensions
        new_height = int(height * scale/100)
        new_width = int(width * scale/100)
        
        # Resize image
        img = cv2.resize(src_img, (new_width, new_height), interpolation = cv2.INTER_AREA)
        
        # Return new image
        return img


    def blob_detection(self, img):
        """Executes blob detection on an image."""
        keypoints = self.blob_detector.detect(img)
        return keypoints


    def find_contour_centroid(self, cnt):
        """Finds the centroid of a contour"""
        M = cv2.moments(cnt)
        x_c = int(M["m10"] / M["m00"])
        y_c = int(M["m01"] / M["m00"])
        return (x_c, y_c)    


    def classify_card_value(self, extracted_card):
        """Returns the blackjack value of an extracted card image."""

        # lower_blue = np.array([100,50,50])
        # if self.check_face_down(extracted_card):
        #     return 0
        if self.is_royal_card(extracted_card):
            return 10
        else:
            img = self.preprocess(extracted_card, 160)
            num_blobs = len(self.blob_detection(img))
            if num_blobs == 1:
                return 11
            return num_blobs


    def is_royal_card(self, card_image):
        """Returns whether a extract card image is a royal card."""
        with torch.no_grad():
            img = np.asarray(card_image)
            img = Image.fromarray(img)
            img = self.cnn_transform(img)
            img = torch.unsqueeze(img, 0)
            pred = self.card_classifier(img)
            pred_class = np.argmax(pred)

        return False if pred_class.numpy() == 0 else True

    
    def check_face_down(self, img):
        """Returns whether an extracted card image is a face down card."""

        num_pixels = img.shape[0] * img.shape[1]

        face_down_flag = False

        hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # Check for Blue
        lower_blue = np.array([100, 50, 50])
        upper_blue = np.array([130, 255, 255])
        mask_blue = cv2.inRange(hsv_img, lower_blue, upper_blue)

        # Count masked pixels
        num_white_px = np.sum(img == 255)

        if num_white_px / num_pixels > 0.5:
            return True
        
        # if (num_white_px > 2000):
        #     # print(number_of_white_pix)
        #     face_down_flag = True
        # cv2.imshow("mask_blue", self.scale_img(mask_blue, 100))

        return False



if __name__ == '__main__':
    ImageProcessor()