#!/usr/bin/env python3
from __future__ import print_function

# ROS modules
import roslib
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from vision_recognition.msg import Encodings
from vision_recognition.srv import DataFrame
from cv_bridge import CvBridge, CvBridgeError

# Common modules
import sys
import os
import time
import cv2
import asyncio
from datetime import datetime
import logging

# Main modules
from data_interface import DataInterface
from image_processor import ImageProcessor
import face_recognition as fr
import numpy as np
import cv2


PKG_DIR = os.path.dirname(os.path.abspath(os.path.dirname(__file__)))
KNOWN_FACES_DIR = 'images/known_faces'

class Face_publisher:
    def __init__(self):
        self.imageProcessor = ImageProcessor()
        self.bridge = CvBridge()
        self.datainterface = DataInterface()
        self.tolerance = 0.6

        self.image_pub = rospy.Publisher('/image/detected_faces', Image, queue_size = 1)
        self.encoding_pub = rospy.Publisher('/face/encodings', Encodings, queue_size = 10)

        self.process_this_frame = False
        self.publish_face_timer = 0

        # location, encodings, names currently on image feed
        self.locations = []
        self.encodings = []
        self.names  = []
        self.uids = []

        # location, encodings, names in cache
        self.encodings_cache = []
        self.known_names_cache = []
        self.known_uids_cache = []

        # initialize cache with data from starting database
        while not self.get_db('Start'):
            # waits untill database is available
            time.sleep(5)

        self.image_sub = rospy.Subscriber('/image/webcam/raw', Image, self.callback)
        self.update_cache_sub = rospy.Subscriber('/face/queriedencodings', Encodings, self.update_cache_callback)


    def get_db(self, db:str):
        print(f'Requesting {db} database')
        db_loaded = False
        request = db
        rospy.wait_for_service('get_db')
        try:
            get_db = rospy.ServiceProxy('get_db', DataFrame)
            response = get_db(request)
            if response.db_is_live:
                self.encodings_cache, self.known_names_cache, self.known_uids_cache = \
                    self.datainterface.msg2enc(response)
                print('Starting database loaded')
                db_loaded = True
            else:
                db_loaded = False
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            db_loaded = False

        return db_loaded

    def callback(self,data):
        try:
            cv_frame = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            print(e)

        if self.process_this_frame:
            self.locations = []
            self.encodings = []
            self.locations, self.encodings = self.imageProcessor.process_frame(frame=cv_frame)

        self.process_this_frame = not self.process_this_frame

        if self.find_matches():
            cv_frame = self.imageProcessor.display_locations(
                frame=cv_frame,
                locations=self.locations,
                names=self.names
                )

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_frame, "bgr8"))
        except CvBridgeError as e:
            print(e)

        try:
            if len(self.locations) > 0:
                if self.publish_face_timer > 25:
                    self.publish_face_timer = 0
                    self.encoding_pub.publish(self.datainterface.enc2msg(self.encodings, self.names, self.uids))
                else:
                    self.publish_face_timer = self.publish_face_timer + 1
        except Exception as e:
            print(e)

    def find_matches(self):
        if len(self.locations) == 0:
            return False

        self.names = []
        self.uids = []
        for i, face_encoding in enumerate(self.encodings):
            # compares to all known faces
            # print(face_encoding)
            matches = fr.compare_faces(self.encodings_cache, face_encoding, self.tolerance)
            name = 'Unknown'
            uid = '0'

            face_distances = fr.face_distance(self.encodings_cache, face_encoding)
            best_match_index = np.argmin(face_distances)

            # if the closeset face is a True match
            if matches[best_match_index]:
                name = self.known_names_cache[best_match_index]
                uid = self.known_uids_cache[best_match_index]

            self.names.append(name)
            self.uids.append(uid)
        return True

    def update_cache_callback(self, data):
        encodings, names, uids = self.datainterface.msg2enc(data)

        self.encodings_cache = np.array(list(self.encodings_cache) + list(encodings))
        self.known_names_cache = np.array(list(self.known_names_cache) + list(names))
        self.known_uids_cache = np.array(list(self.known_uids_cache) + list(uids))

        print(self.known_names_cache)

def main(args):
    rospy.init_node('image_converter', anonymous=False)
    face_publisher = Face_publisher()
    rate = rospy.Rate(10)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
