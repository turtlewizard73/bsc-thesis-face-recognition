#!/usr/bin/env python3
from __future__ import print_function
import cv2
import face_recognition as fr


class ImageProcessor(object):
    FRAME_THICKNESS = 3 #pixel
    FONT_THICKNESS = 2
    COLOR_KNOWN = [0, 200, 0]
    COLOR_SEEN = [200, 200, 0]
    COLOR_UNKNOWN = [0, 75, 250]
    def __init__(self, model='hog', tolerance=0.6, factor=4):
        self.model = model
        self.tolerance = tolerance
        self.factor = factor

    def process_frame(self, frame):
        '''
        Input: frame
        Processes the image and gets the faces' encodings and locations
        Output: location, encoding
        '''
        f = 1/self.factor # scales the original frame with the factor of f
        face_locations = []
        face_encodings = []

        if frame is not None:
            small_frame = cv2.resize(frame, (0,0), fx=f, fy=f)

            # Convert the image from BGR color (which OpenCV uses) to RGB color (which face_recognition uses)
            # rgb_small_frame = small_frame[:, :, ::-1]
            rgb_small_frame = cv2.cvtColor(small_frame, cv2.COLOR_BGR2RGB)

            face_locations = fr.face_locations(rgb_small_frame)
            face_encodings = fr.face_encodings(rgb_small_frame, face_locations)

        return face_locations, face_encodings

    def display_locations(self, frame, locations, names):
        '''
        Input: frame
        Draws the boxes and names on the frame
        Output: frame with boxes and names
        '''
        for (top, right, bottom, left), name in zip(locations, names):
            top *= self.factor
            right *= self.factor
            bottom *= self.factor
            left *= self.factor

            color = self.COLOR_UNKNOWN
            if name.startswith('Unknown_'):
                color = self.COLOR_SEEN
            elif not name.startswith('Unknown'):
                color = self.COLOR_KNOWN
            # Draw a box around the face
            cv2.rectangle(frame, (left,top), (right, bottom), color, self.FRAME_THICKNESS)

            # Draw a label with a name below the face
            cv2.rectangle(frame, (left, bottom - 35), (right, bottom), color, cv2.FILLED)
            cv2.putText(
                img=frame,
                text=name,
                org=(left + 6, bottom - 6),
                fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                fontScale=1.0,
                color=(255, 255, 255),
                thickness=self.FONT_THICKNESS)
        return frame