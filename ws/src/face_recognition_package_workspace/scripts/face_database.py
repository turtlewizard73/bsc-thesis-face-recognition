#!/usr/bin/env python
from __future__ import print_function

# Ros modules
import rospy
from std_msgs.msg import String
from vision_recognition.msg import Encodings
from vision_recognition.srv import DataFrame

# Common modules
import sys
import os
from uuid import uuid4
import logging

# Main modules
from data_interface import DataInterface
from database_manager import DatabaseManager
import numpy as np
import pandas as pd
import face_recognition as fr


PKG_DIR = os.path.dirname(os.path.abspath(os.path.dirname(__file__)))
KNOWN_FACES_DIR = 'images/known_faces'
UNKNOWN_FACES_DIR = 'images/unknown_faces'

MAIN_DB='database_beta.pkl'
START_DB='start_database.pkl'

class FaceDatabase():
    COLUMNS = ['Name', 'Id', 'Folder','Filename','Matchcounter','Face']
    def __init__(self):
        super().__init__()
        self.datainterface = DataInterface()

        self.start_db_is_live = False
        self.main_df_is_live = False

        self.start_df = DatabaseManager(db_name='start_database', startdb=True)
        self.start_df_is_live = self.start_df.is_live
        self.start_df.sprint()

        self.main_df = DatabaseManager(db_name='main_database_alpha', )
        self.main_df.add_db(self.start_df)
        self.main_df_is_live = self.main_df.is_live
        self.main_df.sprint()

        self.tolerance = 0.6
        self.MAX_ENC_PER_FACE  = 10
        self.MIN_ENC_PER_FACE  = 5
        self.queriedencodings_pub = rospy.Publisher('/face/queriedencodings', Encodings, queue_size = 10)
        self.database_server = rospy.Service('get_db', DataFrame, self.handle_get_db)

        self.live_df = self.main_df
        print('Live database:')
        self.live_df.sprint(10)

        self.encoding_sub = rospy.Subscriber('/face/encodings', Encodings, self.callback)

    def callback(self, data):
        # self.live_df = self.live_df.sort_values(by='Name', ignore_index=True)
        # self.live_df.drop_duplicates(subset=['Id'], keep='last', ignore_index=True)
        self.live_df.sort_df()

        names = []
        encodings = []
        if data.faces == 0:
            return

        detected_encodings, detected_names, detected_uids = self.datainterface.msg2enc(data)
        known_face_encodings = self.live_df.get_encodings()

        query_names = []

        similar_encodings = []
        similar_names = []
        similar_uids = []

        for uid, name, encoding in zip(detected_uids, detected_names, detected_encodings):
            n_name = name
            n_uid = uid

            if name == 'Unknown':
                matches = fr.compare_faces(known_face_encodings, encoding, self.tolerance)
                if True in matches:
                    print('Found match in database')
                    face_distances = fr.face_distance(known_face_encodings, encoding)
                    best_match_index = np.argmin(face_distances)

                    # if the closeset face is a True match
                    if matches[best_match_index]:
                        n_name = self.live_df.get_name(best_match_index)
                        n_uid = self.live_df.get_uid(best_match_index)

                        similar_encodings.append(encoding)
                        similar_names.append(n_name)
                        similar_uids.append(n_uid) # we need the id of the corresponding encoding which matched

                else:
                    n_name, n_uid = self.live_df.handle_new_encoding(encoding=encoding)

                query_names.append(n_name)
            else:
                similar_encodings.append(encoding)
                similar_names.append(n_name)
                similar_uids.append(n_uid)


        known_face_encodings = self.live_df.get_encodings()
        for uid, name, encoding in zip(similar_uids, similar_names, similar_encodings):
            # add similar encoding to db
            self.live_df.handle_new_encoding(encoding=encoding, name=name)

            # sort db
            self.live_df.sort_df()

            # uid contains information about which previous encoding was the best match
            # increase its match counter by one
            if self.live_df.inc_match_counter(uid) == False:
                print('Face encoding no longer in the database')
                # query_names.append(name)
            # checking if the encodings for this people are more than needed
            # dups = self.live_df.df.pivot_table(index = ['Name'], aggfunc ='size')
            # entries = dups[name]
            my_entries = self.live_df.get_rows(attr='Name', value=name)
            if len(my_entries) > self.MAX_ENC_PER_FACE:
                self.handle_sort_one_face(name, my_entries)
                pass

        print('Current DF _________________________')
        self.live_df.sprint(40)


        if len(query_names) > 0:
            query_names = [*set(query_names)]

            queried_encodings = []
            queried_names = []
            queried_uids = []
            for name in query_names:
                # get all indicies
                query = self.live_df.get_rows(attr='Name', value=name)
                query.sort_values(by='Name', ignore_index=True)
                # get match counter and selecting the id of the max
                max_mc_id = query['Matchcounter'].argmax()
                q_enc = query['Face'].loc[query.index[max_mc_id]]
                q_uid = query['Id'].loc[query.index[max_mc_id]]

                queried_encodings.append(q_enc)
                queried_names.append(name)
                queried_uids.append(q_uid)

            try:
                self.queriedencodings_pub.publish(self.datainterface.enc2msg(queried_encodings, queried_names, queried_uids))
            except Exception as e:
                print(e)

    def optimize_df(self):
        pass

    def handle_get_db(self, req):
        print(f'Sending {req.cmd} dataframe... ')
        df = self.main_df.df
        live = self.main_df_is_live
        if req.cmd == 'Start':
            df = self.start_df.df
            live = self.start_df_is_live

        if live:
            encodings = [np.asarray(a=f, dtype=np.float64) for f in df['Face'].tolist()]
            names = df['Name'].tolist()
            uids = df['Id'].tolist()
        else:
            encodings = []
            names = []
            uids = []
        answer = self.datainterface.enc2srv(live, encodings, names, uids)
        return answer

    def handle_sort_one_face(self, name, my_entries):
        print(f'Sorting encodings for: {name}')
        my_encodings = [np.asarray(a=f, dtype=np.float64) for f in my_entries['Face'].tolist()]
        my_uids = my_entries['Id'].tolist()
        my_match_counts = my_entries['Matchcounter']

        # getting the most accurate encoding as a base to calculate distances from
        max_mc_index = my_match_counts.argmax()
        base_encoding = my_encodings[max_mc_index]

        my_distances = fr.face_distance(base_encoding, my_encodings)
        # selecting the 'k' farthest distances' indecies to drop
        k = len(my_entries) - self.MIN_ENC_PER_FACE
        # list of indices if the 'farthest' encodings
        farthest_distances_ids = np.argpartition(my_distances, k)[-k:]

        for i in farthest_distances_ids:
            uid =  my_uids[i]
            self.live_df.drop_rows('Id', uid)
            print(f"Dropped encoding's uid: {uid}")

        return

    def save_live_db(self):
        self.live_df.save_db()


def main(args):
    print('Starting DataBase NODE')
    rospy.init_node('face_database', anonymous=False)
    fdb = FaceDatabase()
    rate = rospy.Rate(10)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    fdb.save_live_db()

if __name__ == "__main__":
    main(sys.argv)