#!/usr/bin/env python3
from __future__ import print_function

import os
from uuid import uuid4
import numpy as np
import pandas as pd
import face_recognition as fr
from data_interface import DataInterface

class DatabaseManager(object):
    PKG_DIR = os.path.dirname(os.path.abspath(os.path.dirname(__file__)))
    DATA_DIR = f'{PKG_DIR}/data'
    KNOWN_FACES_DIR = 'images/known_faces'
    UNKNOWN_FACES_DIR = 'images/unknown_faces'
    COLUMNS = ['Name', 'Id', 'Folder','Filename','Matchcounter','Face']

    def __init__(self, db_name:str, startdb:bool=False):
        self.datainterface = DataInterface()
        self.upload_path = f'{self.PKG_DIR}/{self.KNOWN_FACES_DIR}' if startdb \
            else f'{self.PKG_DIR}/{self.UNKNOWN_FACES_DIR}'
        self.db_name = f'{db_name}.pkl'
        self.db_path = f'{self.DATA_DIR}/{self.db_name}'
        self.df = self.init_df()
        self.sort_df()
        self.is_live

    def init_df(self):
        df = pd.DataFrame(columns=self.COLUMNS)
        if self.db_name in os.listdir(self.DATA_DIR):
            print(f'{self.db_name} Database found')
            print('Loading database...')
            # self.df = pd.concat((chunk for chunk in pd.read_csv(filepath_or_buffer=DB_PATH, header=0, sep = ",", chunksize = 200)))
            df = pd.read_pickle(self.db_path )
        else:
            print(f'{self.db_name} Database not found')
            print('Creating database...')

            names, ids, folders, filenames, encodings = self.load_known_faces()
            encodings = np.asarray(encodings)
            names = np.asarray(names)

            encodings_list = [e.tolist() for e in encodings]

            df = pd.DataFrame({
                'Name': names,
                'Id': ids,
                'Folder': folders,
                'Filename': filenames,
                'Matchcounter': [0]*len(names),
                'Face': encodings_list})
            df.to_pickle(self.db_path )
        self.is_live = True
        return df

    def load_known_faces(self):
        print('Loading known faces...')
        known_names = []
        ids = []
        folders = []
        filenames = []
        known_face_encodings = []


        for name in os.listdir(self.upload_path):
            # iterating through pictures of people by name
            for filename in os.listdir(f'{self.upload_path}/{name}'):
                image = fr.load_image_file(f'{self.upload_path}/{name}/{filename}')
                encoding = fr.face_encodings(image)[0] #first face to find (works on images with one face)

                known_names.append(name)
                ids.append(str(uuid4()))
                folders.append(name)
                filenames.append(filename)
                known_face_encodings.append(encoding)

        return known_names, ids, folders, filenames, known_face_encodings

    def add_db(self, df):
        self.df = pd.concat([df.df, self.df], ignore_index=True)
        self.df.drop_duplicates(subset=['Id'], inplace=True, keep='last', ignore_index=True)

    def sprint(self, i=5):
        print(self.df.head(i))

    def sort_df(self):
        self.df = self.df.sort_values(by='Name', ignore_index=True)
        self.df.drop_duplicates(subset=['Id'], inplace=True, keep='last', ignore_index=True)

    def inc_match_counter(self, n_uid, index=None):
        success = True
        if index is None:
            success, index = self.get_id_by_uid(n_uid)

        if not success:
            print(f'The entry has been deleted from databse')
            return False

        print('MATCH FOUND at index: ', index)
        self.df.at[index, 'Matchcounter'] = self.df.loc[self.df['Id'] == n_uid]['Matchcounter'] + 1

    def get_id_by_uid(self, uid):
        print(self.df.index[self.df['Id']==uid])
        index = self.df.index[self.df['Id']==uid].tolist()
        if len(index) == 0:
            return False, 0
        return True, index[0]

    def handle_new_encoding(self, encoding, name=None):
        n_name = name
        index = self.new_index()
        if name == None:
            print('Adding new person to database')
            n_name  = f'Unknown_{index}'

        n_uid = str(uuid4())
        tmp_df = self.datainterface.enc2df(n_name, n_uid, encoding)
        self.df = pd.concat([self.df, tmp_df], ignore_index=True)

        return n_name, n_uid

    def new_index(self):
        index = self.df.last_valid_index() + 1
        return index


    def save_db(self):
        print(f'Saving database to file: {self.db_path}')
        self.df.to_pickle(self.db_path)

    def get_encodings(self):
        encodings = [np.asarray(a=f, dtype=np.float64) for f in self.df['Face'].tolist()]
        return encodings

    def get_rows(self, attr, value):
        rows = self.df.loc[self.df[attr] == value]
        return rows

    def drop_rows(self, attr:str, value):
        rows = self.df.index[self.df[attr] == value]
        self.df = self.df.drop(rows)

    def get_name(self, index):
        name = self.df['Name'].loc[self.df.index[index]]
        return name

    def get_uid(self, index):
        uid = self.df['Id'].loc[self.df.index[index]]
        return uid

    def get_id(self, index):
        uid = self.df['Id'].loc[self.df.index[index]]
        return uid

    def get_enc(self, index):
        enc = self.df['Face'].loc[self.df.index[index]]
        return enc

    def get_mc(self, index):
        mc = self.df['Face'].loc[self.df.index[index]]
        return mc

    