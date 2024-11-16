#!/usr/bin/env python3
from __future__ import print_function

from vision_recognition.msg import Encodings
from vision_recognition.srv import DataFrameResponse

import numpy as np
import pandas as pd

class DataInterface(object):
    '''
    .enc : list(numpy.ndarray(numpy.float64))
        list dimension: number of faces detected
        ndarray dimension: 128*float64 face encoding
    .msg : list(enc1(128), enc2(128), enci(128) ... encn)
        n : number of faces detected
    '''

    def enc2msg(self, encodings, names, uids):
        msg = Encodings()
        faces = len(encodings)
        enc_dim = len(encodings[0])
        enc = np.asarray(encodings).reshape([enc_dim*faces])

        msg.faces = faces
        msg.encoding_dim = enc_dim
        msg.encodings = enc
        msg.names = names
        msg.uids = uids
        return msg

    def msg2enc(self, data):
        uids = np.asarray(data.uids)
        names = np.asarray(data.names)
        encodings =  np.asarray(data.encodings).reshape([data.faces, data.encoding_dim])
        return encodings, names, uids

    def enc2srv(self, is_live, encodings, names, uids):
        srv = DataFrameResponse()
        faces = len(encodings)
        enc_dim = 0
        enc = []
        if faces > 0:
            enc_dim = len(encodings[0])
            enc = np.asarray(encodings).reshape([enc_dim*faces])

        srv.db_is_live = is_live
        srv.faces = faces
        srv.encoding_dim = enc_dim
        srv.encodings = enc
        srv.names = names
        srv.uids = uids
        return srv

    def enc2df(self, name:str, uid, encoding, match_count=0):
        new_row = {
            'Name': name,
            'Id': uid,
            'Matchcounter': match_count,
            'Face': encoding
        }
        df = pd.DataFrame([new_row])
        return df


