#!/usr/bin/python

import sys
import pickle

name = str(sys.argv[-1])

with open('rect_list_'+name+'.data', 'rb') as filehandle:
    # read the data as binary data stream
    rect_list = pickle.load(filehandle)

print(len(rect_list))
