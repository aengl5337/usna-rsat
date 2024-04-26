# -*- coding: utf-8 -*-
"""
Created on Fri Aug 25 11:52:23 2023

Credit: https://stackoverflow.com/questions/57983431/whats-the-most-space-efficient-way-to-compress-serialized-python-data

Is this a good test of image data?

() -Load ratios and time file and only change if file is changed, and only if file is THE SAME FILE!!!!!!!!!!
() +how to implement open -w finctionality across all compression methods?  (use methods in hdSaveImagesExample.py)
() +Implement zpaq
() -fix w_brootli 
() - any reason for compr() class?  just a glorified dict at this point

@author: Alec
"""

import os
import time

import pickle
import json # for saving results only
import configparser # This module is built-in

import bz2
import gzip
import lzma
import brotli
import blosc
# import zpaq

# from hdSaveImagesExample import *



class SomeObject():

    a = 'some data'
    b = 123
    c = 'more data'

    def __init__(self, i):
        self.i = i

def w_pickle(data, fname):
    with open(fname, 'wb') as f:
        pickle.dump(data, f)

def w_gzip(data, fname):
    with gzip.open(fname, "wb") as f:
        pickle.dump(data, f)

def w_bz2(data, fname):
    with bz2.bz2(fname, 'wb') as f:
        pickle.dump(data, f)
def w_lzma(data, fname):
    with lzma.open(fname, "wb") as f:
        pickle.dump(data, f)
# def w_brotli(data, fname):
#     # pfname = 
#     # bfname = 
#     with open(pfname, 'rb') as f:
#         pdata = f.read()
#         with open(bfname, 'wb') as b:
#             b.write(brotli.compress(pdata))
def w_blosc(data, fname):
    pickled_data = pickle.dumps(data)  # returns data as a bytes object
    compressed_pickle = blosc.compress(pickled_data)
    
    with open(fname, 'wb') as f:
        f.write(compressed_pickle)

# def w_(data, fname):
# def w_(data, fname):
# def w_(data, fname):
# def w_(data, fname):
# def w_(data, fname):
# def w_(data, fname):

class compr():
    names = ['p',      'gz2', 'bz2',  'lzma','brotli','blosc']
    exts =  ['.pickle','.gz', '.pbz2','.xz', '.bt',   '.dat']
    meths = [w_pickle, w_gzip,w_bz2   w_lzma,w_brotli,w_blosc]
    
    def __init__(self,name):
        assert name in names, 'Method not recognized'
        
        ind = names.index(name)
        self.name = name
        self.ext = exts[ind]
    
    def save(self, data, fname = -1):
        t0 =time.time()
        
        t1 =time.time()
        times[key][self.name] = t1-t0
        if not os.path.isfile(pfname):
        pfsize = os.path.getsize(pfname) # in bytes
        
        ratios[key]['gzip'] = os.path.getsize(gzipfname)/pfsize
        ratios[key]['lzma'] = os.path.getsize(lzmafname)/pfsize                

RMEAS = True
TMEAS = True

rndData = [SomeObject(i) for i in range(1, 1000000)]

datas = {'rnd':rndData}

wdir = 'comprtest'
os.chdir('comprtest')

pfname = 'no_compression.pickle'
gzipfname = 'gzip_test.gz'
bz2fname = 'bz2_test.pbz2'
lzmafname = 'lzma_test.xz'
brotlifname = 'brotli_test.bt'
bloscfname = 'blosc_test.dat'

ratios = {'rnd':{'p':1}}
times = {'rnd':{'p':1}}

for key in datas:
    data = datas[key]
    
    os.chdir(key) # Change directory based on the file used    

if RMEAS: 
    with open('ratios.json','w') as f:
        json.dump(ratios,f)
if TMEAS:
    with open('times.json','w') as f:
        json.dump(times,f)

# x = zpaq.compress(b'test')
# y = zpaq.compress('test') # outputs same as b'test'
# print(zpaq.decompress(x))