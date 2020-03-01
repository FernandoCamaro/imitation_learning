import time
import math
from datetime import datetime
import struct
import sys
import os, fnmatch
import argparse
from collections import OrderedDict 

def readLogFile(filename, verbose=False):
  f = open(filename, 'rb')
  print('Opened')
  print(filename)

  keys = f.readline().decode('utf8').rstrip('\n').split(',')
  fmt = f.readline().decode('utf8').rstrip('\n')

  # The byte number of one record
  sz = struct.calcsize(fmt)

  # The type number of one record
  ncols = len(fmt)

  if verbose:

    print('Keys:'),
    print(keys)
    print('Format:'),
    print(fmt)
    print('Size:'),
    print(sz)
    print('Columns:'),
    print(ncols)

  # Read data
  wholeFile = f.read()

  # split by alignment word
  chunks = wholeFile.split(b'\xaa\xbb')

  log = OrderedDict() 

  if verbose:

    print("num chunks:")
    print(len(chunks))

  chunkIndex = 0

  for chunk in chunks:
    
    if verbose:
      print("len(chunk)=", len(chunk), " sz = ", sz)
    if len(chunk) == sz:
      if verbose:
        print("chunk #", chunkIndex)
      chunkIndex = chunkIndex + 1
      values = struct.unpack(fmt, chunk)
      step_count = values[0]
      time_stamp = values[1]
      object_id  = values[2]
      if step_count not in log.keys():
          log[step_count] = {}
          log[step_count]['time_stamp'] = time_stamp
      log[step_count][object_id] = {}
      record = {}

      for i in range(ncols):
        log[step_count][object_id][keys[i]] = values[i]
        if verbose:
          print("    ", keys[i], "=", values[i])

  return log


# fileName_gen = sys.argv[1]
# fileName_vr = sys.argv[2]
# log_gen = readLogFile(fileName_gen, verbose=False)
# log_vr = readLogFile(fileName_vr, verbose=False)
