import numpy as np
import pptk
import pandas as pd
import sys
import os
from random import gauss

def read_points(f):
    # reads Semantic3D .txt file f into a pandas dataframe
    col_names = ['time','intensity','id','x', 'y', 'z', 'azimuth','range','pid']
    col_dtype = {'x': np.float32, 'y': np.float32, 'z': np.float32, 'i': np.int32}
    return pd.read_csv(f, header=0, usecols=[3,4,5],dtype=np.float32)

def add_noise(pc):
   copy_pc = pc.copy()
   for row  in range(0,len(pc.index)):
      for col in range(0,len(pc.columns)):
         copy_pc.iloc[row,col] = copy_pc.iloc[row,col] + gauss(0, 0.1)  
   return copy_pc

file = str(os.path.abspath(sys.argv[1]))
print("reading file: " + file)

points = read_points(file)
noise_points = add_noise(points)

# combine two pointcoulds
combined = pd.concat([points, noise_points])
v_c = pptk.viewer(combined) 
v_c.set(point_size=0.01)

# add colors
color_1 = [1,1,1]
color_2 = [0.11982556, 0.80928971, 0.082647]
colors = [color_1] * (len(combined.index)/2) + [color_2] * (len(combined.index)/2) 
v_c.attributes(colors)
