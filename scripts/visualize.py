import numpy as np
import pptk
import pandas as pd
import sys
import os
from random import gauss

# read the pointcloud file
# must contain 'x,y,z' columns
def read_points(f, columns):
	col_dtype = {'x': np.float32, 'y': np.float32, 'z': np.float32}
	return pd.read_csv(f, header=0, sep=",", names=columns, dtype=col_dtype)

def convert_visualizable(pc):
	filtered = pc.drop(columns=['time','intensity','id', 'azimuth','range','pid'])
	return filtered

# user inputs
columns = ['time','intensity','id','x', 'y', 'z', 'azimuth','range','pid']
file_1 = sys.argv[1]
file_2 = sys.argv[2]

# make them visualizable
pc_vis_1 = convert_visualizable(read_points(file_1, columns))
pc_vis_2 = convert_visualizable(read_points(file_2, columns))

# combine two pointcoulds
combined = pd.concat([pc_vis_1, pc_vis_2])
v_c = pptk.viewer(combined) 
v_c.set(point_size=0.01)

# add colors
color_1 = [1,1,1]
color_2 = [0.11982556, 0.80928971, 0.082647]
colors = [color_1] * (len(pc_vis_1.index)) + [color_2] * (len(pc_vis_2.index)) 
v_c.attributes(colors)
