import numpy as np
import pptk
import pandas as pd
import sys
import os
from random import gauss

def read_points(f):
	# reads Semantic3D .txt file f into a pandas dataframe
	col_names = ['time','intensity','id','x', 'y', 'z', 'azimuth','range','pid']
	# col_names = ['x','y','z']
	col_dtype = {'x': np.float32, 'y': np.float32, 'z': np.float32}
	return pd.read_csv(f, header=0, sep=",", names=col_names, dtype=col_dtype)

# Performs a transformation on the pointcloud
def perform_transform(pc, tf):
	copy_pc = pc.copy()
	for row  in range(0,len(pc.index)):
	# create vector from coordinates
		point_vector = np.matrix([[copy_pc.at[row,'x']],[copy_pc.at[row,'y']],[copy_pc.at[row,'z']],[1]])
		transformed = tf * point_vector
		print("-------------------------")
		print(str(tf) + "\n \t* \n" + str(point_vector) + "\n\t=\n" + str(transformed))
		copy_pc.at[row,'x'] = transformed[0]
		copy_pc.at[row,'y'] = transformed[1]
		copy_pc.at[row,'z'] = transformed[2]
	return copy_pc 

# Add noise to every point in the pointcloud and 
# return a copy of the original pointcloud with noise
def add_noise(pc, mu=0, sigma=0.1):
	copy_pc = pc.copy()
	for row in range(1,len(pc.index)):
	#print("x: " + str(pc.at[row, 'x']) + "\ty: " + str(pc.at[row, 'y']) + "\tz: " + str(pc.at[row, 'z'])) 
		copy_pc.at[row,'x'] = copy_pc.at[row,'x'] + gauss(mu, sigma)  
		copy_pc.at[row,'y'] = copy_pc.at[row,'y'] + gauss(mu, sigma)
		copy_pc.at[row,'z'] = copy_pc.at[row,'z'] + gauss(mu, sigma)
	return copy_pc

def convert_visualizable(pc):
	filtered = pc.drop(columns=['time','intensity','id', 'azimuth','range','pid'])
	return filtered

file = str(os.path.abspath(sys.argv[1]))
print("reading file: " + file)
points = read_points(file)

tf = np.matrix(	"1,0,0,5;\
		 0,1,0,0;\
		 0,0,1,0;\
		 0,0,0,1")
transformed = perform_transform(points, tf)
noise_points = add_noise(transformed)

# saving Tf + noisy file
points.to_csv("input.csv", index=False)
transformed.to_csv("input_tf.csv", index=False)
noise_points.to_csv("input_tf_noise.csv", index=False)

# make them visualizable
pc_vis_1 = convert_visualizable(points)
pc_vis_2 = convert_visualizable(noise_points)

# combine two pointcoulds
combined = pd.concat([pc_vis_1, pc_vis_2])
v_c = pptk.viewer(combined) 
v_c.set(point_size=0.01)

# add colors
color_1 = [1,1,1]
color_2 = [0.11982556, 0.80928971, 0.082647]
colors = [color_1] * (len(combined.index)/2) + [color_2] * (len(combined.index)/2) 
v_c.attributes(colors)
