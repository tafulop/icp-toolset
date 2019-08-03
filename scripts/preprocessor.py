import numpy as np
import pandas as pd
import sys
import os
from random import gauss

# read the pointcloud file
# must contain 'x,y,z' columns
def read_points(f, columns):
	col_dtype = {'x': np.float32, 'y': np.float32, 'z': np.float32}
	return pd.read_csv(f, header=0, sep=",", names=columns, dtype=col_dtype)

def shrink_data(pc, keep_ratio = 0.8):
	shrink_pc = pd.DataFrame()
	items_to_remove = len(pc.index) / (len(pc.index) * (1 - keep_ratio))
	for row in range(0,len(pc.index)):
		if(row % int(items_to_remove)):   
			shrink_pc.at[row,'x'] = pc.at[row,'x']
			shrink_pc.at[row,'y'] = pc.at[row,'y']
			shrink_pc.at[row,'z'] = pc.at[row,'z']
	print("original dataset elements: " + str(len(pc.index)))
	print("shrunk dataset elements:   " + str(len(shrink_pc.index)))
	return shrink_pc

# Performs a transformation on the pointcloud
def perform_transform(pc, tf):
	copy_pc = pc.copy()
	for row  in range(0,len(pc.index)):
	# create vector from coordinates
		point_vector = np.matrix([[copy_pc.at[row,'x']],[copy_pc.at[row,'y']],[copy_pc.at[row,'z']],[1]])
		transformed = tf * point_vector
		copy_pc.at[row,'x'] = transformed[0]
		copy_pc.at[row,'y'] = transformed[1]
		copy_pc.at[row,'z'] = transformed[2]
	return copy_pc 

# Add noise to every point in the pointcloud and 
# return a copy of the original pointcloud with noise
def add_noise(pc, mu=0, sigma=0.1):
	copy_pc = pc.copy()
	for row in range(1,len(pc.index)):
		copy_pc.at[row,'x'] = copy_pc.at[row,'x'] + gauss(mu, sigma)  
		copy_pc.at[row,'y'] = copy_pc.at[row,'y'] + gauss(mu, sigma)
		copy_pc.at[row,'z'] = copy_pc.at[row,'z'] + gauss(mu, sigma)
	return copy_pc

# user inputs
columns = ['time','intensity','id','x', 'y', 'z', 'azimuth','range','pid']
file = sys.argv[1]
tf = np.matrix(	"1,0,0,5; 0,1,0,0; 0,0,1,0; 0,0,0,1")
mu = 0
sigma = 0.5

# perform calcuations
pc = read_points(file, columns)
pc_s = shrink_data(pc, 0.8)
print(pc_s)
pc_tf = perform_transform(pc_s, tf)
pc_tf_n = add_noise(pc_tf, mu, sigma)

# save results
pc_tf.to_csv("output_transformed.csv", index=False)
pc_tf_n.to_csv("output_transformed_noised.csv", index=False)