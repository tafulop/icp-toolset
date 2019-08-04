import numpy as np
import pandas as pd
import sys
import os
from random import gauss

# read the pointcloud file
# must contain 'x,y,z' columns
def read_points(f, columns):
	print("-" * 50)
	print("1. reading file")
	print("-" * 50)
	print("file: " + str(f))

	col_dtype = {'x': np.float32, 'y': np.float32, 'z': np.float32}
	input = pd.read_csv(f, header=0, sep=",", names=columns, dtype=col_dtype)
	print("data:\n\n" + str(input))
	return input

# removes items from the dataset and keeping the given ratio
# items removed in a sequential way based on the index
def shrink_data(pc, keep_ratio = 0.8):
	print("-" * 50)
	print("2. shrinking dataset to ratio: " + str(keep_ratio))
	print("-" * 50)

	indicies_to_drop = []
	items_to_remove = len(pc.index) / (len(pc.index) * (1 - keep_ratio))
	for row in range(1,len(pc.index)):
		if(not row % int(items_to_remove)):   
			indicies_to_drop.append(row)
	dropped = pc.drop(indicies_to_drop)
	print("original dataset elements: " + str(len(pc.index)))
	print("shrunk dataset elements:   " + str(len(dropped.index)))
	return dropped.reset_index(drop=True)

# Performs a transformation on the pointcloud
def perform_transform(pc, tf):
	print("-" * 50)
	print("3. applying transformation")
	print("-" * 50)
	print("tf matrix: \n" + str(tf))

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
	print("-" * 50)
	print("4. adding Gaussian noise")
	print("-" * 50)
	print("mu = " + str(mu) + "\nsigma=" + str(sigma))

	for row in range(1,len(pc.index)):
		copy_pc.at[row,'x'] = copy_pc.at[row,'x'] + gauss(mu, sigma)  
		copy_pc.at[row,'y'] = copy_pc.at[row,'y'] + gauss(mu, sigma)
		copy_pc.at[row,'z'] = copy_pc.at[row,'z'] + gauss(mu, sigma)
	return copy_pc

# save output files
def save_outputs(file_dict):
	print("-" * 50)
	print("saving results")
	print("-" * 50)
	for file, data in file_dict.items(): 
		data.to_csv(file, index=False)
		print("saved '" + file + "' with data:\n\n" + str(data))

# user inputs
columns = ['time','intensity','id','x', 'y', 'z', 'azimuth','range','pid']
file = sys.argv[1]
tf = np.matrix(	"1,0,0,0; 0,1,0,7; 0,0,1,0; 0,0,0,1")
mu = 0
sigma = 0.05
transformed_output_file = "output_transformed.csv"
noised_transformed_output_file = "output_transformed_noised.csv"

# perform calcuations
pc = read_points(file, columns)
pc_s = shrink_data(pc, 0.8)
pc_tf = perform_transform(pc_s, tf)
pc_tf_n = add_noise(pc_tf, mu, sigma)

# save results
save_outputs({transformed_output_file : pc_tf, 
			  noised_transformed_output_file : pc_tf_n})