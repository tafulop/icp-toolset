# icp-tool
A toolset to make you able to work with pointclouds and perform an iterative closest point algorithm on them.

# The toolset was tested based on
Sydney Urban Objects Dataset
http://www.acfr.usyd.edu.au/papers/SydneyUrbanObjectsDataset.shtml


# Usage
To use the tool, it is recommended to download the attached docker image and do everything there. Image could be used without any other dependencies.

If you would like to set up your own environment, here are the dependencies:

### Python 3.6 with the following modules:
* numpy
* pptk
* pandas

### To build 
* CMake 3.5.1
* libpcl-all 1.7
* libpcl-dev 1.7

### Docker image
https://drive.google.com/file/d/1BTBZcGvzARoNZf0fc_SFsqn2ub9U8g9W/view?usp=sharing

## 1. Prepare the data with preprocessor.py

The preprocessing step reads a .csv file, optionally reduces the dataset to a user-defined percentage, applies a user-defined transformation on it, adds Gaussian noise and save the generated datasets into dedicated .csv files. The output files are falling into two categories, the first is a full column set output file, to have the option to affect only the values and not the structure of the input dataset. The second category contains a reduced column set, namely x,y,z coordinates of the points and nothing else.

Following things can be defined by the user:
* columns in the input csv file as a Python list
* transformation matrix as a 4x4 numpy matrix (eg: "1,0,0,5; 0,1,0,5; 0,0,1,0; 0,0,0,1")
* mu and sigma for the Gaussian noise generation
* name of the different output files

```bash
preprocessor.py <input_file.csv>
```


## 2. Compare generated data with original one

The visualizer simply displays the two pointclouds with different color defined by the user.

```bash
visualizer.py <original.csv> <generated.csv>
```

User defined parameters (at the moment inside the script):
* columns: name of the columns in the input files, eg. ['time','intensity','id','x', 'y', 'z', 'azimuth','range','pid'] with the Syndey Dataset
* color_1, color_2: the color of the points in the two pointclouds encoded in RGB

Sample:
![alt text](https://github.com/tafulop/ipc-toolset/blob/master/meta/Screenshot%202019-08-04%20at%2009.03.08.png?raw=true "Visualization")

## 3. Run icp tool

This tool uses the PCL C++ library to compute the transformation between the original and the generated pointclouds. Use the provided docker image to run the tool by executing:

```bash
iterative_point_cloud <original.csv> <generated.csv>
```

The input files could be easily added to the docker container by the 'docker cp' command. See https://docs.docker.com/engine/reference/commandline/cp/
