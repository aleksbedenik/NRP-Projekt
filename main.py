# import the necessary libraries 
import csv 
import numpy as np 

# open the CSV file 
with open('dataset1.csv', 'r') as csvfile: 
    reader = csv.reader(csvfile) 

# create an empty list to store the data 
    data = [] 

# read each row of the CSV file 
    for row in reader: 
        data.append(row) 

# convert the data to a numpy array 
    data = np.array(data) 

# extract the x, y, and z values from the data 
    x = data[:,0].astype(float) 
    y = data[:,1].astype(float) 
    z = data[:,2].astype(float) 
    x1 = data[1:,0].astype(float) 
    y1 = data[1:,1].astype(float) 
    z1 = data[1:,2].astype(float)
    x1 = np.resize(x1,(1000,))
    y1 = np.resize(y1,(1000,))
    z1 = np.resize(z1,(1000,))
    #print("x",x,"x1",x1,"\n")
# calculate the speed from the x, y, and z values 
    speed = (((x1) - (x)) + ((y1) - (y)) + ((z1) - (z))) / 0.5

# add the speed column to the CSV file 
    data = np.column_stack((data, speed)) 

# write the data back to a new CSV file 
    with open('dataset_with_speed.csv', 'w') as csvfile: 
        writer = csv.writer(csvfile) 
        writer.writerows(data)
