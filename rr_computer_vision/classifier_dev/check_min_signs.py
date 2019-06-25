import os


path = "C:/Users/ethie/OneDrive - University of Waterloo/Documents/UWRobotics/StreetImageDataset/synthetic_data/haar_training_data/positives/info.dat"

file = open(path,"r") 
lines = file.readlines()
file.close() 

min_side = 100
for line in lines: 
    w = int(line.strip().split()[4])
    h = int(line.strip().split()[5])
    if w < min_side:
        min_side = w
    if h < min_side:
        min_side = h

print(min_side)  




