import pickle
import numpy as np
import matplotlib.pyplot as plt
import xml.etree.ElementTree as ET


#store coordinates in a list
#[(x,y),...]
ptList = []
xList = []
yList = []


tree = ET.parse("ecebModel.sdf")
root = tree.getroot()
model = root[0]

pltList = []
for n in range(2,len(model)-1):
    link = model[n]
    dimension_str = link[0][0][0][0].text
    parse_dim = dimension_str.split(' ')
    x_dim = float(parse_dim[0])
    y_dim = float(parse_dim[1])
    z_dim = float(parse_dim[2])

    position_str = link[2].text
    parse_pos = position_str.split(' ')
    x_pos = float(parse_pos[0])
    y_pos = float(parse_pos[1])
    z_pos = float(parse_pos[2])
    angle = float(parse_pos[5])




    pt1Origin = np.array([[0-(x_dim/2)], [0]])
    pt2Origin = np.array([[0+(x_dim/2)], [0]])
    rot =  np.array([[np.cos(angle), -np.sin(angle)],[np.sin(angle), np.cos(angle)]])
    pt1 = np.dot(rot, pt1Origin) + np.array([[x_pos],[y_pos]])
    pt2 = np.dot(rot, pt2Origin) + np.array([[x_pos],[y_pos]])


    ptList.append(( (pt1[0][0],pt1[1][0]) , (pt2[0][0],pt2[1][0]) ))
    xList.append(pt1[0][0])
    xList.append(pt2[0][0])
    yList.append(pt1[1][0])
    yList.append(pt2[1][0])




with open('obstaclePts.data', 'wb') as filehandle:
    # store the data as binary data stream
    pickle.dump(ptList, filehandle)

with open('obstaclePts.data', 'rb') as filehandle:
    # read the data as binary data stream
    test = pickle.load(filehandle)

# print(test)
for i in range(0, len(test)):
    wall = test[i]
    pt1 =wall[0]
    pt2= wall[1]
    plt.plot([pt1[0], pt2[0]], [pt1[1],pt2[1]], 'ro-')
    # plt.plot(xtest[i:i+2], ytest[i:i+2], 'ro-')
# print(len(test))
plt.show()
