import pickle
import numpy as np
import matplotlib.pyplot as plt
import xml.etree.ElementTree as ET


#store coordinates in a list
#[(x,y),...]
rect_list = []


tree = ET.parse("ecebModel.sdf")
root = tree.getroot()

model = root[0]


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
    #print(n, angle)
    #print(x_dim, y_dim, x_pos, y_pos, z_pos, angle)
    # if(angle == 1.57079 or angle == 1.5708):
    #     lowerX = 0 - (x_dim/2)
    #     lowerY = 0 - (y_dim/2)
    #     upperX = 0 + (x_dim/2)
    #     upperY = 0 + (y_dim/2)
    #
    # if(angle == 0.0):


    #For polytope
    # wallAtOrigin_lowerX = 0 - (x_dim/2)
    # wallAtOrigin_lowerY = 0 - (y_dim/2)
    # wallAtOrigin_upperX = 0 + (x_dim/2)
    # wallAtOrigin_upperY = 0 + (y_dim/2)
    #
    #
    # #[wall #, (wall position at origin), translation, rotation angle]
    # bottom_left_pt = [n, (wallAtOrigin_lowerX, wallAtOrigin_lowerY, 0), (x_pos, y_pos, z_pos), angle ]
    # top_right_pt = [n, (wallAtOrigin_upperX, wallAtOrigin_upperY, 2.5), (x_pos, y_pos, z_pos), angle ]
    # #print(bottom_left_pt, top_right_pt, '\n\n\n')
    # rect_list.append(bottom_left_pt)
    # rect_list.append(top_right_pt)
    ###########################################################################

    #for do rect inter
    #if(angle == 0 or angle == 3.14159 or abs(angle) ==1.5708):
    pt1_origin = np.array([[0-(x_dim/2)], [0-(y_dim/2)]])
    pt2_origin = np.array([[0+(x_dim/2)], [0-(y_dim/2)]])
    pt3_origin = np.array([[0-(x_dim/2)], [0+(y_dim/2)]])
    pt4_origin = np.array([[0+(x_dim/2)], [0+(y_dim/2)]])
    rot =  np.array([[np.cos(angle), -np.sin(angle)],[np.sin(angle), np.cos(angle)]])
    pt1 = np.dot(rot, pt1_origin) + np.array([[x_pos],[y_pos]])
    pt2 = np.dot(rot, pt2_origin) + np.array([[x_pos],[y_pos]])
    pt3 = np.dot(rot, pt3_origin) + np.array([[x_pos],[y_pos]])
    pt4 = np.dot(rot, pt4_origin) + np.array([[x_pos],[y_pos]])
    pt_list = [pt1,pt2,pt3,pt4]
    pt_sum_list = [pt1[0]+pt1[1],pt2[0]+pt2[1],pt3[0]+pt3[1],pt4[0]+pt4[1]]
    lower_pt = pt_list[pt_sum_list.index(min(pt_sum_list))]
    upper_pt = pt_list[pt_sum_list.index(max(pt_sum_list))]
    # plt.plot(pt1[0], pt1[1],".k")
    # plt.plot(pt2[0], pt2[1],".k")
    # plt.plot(pt3[0], pt3[1],".k")
    # plt.plot(pt4[0], pt4[1],".k")
    # if n==81:
    #     print(pt_list)
    #     print(pt_sum_list)
    #     print(angle)
    # print(angle)
    # rect_list.append([n,lower_pt])
    # rect_list.append([n,upper_pt])

    rect_list.append(((lower_pt[0][0],lower_pt[1][0],0), (upper_pt[0][0], upper_pt[1][0],2.5)))

    # pt1_origin = np.array([[0-(x_dim/2)], [0-(y_dim/2)]])
    # pt2_origin = np.array([[0+(x_dim/2)], [0-(y_dim/2)]])
    # pt3_origin = np.array([[0-(x_dim/2)], [0+(y_dim/2)]])
    # pt4_origin = np.array([[0+(x_dim/2)], [0+(y_dim/2)]])
    # rot =  np.array([[np.cos(angle), -np.sin(angle)],[np.sin(angle), np.cos(angle)]])
    # pt1 = (np.dot(rot, pt1_origin) + np.array([[x_pos],[y_pos]]))#.tolist()
    # pt2 = (np.dot(rot, pt2_origin) + np.array([[x_pos],[y_pos]]))#.tolist()
    # pt3 = (np.dot(rot, pt3_origin) + np.array([[x_pos],[y_pos]]))#.tolist()
    # pt4 = (np.dot(rot, pt4_origin) + np.array([[x_pos],[y_pos]]))#.tolist()
    # bottom_rect = [(pt1[0][0], pt1[1][0],0),(pt2[0][0], pt2[1][0],0),(pt3[0][0], pt3[1][0],0),(pt4[0][0], pt4[1][0],0)]
    # top_rect = [(pt1[0][0], pt1[1][0],2.5),(pt2[0][0], pt2[1][0],2.5),(pt3[0][0], pt3[1][0],2.5),(pt4[0][0], pt4[1][0],2.5)]
    # rect_list.append([n,bottom_rect, top_rect])

with open('rect_list.data', 'wb') as filehandle:
    # store the data as binary data stream
    pickle.dump(rect_list, filehandle)

with open('rect_list.data', 'rb') as filehandle:
    # read the data as binary data stream
    rect_list = pickle.load(filehandle)
