#!/usr/bin/env python

import cv2
import numpy as np
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Polygon
from geometry_msgs.msg import Point
#cut out the right part of the image to work with ... as in cut the white part with the line out of the picture the bot takes



#make a copy of the picture and draw a line in the black line
class get_line_coordinates:
    def __init__(self):
        self.img = cv2.imread(str(raw_input('image:  '))) #img is later changed to be the cut version of the picture
        self.WIDTH = self.img.shape[1]
        self.HEIGHT = self.img.shape[0]
        self.run = True
        self.current_len = 0

    def val(self,pixel):
        value = 0
        for c in range(3):
            value += pixel[c]
            return value

    def follow(self,start_pos, wall_dist): #gets a starting point and then follows the line
        #max_line_len = 0
        line_pos_list = [] #all positions of the line to draw, currently holds every second pixel of the line
        run_follow = True
        x = start_pos[1] # x starting position of the line to follow
        y = start_pos[0] # y starting position of the line to follow
        bool_x = True
        bool_y_up = False
        bool_y_down = False
        x_count = 0
        y_up_count = 0
        y_down_count = 0

        while run_follow:
            if bool_x:  #asking for what diection to follow
                if self.val(self.img[y,x]) > 50: #if the line is not black anymore
                    bool_x = False #wont move in x direction anymore
                    if x_count >= 4: # if it ran at least 4 times in x direction go in y direction upwards
                        bool_y_up = True #moves in y_up direction now
                        x_count = 0
                    for i in range(wall_dist/2): #deletes all positions until back in the middle of the black line
                        line_pos_list.pop(-1)
                    x -= (2+wall_dist)
                    line_pos_list.append((y,x))

                else:
                    line_pos_list.append((y,x)) #appends position and goes forward 2 pixels
                    #print 'im going forward'
                    x += 2
                    x_count += 1

            elif bool_y_up: #does the same as the 'if bool_x' part in y_up direction
                if self.val(self.img[y,x]) > 50:
                    bool_y_up = False
                    if y_up_count <= 4: 
                        bool_y_down = True
                        y_up_count = 0
                    else:
                        bool_x = True
                        y_up_count = 0
                    y += (2+wall_dist)
                    for i in range(wall_dist/2):
                        line_pos_list.pop(-1)
                    line_pos_list.append((y,x))
                else:
                    line_pos_list.append((y,x))
                    #print 'im going up'
                    y -= 2
                    y_up_count += 1

            elif bool_y_down: #does the same as the 'if bool_x' part in y_down direction
                if self.val(self.img[y,x]) > 50:
                    bool_y_down = False
                    if y_down_count >= 4: 
                        bool_x = True
                        y_down_count = 0
                    y -= (2+wall_dist)
                    for i in range(wall_dist/2):
                        line_pos_list.pop(-1)
                    line_pos_list.append((y,x))
                else:
                    line_pos_list.append((y,x))
                    #print 'im going down'
                    y += 2
                    y_down_count += 1

            else:  #if you cant go in any direction anymore stop and return the list
                return line_pos_list
                run_follow = False
            #max_line_len += 1
            #if max_line_len ==400:
            #    return line_pos_list
            #    run = False

    def search(self):
        for b in range (0 ,self.WIDTH, 3):
            if not self.run: break
            darkness_len = 0
            for a in range(self.HEIGHT):
                current_pixel = self.img[a, b]
                #self.draw_img[a,b] = [0,200,0]
        
                if self.val(current_pixel) < 50: #if the pixel is black
                    self.current_len += 1
                else: 
                    darkness_len = self.current_len
                    self.current_len = 0
                if darkness_len > 5: #if the length of black pixels in a row is bigger than 5
                    line_middle = (a-darkness_len/2, b) 
                    if self.val(self.img[line_middle[0],line_middle[1]+5]) < 50 and self.val(self.img[line_middle[0],line_middle[1]+7]) < 50: #checks if the pixels to the right are black too, so that it doesnt trigger on a thin horizontal black line
                        return self.follow(line_middle,(darkness_len/2)-1) #starts following the line and returns the line_pos_list
                        self.run = False
                        break

glc = get_line_coordinates()
draw_img = glc.img.copy()

#for h in range(self.WIDTH):
#    for w in range(self.HEIGHT):
#        draw_img[w,h] = [255,255,255]

#this part is to test and draws a thin red line in the black line. later the arm follows the coordinates of the line_pos_list
line_pos_list = glc.search()

def get_corner_coords(line_pos_list):
    xp = False
    xn = False
    yp = False
    yn = False
    corner_coords = []

    for a in range (len(line_pos_list)):
        if a != (len(line_pos_list)-1): b = a+1

        if line_pos_list[a][1] < line_pos_list[b][1]:
            if not xp:
                corner_coords.append(line_pos_list[a])
                #print 'xp'
            xp = True
            xn = False
            yp = False
            yn = False
        elif line_pos_list[a][1] > line_pos_list[b][1]:
            if not xn:
                corner_coords.append(line_pos_list[a])
                #print 'xn'
            xn = True
            xp = False
            yp = False
            yn = False

        if line_pos_list[a][0] < line_pos_list[b][0]:
            if not yp:
                corner_coords.append(line_pos_list[a])
                #print 'yp'
            yp = True
            xp = False
            xn = False
            yn = False
        elif line_pos_list[a][0] > line_pos_list[b][0]:
            if not yn:
                corner_coords.append(line_pos_list[a])
                #print 'yn'
            yn = True
            xp = False
            xn = False
            yp = False
    return corner_coords

corner_coords = get_corner_coords(line_pos_list)
print corner_coords
for b in range(len(line_pos_list)-1):
    for c in range(10):
        draw_img[(line_pos_list[b][0]+int(round(((line_pos_list[b+1][0]-line_pos_list[b][0])/10.0)*c))),(line_pos_list[b][1]+int(round(((line_pos_list[b+1][1]-line_pos_list[b][1])/10.0)*c)))] = [0,0,200] #draws lines between every two coordinates in the line_pos_list

cv2.imwrite('line.png',draw_img)
#print line_pos_list
print 'Done'

class ros_node:
    def __init__(self):
        self.pub = rospy.Publisher("line_following_coordinates", Polygon, queue_size = 10)
#        self.sub = rospy.Subscriber("topic the camera publishes on?", picture message? , self.subcallback)
        self.timer = rospy.Timer(rospy.Duration(10.0), self.timercallback)
#    def subcallback(self, data):
#        camera_img = 
    def timercallback(self, data):
        p = Point()
        msg = Polygon()
        for item in corner_coords:
            p.x = item[0]
            p.y = item[1]
            msg.points.append(p)
        self.pub.publish(msg)
    def run(self,rate):
        rate.sleep()

def main():
    rospy.init_node('line_following')

    r = ros_node()
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        r.run(rate)


if __name__=="__main__":
    main()
