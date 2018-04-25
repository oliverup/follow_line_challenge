#!/usr/bin/env python

import cv2
import numpy as np
import rospy
import imutils
from math import sin, cos
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from geometry_msgs.msg import Polygon, Point, PoseArray, Pose
from sensor_msgs.msg import Image, CameraInfo

           #--testing--#
#cam_pick = cv2.imread('practise_3.png')
#cam = cv2.VideoCapture(0)
#res, cam_pick = cam.read()
#cam.release()

class get_line_coordinates:
    def __init__(self,obj):
        self.rn = obj
        self.img = self.rn.camera_img
        self.WIDTH = (self.img.shape[1]-1)
        self.HEIGHT = (self.img.shape[0]-1)
        self.current_len = 0
        self.lower_limit = 90 #                                             <---
        self.temp_co_coo = []
    
    #def pick_act(self):
    #    self.img = ros_node().camera_img
    #    self.WIDTH = (self.img.shape[1]-1)
    #    self.HEIGHT = (self.img.shape[0]-1)
    #    #print self.img
        
    def val(self,pixel):
        value = 0
        for c in range(3):
            value += pixel[c]
            return value

    def search(self):
        for b in range (0 ,self.WIDTH, 3):
            darkness_len = 0
            self.current_len = 0
            if b >= (self.WIDTH-3):
                print "null_list"
                return ([(0,0),(0,0)], 1)
            for a in range(self.HEIGHT):
                current_pixel = self.img[a, b]
        
                if self.val(current_pixel) < self.lower_limit:
                    self.current_len += 1
                else: 
                    darkness_len = self.current_len
                    self.current_len = 0
                if darkness_len > 15: #min line width                       <---
                    line_middle = (a-darkness_len/2, b)
                    #print line_middle
                    if self.val(self.img[line_middle[0]-1,line_middle[1]+5]) < self.lower_limit and self.val(self.img[line_middle[0]+1,line_middle[1]+7]) < self.lower_limit: #checks if the pixels to the right are black too, so that it doesnt trigger on a thin horizontal line
                        print "follow_list"
                        return (self.follow(line_middle,(darkness_len/2)), (darkness_len/2)) #starts following
        return [(0,0),(0,0)]
        
    def follow(self,start_pos, wall_dist): #gets a starting point and then follows the line
        line_pos_list = [start_pos]
        x = start_pos[1]
        y = start_pos[0]
        bool_x = True
        bool_y_up = False
        bool_y_down = False
        x_count = 0
        y_up_count = 0
        y_down_count = 0

        while x < self.WIDTH and y < self.HEIGHT:
            if (len(line_pos_list) > 1000 or not (x < self.WIDTH-3 and y < self.HEIGHT-3)):
                print 'looped'
                return line_pos_list
            
            elif bool_x:  #asking for what diection to follow
                if self.val(self.img[y,x]) > self.lower_limit: #if the line is not black anymore
                    bool_x = False #wont move in x direction anymore
                    if x_count >= ((wall_dist/2)+1): # if it ran at least (wall_dist/2)+1 times in x direction
                        bool_y_up = True #moves in y_up direction now
                        x_count = 0
                    if len(line_pos_list) > (wall_dist/2):
                        for i in range(wall_dist/2 +1): #deletes all positions until back in the middle of the black line
                            line_pos_list.pop(-1)
                            
                    x = line_pos_list[-1][1]
                    line_pos_list.append((y,x))

                else: #appends position and goes forward 2 pixels
                    line_pos_list.append((y,x))
                    x += 2
                    x_count += 1

            elif bool_y_up: #does the same as the 'if bool_x' in y_up direction
                if self.val(self.img[y,x]) > self.lower_limit:
                    bool_y_up = False
                    if y_up_count <= ((wall_dist/2) +1): 
                        bool_y_down = True
                        for i in range(y_up_count):
                            line_pos_list.pop(-1)
                        y_up_count = 0
                    else:
                        bool_x = True
                        y_up_count = 0
                        for i in range(wall_dist/2):
                            line_pos_list.pop(-1)
                    y += (2+wall_dist)
                    line_pos_list.append((y,x))
                else:
                    line_pos_list.append((y,x))
                    y -= 2
                    y_up_count += 1

            elif bool_y_down: #does the same as the 'if bool_x' in y_down direction
                if self.val(self.img[y,x]) > self.lower_limit:
                    bool_y_down = False
                    if y_down_count >= ((wall_dist/2)+1): 
                        bool_x = True
                        y_down_count = 0
                    y -= (2+wall_dist)
                    for i in range(wall_dist/2):
                        line_pos_list.pop(-1)
                    line_pos_list.append((y,x))
                else:
                    line_pos_list.append((y,x))
                    y += 2
                    y_down_count += 1


            else:  #if you cant go in any direction anymore stop and return the list
                print 'wtf'
                return line_pos_list

    def get_corner_coords(self,line_pos_list, wall_dist):
        xp = False
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
                yp = False
                yn = False

            if line_pos_list[a][0] < line_pos_list[b][0]:
                if not yp:
                    corner_coords.append(line_pos_list[a])
                    #print 'yp'
                yp = True
                xp = False
                yn = False
            elif line_pos_list[a][0] > line_pos_list[b][0]:
                if not yn:
                    corner_coords.append(line_pos_list[a])
                    #print 'yn'
                yn = True
                xp = False
                yp = False
        
        #if dist between last two corner_coords < walldist+ ~3 pop the first of them
        corner_coords.append(line_pos_list[-1])
        if len(corner_coords) > 0:
            corner_coords.append(corner_coords[-1])
        else: 
            #print self.temp_co_coo
            return self.temp_co_coo
        for s in range(len(corner_coords)-1):
            first = corner_coords[s]
            second = corner_coords[s+1]
            diff = (first[0]-second[0])+(first[1]-second[1])
            if ((diff > wall_dist*2) or (diff < -(wall_dist*2)) or s == (len(corner_coords)-2)):
                self.temp_co_coo.append(first)
            else: pass #print "pass"
        #print corner_coords
        #print self.temp_co_coo
        
        temp_co_coo = self.arrange_1(self.temp_co_coo, (wall_dist+1))
        
        another_list = []
        for b in range(len(temp_co_coo)-1):
            for c in range(11):
                another_list.append(((temp_co_coo[b][0]+int(round(((temp_co_coo[b+1][0]-temp_co_coo[b][0])/10.0)*c))),(temp_co_coo[b][1]+int(round(((temp_co_coo[b+1][1]-temp_co_coo[b][1])/10.0)*c)))))
        
        #temp_co_coo = self.arrange_1(another_list, (wall_dist+1))
        print temp_co_coo
        return temp_co_coo
        
    def arrange_1(self,temp_co_coo, wall_dist):
        too_many_lists = [temp_co_coo[0]]
        for g in range(len(temp_co_coo)):
            if g > 0:
                yn_wall_hit = 0
                yp_wall_hit = 0
                xn_wall_hit = 0
                xp_wall_hit = 0
                distance = 0
                break_count = 0
                while True:
                    distance += 1
                    if ((temp_co_coo[g][0] + (distance)) < self.HEIGHT):
                        if self.val(self.img[(temp_co_coo[g][0] + distance),(temp_co_coo[g][1])]) > (self.lower_limit) and xp_wall_hit == 0:
                            break_count += 1
                            xp_wall_hit = distance-wall_dist
                        if break_count == 2:
                            break

                    if ((temp_co_coo[g][1] + (distance)) < self.WIDTH):
                        if self.val(self.img[(temp_co_coo[g][0]),(temp_co_coo[g][1] + distance)]) > (self.lower_limit) and yp_wall_hit == 0:
                            break_count += 1
                            yp_wall_hit = distance-wall_dist
                        if break_count == 2:
                            break
                        
                    if self.val(self.img[(temp_co_coo[g][0] - distance),(temp_co_coo[g][1])]) > (self.lower_limit) and xn_wall_hit == 0:
                        break_count += 1
                        xn_wall_hit = -(distance-wall_dist)
                    if break_count == 2:
                        break
                        
                    if self.val(self.img[(temp_co_coo[g][0]),(temp_co_coo[g][1] - distance)]) > (self.lower_limit) and yn_wall_hit == 0:
                        break_count += 1
                        yn_wall_hit = -(distance-wall_dist)
                    if break_count == 2:
                        break

                divider = 2
                if min(abs(xn_wall_hit), abs(xp_wall_hit)) == 0 and max(abs(xn_wall_hit), abs(xp_wall_hit)) != 0:
                    if min(abs(yn_wall_hit), abs(yp_wall_hit)) == 0 and max(abs(yn_wall_hit), abs(yp_wall_hit)) != 0:
                        divider = 1
                        #print "|"
                
                too_many_lists.append(((temp_co_coo[g][0] + (xn_wall_hit+xp_wall_hit)/divider),(temp_co_coo[g][1] + (yn_wall_hit+yp_wall_hit)/divider)))
                #print distance
        #print too_many_lists
        return too_many_lists


def red_line(line_pos_list,draw_img):
    #this part is to test and draws a thin red line in the black line
    for b in range(len(line_pos_list)-1):
        for c in range(1):
            draw_img[(line_pos_list[b][0]+int(round(((line_pos_list[b+1][0]-line_pos_list[b][0])/1.0)*c))),(line_pos_list[b][1]+int(round(((line_pos_list[b+1][1]-line_pos_list[b][1])/1.0)*c)))] = [0,0,200] #draws lines between every two coordinates in the line_pos_list

    cv2.imwrite('line_1.png',draw_img)

def green_line(line_pos_list,draw_img): #takes corner coords
    #this part is to test and draws a thin green line in the black line
    for b in range(len(line_pos_list)-1):
        for c in range(300):
            draw_img[(line_pos_list[b][0]+int(round(((line_pos_list[b+1][0]-line_pos_list[b][0])/300.0)*c))),(line_pos_list[b][1]+int(round(((line_pos_list[b+1][1]-line_pos_list[b][1])/300.0)*c)))] = [0,200,0] #draws lines between every two coordinates in the line_pos_list

    cv2.imwrite('line_1.png',draw_img)


class ros_node:
    def __init__(self):
        self.pub = rospy.Publisher("/line_following_coordinates", PoseArray, queue_size = 10)
        self.sub_0 = rospy.Subscriber("/camera/rgb/image_color", Image , self.subcallback)
        self.sub_1 = rospy.Subscriber("/camera/depth/image_rect", Image , self.depth_image_subcallback)
        self.sub_2 = rospy.Subscriber("/camera/rgb/camera_info", CameraInfo , self.info_subcallback)
        self.timer = rospy.Timer(rospy.Duration(1.0), self.timercallback)
        self.bridge = CvBridge()
        self.cam_info = False
        self.pick_taken = False
        self.depth_pick_taken = False
        self.processed_img = False
        self.camera_img = [] #imutils.resize(cam_pick, width=400) #
        self.corner_coords = []
        self.wall_dist = 0
        self.K = 0
        self.Z = 0
        
    def info_subcallback(self, data):
        if not self.cam_info:
            self.cam_info = True
            print data.K
            self.K = data.K
    
    def depth_image_subcallback(self, data):
        if not self.depth_pick_taken:
            self.depth_pick_taken = True
            data = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
            #print data.shape
            print data[(data.shape[0]/2), (data.shape[1]/2)]
            self.Z = data[(data.shape[0]/2), (data.shape[1]/2)]
    
    def subcallback(self, data):
        if not self.pick_taken:
            self.pick_taken = True
            self.camera_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
            
    def timercallback(self, no_idea_what_data_this_is):
        if self.pick_taken and not self.processed_img:
            self.processed_img = True
            print "go"
            glc = get_line_coordinates(self)
            draw_img = glc.img.copy()
            ab = glc.search()
            line_pos_list, self.wall_dist = ab[0],ab[1]
            print (self.wall_dist)#line_pos_list, 
            self.corner_coords = glc.get_corner_coords(line_pos_list, self.wall_dist)
            red_line(line_pos_list,draw_img)
            green_line(self.corner_coords,draw_img)
            cv2.imshow("img", draw_img)
            cv2.waitKey(1)
            self.pick_taken = False
            print 'Done'
        else: 
            self.processed_img = True
            print 'no topic to subscribe'
            
        print ':('
        if self.processed_img:
            p = Pose()
            pose_msg = PoseArray()
            pose_msg.header.stamp = rospy.get_rostime()
            pose_msg.header.frame_id = "camera_frame"
            i = 0
            while (i <= len(self.corner_coords)):
                if i == len(self.corner_coords)-1:
                    break
                p.position.z = self.Z
                # v = fy * Y + cy * Z
                p.position.y = (self.corner_coords[i][0] - self.K[4] * p.position.z)/self.K[5]
                # u = fx * X + cx * Z
                p.position.x = (self.corner_coords[i][1] - self.K[0] * p.position.z)/self.K[2]
                p.orientation.w = 1
                pose_msg.poses.append(p)

                i += 1

                p.position.z = self.Z
                # v = fy * Y + cy * Z
                p.position.y = (self.corner_coords[i][0] - self.K[4] * p.position.z)/self.K[5]
                # u = fx * X + cx * Z
                p.position.x = (self.corner_coords[i][1] - self.K[0] * p.position.z)/self.K[2]
                p.orientation.w = 1
                pose_msg.poses.append(p)
                
                print p
                #print pose_msg
                print i

            self.pub.publish(pose_msg)
            print 'this should really be published'

#        p = Point()
#        msg = Polygon()
#        if self.processed_img:
#            for item in self.corner_coords:
#                p.x = item[0]
#                p.y = item[1]
#                msg.points.append(p)
#            self.pub.publish(msg)

    def run(self,rate):
        rate.sleep()

'''
pose_msg = PoseArray()
pose_msg.header.stamp = rospy.get_rostime()
pose_msg.header.frame_id = "camera_frame"
for(i = 0, i < len(self.corner_coords), i += 1):
    pose_msg.poses[i].position.z = self.Z
    # v = fy * Y + cy * Z
    pose_msg.poses[i].position.y = (self.corner_coords[i][0] - self.K[4] * pose_msg.poses[i].position.z)/self.K[5]
    # u = fx * X + cx * Z
    pose_msg.poses[i].position.x = (self.corner_coords[i][1] - self.K[0] * pose_msg.poses[i].position.z)/self.K[2]
    pose_msg.poses[i].orientation.w = 1

    i += 1

    pose_msg.poses[i].position.z = self.Z
    # v = fy * Y + cy * Z
    pose_msg.poses[i].position.y = (self.corner_coords[i][0] - self.K[4] * pose_msg.poses[i].position.z)/self.K[5]
    # u = fx * X + cx * Z
    pose_msg.poses[i].position.x = (self.corner_coords[i][1] - self.K[0] * pose_msg.poses[i].position.z)/self.K[2]
    pose_msg.poses[i].orientation.w = 1
    
    i -= 1

pub.publish(pose_msg)
'''
def main():
    rospy.init_node('line_following')

    r = ros_node()

    rate = rospy.Rate(1)
    while not rospy.is_shutdown() and not r.processed_img:
        r.run(rate)
    cv2.destroyAllWindows()

if __name__=="__main__":
    main()
