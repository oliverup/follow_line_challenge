#!/usr/bin/env python

import time
import cv2
import numpy as np
import rospy
#import imutils
from math import sin, cos
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from geometry_msgs.msg import PoseArray, Pose
from sensor_msgs.msg import Image, CameraInfo
from std_srvs.srv import Empty as Empty

time.sleep(15)
class get_line_coordinates:
    def __init__(self,obj):
        self.rn = obj
        self.img = self.rn.camera_img
        self.WIDTH = (self.img.shape[1])
        self.HEIGHT = (self.img.shape[0])
        self.current_len = 0
        self.temp_co_coo = []

    def setup(self,obj):
        self.rn = obj
        print "got a new img"
        self.img = self.rn.camera_img
        self.WIDTH = (self.img.shape[1])
        self.HEIGHT = (self.img.shape[0])
        self.current_len = 0
        self.temp_co_coo = []
        
        #white = cv2.imread('white.png')
        #white_resized = cv2.resize(white,(self.WIDTH, self.HEIGHT), interpolation = cv2.INTER_CUBIC)
        ############# create bin image without grid #################
        #self.img[(self.HEIGHT-50):self.HEIGHT, :] = white_resized[(self.HEIGHT-50):self.HEIGHT, :]
        self.img = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
        kernel = np.ones((5,5),np.uint8)
        ret,thresh = cv2.threshold(self.img,95	,255,cv2.THRESH_BINARY_INV)#        <-------------THRESHHOLD
        self.img = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
        cv2.imshow('binary', self.img)
        cv2.waitKey(5000)
        cv2.destroyAllWindows()

    def search(self):
        for b in range (0 ,self.WIDTH, 3):
            darkness_len = 0
            self.current_len = 0
            if b >= (self.WIDTH-3):
                print "no starting point found!"
                return ([(0,0),(0,0)], 1)
            for a in range(self.HEIGHT):
                current_pixel = self.img[a][b]
        
                if current_pixel == 255:
                    self.current_len += 1
                else: 
                    darkness_len = self.current_len
                    self.current_len = 0
                if darkness_len > 50:
                    pass
                elif darkness_len > 15: #                                            <------------MIN LINE WIDTH 
                    line_middle = (a-darkness_len/2, b)
                    print "starting point found!"
                    return (self.follow(line_middle,(darkness_len/2)), (darkness_len/2))
        
    def follow(self,start_pos, wall_dist):
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
            
            elif bool_x:
                if self.img[y,x] == 0:
                    bool_x = False
                    if x_count >= ((wall_dist/2)+1):
                        bool_y_up = True
                        x_count = 0
                    if len(line_pos_list) > (wall_dist/2):
                        for i in range(wall_dist/2 +1):
                            line_pos_list.pop(-1)
                            
                    x = line_pos_list[-1][1]
                    line_pos_list.append((y,x))

                else:
                    line_pos_list.append((y,x))
                    x += 2
                    x_count += 1

            elif bool_y_up:
                if self.img[y,x] == 0:
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

            elif bool_y_down:
                if self.img[y,x] == 0:
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


            else:
                print 'finished following!'
                return line_pos_list

    def get_corner_coords(self,line_pos_list, wall_dist):
        xp = False
        yp = False
        yn = False
        temp_co_coo = []

        for a in range (len(line_pos_list)):
            if a != (len(line_pos_list)-1):
                b = a+1

            if line_pos_list[a][1] < line_pos_list[b][1]:
                if not xp:
                    temp_co_coo.append(line_pos_list[a])
                xp = True
                yp = False
                yn = False

            if line_pos_list[a][0] < line_pos_list[b][0]:
                if not yp:
                    temp_co_coo.append(line_pos_list[a])
                yp = True
                xp = False
                yn = False
            elif line_pos_list[a][0] > line_pos_list[b][0]:
                if not yn:
                    temp_co_coo.append(line_pos_list[a])
                yn = True
                xp = False
                yp = False
        
        temp_co_coo.append(line_pos_list[-1])
        
        if len(temp_co_coo) > 0:
            temp_co_coo.append(temp_co_coo[-1])
        else:
            print "couldnt find corners!"
            return self.temp_co_coo
        for s in range(len(temp_co_coo)-1):
            first = temp_co_coo[s]
            second = temp_co_coo[s+1]
            diff = (first[0]-second[0])+(first[1]-second[1])
            if ((diff > wall_dist*2) or (diff < -(wall_dist*2)) or s == (len(temp_co_coo)-2)):
                self.temp_co_coo.append(first)
        
        corner_coords = self.arrange(self.temp_co_coo, (wall_dist+1))
        
        print corner_coords
        return corner_coords
        
    def arrange(self,corner_coords, wall_dist):
        temp_co_coo = [corner_coords[0]]
        for g in range(len(corner_coords)):
            if g > 0:
                yn_wall_hit = 0
                yp_wall_hit = 0
                xn_wall_hit = 0
                xp_wall_hit = 0
                distance = 0
                break_count = 0
                while True:
                    distance += 1
                    if ((corner_coords[g][0] + (distance)) < self.HEIGHT):
                        if self.img[(corner_coords[g][0] + distance),(corner_coords[g][1])] == 0 and xp_wall_hit == 0:
                            break_count += 1
                            xp_wall_hit = distance-wall_dist
                        if break_count == 2:
                            break

                    if ((corner_coords[g][1] + (distance)) < self.WIDTH):
                        if self.img[(corner_coords[g][0]),(corner_coords[g][1] + distance)] == 0 and yp_wall_hit == 0:
                            break_count += 1
                            yp_wall_hit = distance-wall_dist
                        if break_count == 2:
                            break
                        
                    if self.img[(corner_coords[g][0] - distance),(corner_coords[g][1])] == 0 and xn_wall_hit == 0:
                        break_count += 1
                        xn_wall_hit = -(distance-wall_dist)
                    if break_count == 2:
                        break
                        
                    if self.img[(corner_coords[g][0]),(corner_coords[g][1] - distance)] == 0 and yn_wall_hit == 0:
                        break_count += 1
                        yn_wall_hit = -(distance-wall_dist)
                    if break_count == 2:
                        break

                divider = 2
                if min(abs(xn_wall_hit), abs(xp_wall_hit)) == 0 and max(abs(xn_wall_hit), abs(xp_wall_hit)) != 0:
                    if min(abs(yn_wall_hit), abs(yp_wall_hit)) == 0 and max(abs(yn_wall_hit), abs(yp_wall_hit)) != 0:
                        divider = 1
                
                temp_co_coo.append(((corner_coords[g][0] + (xn_wall_hit+xp_wall_hit)/divider),(corner_coords[g][1] + (yn_wall_hit+yp_wall_hit)/divider)))
                
        return temp_co_coo


def red_line(line_pos_list,draw_img):
    for b in range(len(line_pos_list)-1):
        for c in range(1):
            draw_img[(line_pos_list[b][0]+int(round(((line_pos_list[b+1][0]-line_pos_list[b][0])/1.0)*c))),(line_pos_list[b][1]+int(round(((line_pos_list[b+1][1]-line_pos_list[b][1])/1.0)*c)))] = [0,0,200]
    
    cv2.imwrite('line_1.png',draw_img)

def green_line(corner_coords,draw_img):
    for b in range(len(corner_coords)-1):
        for c in range(300):
            draw_img[(corner_coords[b][0]+int(round(((corner_coords[b+1][0]-corner_coords[b][0])/300.0)*c))),(corner_coords[b][1]+int(round(((corner_coords[b+1][1]-corner_coords[b][1])/300.0)*c)))] = [0,200,0]
    
    cv2.imwrite('line_1.png',draw_img)


class ros_node:
    def __init__(self):
        self.pub = rospy.Publisher("/line_following_coordinates", PoseArray, queue_size = 10)
        self.sub_0 = rospy.Subscriber("/camera/rgb/image_raw", Image , self.subcallback)
        self.sub_1 = rospy.Subscriber("/camera/depth/image_rect", Image , self.depth_image_subcallback)
        self.sub_2 = rospy.Subscriber("/camera/rgb/camera_info", CameraInfo , self.info_subcallback)
        self.timer = rospy.Timer(rospy.Duration(1.0), self.timercallback)
        self.bridge = CvBridge()
        self.cam_info = False
        self.pic_taken = False
        self.depth_pic_taken = False
        self.gathered_all_data = False
        self.temp = False
        self.camera_img = []
        self.K = 0
        self.Z = 0
        self.pose_msg = PoseArray()
        self.service = rospy.Service('/reset_line_vision', Empty, self.service_cb)
        self.service2 = rospy.Service('/datas_recieved', Empty, self.service_cb2)
        self.data_recieved = False
        
    def info_subcallback(self, data):
        if not self.cam_info:
            self.cam_info = True
            print 'cam_info taken: %s' %str(data.K)
            self.K = data.K
    
    def service_cb(self, data):
        #reset
        self.cam_info = False
        self.pic_taken = False
        self.depth_pic_taken = False
        self.gathered_all_data = False
        self.temp = False
        self.camera_img = []
        self.K = 0
        self.Z = 0
        self.pose_msg = PoseArray()
        self.data_recieved = False
        return []
        
    def service_cb2(self, data):
        self.data_recieved = True
        return []
    
    def depth_image_subcallback(self, data):
        if not self.depth_pic_taken:
            self.depth_pic_taken = True
            data = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
            print 'depth_info taken: %s' %data[(data.shape[0]/2), (data.shape[1]/2)]
            self.Z = data[(data.shape[0]/2), (data.shape[1]/2)]
    
    def subcallback(self, data):
        if not self.pic_taken:
            self.pic_taken = True
            print 'picture taken'
            self.camera_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
           
    def timercallback(self, no_idea_what_data_this_is):
        if self.cam_info and self.pic_taken and self.depth_pic_taken:
            #self.gathered_all_data = True
            if not self.temp: 
                self.main()
            	if not self.data_recieved:
                	self.pub.publish(self.pose_msg)
    '''
            else:
                #print self.pose_msg
                for a in range(len(self.pose_msg.poses)):
                    final_array = PoseArray()
                    final_array.header.stamp = rospy.get_rostime()
                    final_array.header.frame_id = "camera_link"
                    if a < (len(self.pose_msg.poses)-1):
                        final_array.poses.append(self.pose_msg.poses[a])
                        final_array.poses.append(self.pose_msg.poses[a+1])
                        self.pub.publish(final_array)
    '''
    def main(self):
        self.temp = True
        print "Start"
        glc = get_line_coordinates(self)
        draw_img = glc.img.copy()
        glc.setup(self)
        ab = glc.search()
        line_pos_list, wall_dist = ab[0],ab[1]
        corner_coords = glc.get_corner_coords(line_pos_list, wall_dist)
        red_line(line_pos_list,draw_img)
        green_line(corner_coords,draw_img)
        cv2.imshow("img", draw_img)
        cv2.waitKey(5000)
        cv2.destroyAllWindows()
        print '2d work done.'

        p = Pose()
        pose_msg = PoseArray()
        pose_msg.header.stamp = rospy.get_rostime()
        pose_msg.header.frame_id = "camera_link"
        i = 0
        while (i <= len(corner_coords)):
            if i == len(corner_coords)-1:
                break
            p = Pose()
            p.position.z = self.Z
            #p.position.y = (corner_coords[i][0] - self.K[4] * p.position.z)/self.K[5]
            #p.position.x = (corner_coords[i][1] - self.K[0] * p.position.z)/self.K[2]
            #print ("(corner_coords[i][0] - (glc.HEIGHT/2))     :     " + str(corner_coords[i][0] - (glc.HEIGHT/2)))
            #print (((np.tan(np.arccos(self.K[4]/(np.sqrt((self.K[4])**2+(corner_coords[i][0] - (glc.HEIGHT/2))**2))))))*p.position.z)

            #p.position.y = np.tan((np.arccos((corner_coords[i][0] - (glc.HEIGHT/2)) / self.K[4]))) * p.position.z
            #p.position.x = np.tan((np.arccos((corner_coords[i][1] - (glc.WIDTH/2)) / self.K[0]))) * p.position.z
            p.position.y = (((np.tan(np.arccos(self.K[4]/(np.sqrt((self.K[4])**2-(corner_coords[i][0] - (glc.HEIGHT/2))**2))))))*p.position.z)
            p.position.x = (((np.tan(np.arccos(self.K[0]/(np.sqrt((self.K[0])**2+(corner_coords[i][1] - (glc.WIDTH/2))**2))))))*p.position.z)
            p.orientation.w = 1
            p.position.z = -self.Z
            pose_msg.poses.append(p)

            i += 1

            #p = Pose()
            #p.position.z = self.Z
            #p.position.y = (corner_coords[i][0] - self.K[4] * p.position.z)/self.K[5]
            #p.position.x = (corner_coords[i][1] - self.K[0] * p.position.z)/self.K[2]
            #p.orientation.w = 1
            #pose_msg.poses.append(p)
            
            print p
            print i

        #self.pub.publish(pose_msg)
        print 'published the message'
        self.pose_msg = pose_msg
        print 'Done.'
        
    def run(self,rate):
        rate.sleep()

def main():
    rospy.init_node('line_following')
    print 'node_spinning'
    r = ros_node()

    rate = rospy.Rate(1)
    while not rospy.is_shutdown() and not r.gathered_all_data:
        r.run(rate)
    #r.main()
    cv2.destroyAllWindows()

if __name__=="__main__":
    main()
