#!/usr/bin/env python
from pyzbar.pyzbar import decode
import rospy
from vitarana_drone.msg import edrone_cmd
from sensor_msgs.msg import NavSatFix
from vitarana_drone.srv import Gripper, GripperRequest, GripperResponse
from std_msgs.msg import String
import time
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from numpy import zeros
import numpy as np
from collections import deque
from std_msgs.msg import Float32
from std_msgs.msg import Int32
import csv



maze=zeros([501,101],int)
var=0
err=1
prevchange=1
change=1
flag=0
flag2=0

dist = 14
factor = 3.0

inside = False
condition = False
safety = 1



#********************Conversion into metre************************
def lat_to_x(input_latitude):
    return 110692.0702932625*(input_latitude-18.99930)
def long_to_x(input_longitude):
    return -105292.0089353767*(input_longitude-72.0004)


start_time=0
cur_time=0
value=0

#******Function to stop EDrone for some second at an instant*******
def stop_func(time1):
    global start_time,cur_time,value
    if value==0:
        value=1
        start_time=time.time()
    elif value==1:
        cur_time=time.time()
        if cur_time-start_time>time1:
            value=2

#***************Algorithm to find the shortest path between current and**********************
#***********************************destination coordinates***********************************
def astar(maze,start,end):
    parent=zeros([501,101,2],int)
    vis=zeros([501,101],int)
    insertq=zeros([501,101],int)
    for i in range(500):
        for j in range(100):
            parent[i][j][0]=parent[i][j][1]=-1

    q=deque()
    q.append(start)
    val=start
    insertq[start[0]][start[1]]=1

    while q:
        temp=q.popleft()
        vis[temp[0]][temp[1]]=1
        if temp[0] == end[0] and temp[1] == end[1]:
            break
        if temp[0]+1<500 and insertq[temp[0]+1][temp[1]]==0 and maze[temp[0]+1][temp[1]]==0:
            insertq[temp[0]+1][temp[1]]=1
            q.append([temp[0]+1,temp[1]])
            parent[temp[0]+1][temp[1]][0]=temp[0]
            parent[temp[0]+1][temp[1]][1]=temp[1]
        if temp[0]-1>=0 and insertq[temp[0]-1][temp[1]]==0 and maze[temp[0]-1][temp[1]]==0:
            insertq[temp[0]-1][temp[1]]=1
            q.append([temp[0]-1,temp[1]])
            parent[temp[0]-1][temp[1]][0]=temp[0]
            parent[temp[0]-1][temp[1]][1]=temp[1]
        if temp[1]-1>=0 and insertq[temp[0]][temp[1]-1]==0 and maze[temp[0]][temp[1]-1]==0:
            insertq[temp[0]][temp[1]-1]=1
            q.append([temp[0],temp[1]-1])
            parent[temp[0]][temp[1]-1][0]=temp[0]
            parent[temp[0]][temp[1]-1][1]=temp[1]
        if temp[1]+1<100 and insertq[temp[0]][temp[1]+1]==0 and maze[temp[0]][temp[1]+1]==0:
            insertq[temp[0]][temp[1]+1]=1
            q.append([temp[0],temp[1]+1])
            parent[temp[0]][temp[1]+1][0]=temp[0]
            parent[temp[0]][temp[1]+1][1]=temp[1]
        
    
    temp=[end[0],end[1]]
    q1=deque()
    path=[]

    while temp[0]>=0 :
        q1.append(temp)
        temp=parent[temp[0]][temp[1]]

    #****************RETURNING PATH*******************************
    while q1:
        temp=q1.pop()
        path.append(temp)

    print(len(path))

    return path    
        
#***********************Funcion to call path generating function*******************
def operate_Astar(cur_x,cur_y,dest_x,dest_y,cur_obs,drone_x,drone_y):
    print("INSIDE INSIDE INSIDE \n\n")
    global prevchange,err
    global maze
    global inside
    global factor, safety
    inside = True
    start=((int)(cur_x/factor),(int)(cur_y/factor))
    end=((int)(dest_x/factor),(int)(dest_y/factor))
    k1=(int)(drone_x/factor)
    k2=(int)(drone_y/factor)

    print(cur_obs)
    
    #****************Marking obstacles in grid**************************

    #For obstacles in front
    if cur_obs[3]>0.5 and cur_obs[3]<26:
        start_x = (int)((drone_x-cur_obs[3])/factor)-safety
        end_x = (int)((drone_x-cur_obs[3])/factor)+safety
        start_y = k2 - safety
        end_y = k2 + safety
        i = start_x
        while i <= end_x:
            j = start_y
            while j <= end_y:
                maze[i][j]=1
                j=j+1
            i=i+1  

    print("down1\n")

    #For obstacles in back
    if cur_obs[1]>0.5 and cur_obs[1]<26:
        start_x = (int)((drone_x+cur_obs[1])/factor)-safety
        end_x = (int)((drone_x+cur_obs[1])/factor)+safety
        start_y = k2 - safety
        end_y = k2 + safety
        i = start_x
        while i <= end_x:
            j = start_y
            while j <= end_y:
                maze[i][j]=1
                j=j+1
            i=i+1   
    print("down2\n")

    #For obstacles in left    
    if cur_obs[2] > 0.5 and cur_obs[2] < 26:
        start_y=(int)((drone_y-cur_obs[2])/factor)-safety
        end_y = (int)((drone_y-cur_obs[2])/factor)+safety
        start_x = k1-safety
        end_x = k1+safety
        print(start_x,end_x,start_y,end_y)
        i=start_x
        while i<= end_x:
            j=start_y
            while j<= end_y:
                maze[i][j]=1
                j=j+1
            i=i+1

    print("down3\n")

    #For obstacles in right        
    if cur_obs[0]>2 and cur_obs[0] < 26:
        start_y=(int)((drone_y+cur_obs[0])/factor)-safety
        end_y = (int)((drone_y+cur_obs[0])/factor)+safety
        start_x = k1 - safety
        end_x = k1 + safety
        i=start_x
        while i <= end_x :
            j=start_y
            while j<= end_y:
                maze[i][j]=1
                j=j+1
            i=i+1  

    print("down4\n")


    #Marking the current location of EDrone as safe in case it is marked as blocked
    maze[start[0]][start[1]] = 0
   

    
    print("printinf obstacles\n")
    for i in range(500):
        for j in range(100):
            if maze[i][j]==1:
                print(i,j)
    print("completed\n")

    
    #Calling the Astar function    
    path=astar(maze,start,end)
    print(path)
    print("\n\n")
    global flag,flag2
    flag3=0   
    
    #Returning the next subsequent location for reaching the destination
    if(len(path)>1):
        final_dest= (factor*path[1][0],factor*path[1][1])
    else:
        if flag==0:
            flag=1
        else:
            flag2=1
        final_dest= (dest_x,dest_y)


    # Checking that the direction of motion has changed
    if final_dest[0]!=cur_x:
        global change
        change = 1
    else:
        global change
        change = 0

    #Ensuring that if direction has change, the Edrone must handle it better
    global change
    if prevchange == change:
        err = 1
    else:
        err = 0.1

    prevchange = change
    inside = False

    return final_dest

def clear_maze():
    global maze
    for i in range(500):
        for j in range(100):
            maze[i][j]=0


class Position():
    def __init__(self):
        global dist
        #Initialising Node
        rospy.init_node('position_controller', anonymous=True)
        #Variable for current position of Drone
        self.pos = [0,0,0]
        #Variable tht sets the target position 
        self.set_pos = [lat_to_x(19),long_to_x(72), 15] 
        self.nxt_lat=self.set_pos[0]
        self.nxt_long=self.set_pos[1]
        self.gripp_activate = rospy.ServiceProxy('/edrone/activate_gripper', Gripper)
        self.check_result = False

        #shift it        
        #Publisher to Publish velocity 
        self.velocity_publisher = rospy.Publisher('/edrone/drone_command', edrone_cmd, queue_size=10)
        #Subscriber to get the current Position of Drone
        rospy.Subscriber("/edrone/gps", NavSatFix, self.PoseCallBack)
        rospy.Subscriber("/edrone/range_finder_top",LaserScan,self.obstacle_check)
        rospy.Subscriber("/edrone/range_finder_bottom",LaserScan,self.bottom_height)
        rospy.Subscriber("/edrone/gripper_check", String, self.gripp_check)
        self.loop_break = rospy.Publisher("/edrone/loop_break", String,queue_size=10)
        self.building_id = rospy.Publisher("/edrone/curr_marker_id",Int32,queue_size=10)
        self.x_err = rospy.Publisher("/edrone/err_x_m",Float32,queue_size=10)
        self.y_err = rospy.Publisher("/edrone/err_y_m",Float32,queue_size=10)
        self.image_sub = rospy.Subscriber("/edrone/camera/image_raw", Image, self.image_callback) 
        self.logo_cascade = cv2.CascadeClassifier('/home/aman/catkin_ws/src/vitarana_drone/scripts/data/cascade.xml')
        self.img = np.empty([])
        self.bridge = CvBridge()
        self.x_cascade = 0
        self.y_cascade = 0
        self.w_cascade = 0
        self.h_cascade = 0
        self.decoded_data=""
        self.height = 0
        

    
    
    #Callback function to update current Position
    def PoseCallBack(self,msg):
        self.pos[0] = lat_to_x(msg.latitude)
        self.pos[1] = long_to_x(msg.longitude)
        self.pos[2] = msg.altitude

    def gripp_check(self,data):
        self.check_result = data.data

    def bottom_height(self,data_bottom):
        self.height = data_bottom.ranges[0]

    #Callback function to check obstacles
    def obstacle_check(self,range):
        self.obs = range.ranges

    #Callback function for cascade detection
    def image_callback(self,data):
        try:
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8") # Converting the image to OpenCV standard image
            try:
                gray = cv2.cvtColor(self.img,cv2.COLOR_BGR2GRAY)
                logo = self.logo_cascade.detectMultiScale(gray, scaleFactor=1.05)
                for(x,y,w,h) in logo:
                    if(w*h>10000):
                        continue
                    self.x_cascade = x
                    self.y_cascade = y
                    self.w_cascade = w
                    self.h_cascade = h
                    break
                # print(self.x_cascade," ",self.y_cascade," ",self.w_cascade," ",self.h_cascade,"\n")
            except:
                pass
        except CvBridgeError as e:
            print(e)
            return
    


    #Initialisation of differnt variables and parameters
    def command(self):
        self.obs = [0,0,0,0,0]
        self.Iter = 0
        self.prev_err = 0
        self.lat_prev_err = 0
        self.lat_iter = 0
        self.long_iter = 0
        self.long_prev_err = 0
        self.throttle = 1510.0
        self.check_val0 = 1
        self.check_val1 = 0
        self.check_val2 = 0
        self.check_val3 = 0
        self.check_val4 = 0
        self.check_val5 = 0
        self.check_val6 = 0
        self.check_val7 = 0
        self.check_val8 = 0
        self.check_val9 = 0
        self.check_val10 = 0
        self.check_val11 = 0
        self.check_val12 = 0
        self.check_val13 = 0
        self.check_val14 = 0
        self.check_val15 = 0
        self.check_val16 = 0
        self.check_val17 = 0
        self.check_val18 = 0
        self.check_val19 = 0
        self.check_val20 = 0
        self.check_val21 = 0
        self.check_val22 = 0
        self.check_val23 = 0
        self.check_val24 = 0
        self.check_val25 = 0
        self.check_val26 = 0
        self.check_val27 = 0
        self.check_val28 = 0
        self.check_val29 = 0
        self.check_val30 = 0
        self.check_val30 = 0
        self.check_val31 = 0
        self.check_val32 = 0
        self.check_val33 = 0
        self.check_val34 = 0
        self.check_val35 = 0
        self.drone_cmd =edrone_cmd()
        self.loop_rate = rospy.Rate(10)
        self.dest_lat = 0
        self.dest_long = 0
        self.dest_alt = 0
        self.x_mov = 0
        self.y_mov = 0
        self.focal_length = 238.350719
        self.building_no = 1


        while True :
            global dist          
            global start_time,cur_time,value,condition
            with open ("/home/aman/catkin_ws/src/vitarana_drone/scripts/manifest.csv") as myfile:
                reader = csv.reader(myfile)
                data = list(reader)
            print (data)
            
            #-------------------------Altitude PID-----------------------------
            self.pose_err = self.set_pos[2] -self. pos[2]
            self.Iter = (self.Iter + self.pose_err)*0.35

            #Updating new Throttle value depending upon required increase or decrease in altitude 

            self.drone_cmd.rcThrottle =  (self.pose_err)*130 + (self.Iter)+ (self.pose_err-self.prev_err)*1410 + self.throttle
            self.prev_err = self.pose_err


            #Checking and calling the OPERATE ASTAR function when current expected location is reached
            if condition == True:
                global flag,flag2
                lat_error=abs(self.nxt_lat-self.pos[0])
                long_error=abs(self.nxt_long-self.pos[1])
                global err
                if lat_error <= err and long_error <= err:
                    val=operate_Astar(self.nxt_lat,self.nxt_long,self.set_pos[0],self.set_pos[1],self.obs,self.pos[0],self.pos[1])                            
                    self.nxt_lat = val[0]
                    self.nxt_long = val[1]
                        

            #--------------------------Latitude PID-----------------------------

            self.lat_err = (self.nxt_lat - self.pos[0])  #current error in latitude

            self.lat_iter=(self.lat_iter+self.lat_err)*0.4 #integral error in latitude

            #Updating new Pitch value depending upon required orientation 
            self.drone_cmd.rcPitch = 1500 + self.lat_err*400 +self.lat_iter + (self.lat_err-self.lat_prev_err)*3500  

            self.lat_prev_err=self.lat_err

            

                

            #--------------------------Longitude PID-----------------------------

            self.long_err = (self.nxt_long - self.pos[1]) #current error in longitude

            self.long_iter=(self.long_iter+self.long_err)*0.4 #integral error in longitude

            #Updating new Roll value depending upon required orientation

            self.drone_cmd.rcRoll = 1500 - (self.long_err*400 +self.long_iter + (self.long_err-self.long_prev_err)*3500)

            self.long_prev_err = self.long_err

            self.prev_loc = self.pos   


            if self.check_val0 == 1 and self.check_val1==0 and abs(self.pos[2]-self.set_pos[2])<=0.2:
                stop_func(1)
                print("inside 1\n")
                if value ==2:
                    value=0
                    condition = True
                    clear_maze()
                    self.set_pos[0] = lat_to_x(18.9999864489) + 1.5
                    self.set_pos[1] = long_to_x(71.9999430161) - 1.5
                    self.check_val1 = 1

            # if self.check_val1==1 and self.check_val2==0 and abs(self.set_pos[0]-self.pos[0])<=0.1:
            #     stop_func(1)
            #     if value == 2:
            #         value =0
            #         self.check_val2=1

            if self.check_val1==1 and self.check_val3==0 and abs(self.set_pos[1]-self.pos[1])<=0.005 and abs(self.set_pos[0]-self.pos[0])<=0.005:
                stop_func(1)
                if value == 2:
                    value = 0
                    self.set_pos[2]=8
                    self.check_val3=1
                    condition = False

            if self.check_val3==1 and self.check_val4==0 and (self.check_result==True or self.check_result=="True"):
                stop_func(1)
                self.gripp_activate(True)
                if value == 2:
                    value = 0 
                    self.check_val4 = 1
                    #FIRST BUILDING HEIGHT
                    self.set_pos[2] = (float)(data[0][3]) + dist + 0.4
            
            if self.check_val4==1 and self.check_val5==0 and abs(self.set_pos[2]-self.pos[2])<=0.2:
                stop_func(1)
                if value == 2:
                    value = 0
                    self.check_val5 = 1
                    clear_maze()
                    condition = True
                    #FIRST BUILDING COORINATE
                    self.set_pos[0] = lat_to_x((float)(data[0][1]))
                    self.set_pos[1] = long_to_x((float)(data[0][2]))

            # if self.check_val5==1 and self.check_val6==0 and abs(self.set_pos[0]-self.pos[0])<0.1:
            #     stop_func(1)
            #     if value == 2:
            #         value = 0
            #         self.check_val6 = 1

            if self.check_val5==1 and self.check_val7 == 0 and abs(self.set_pos[1]-self.pos[1])<=0.005 and abs(self.set_pos[0]-self.pos[0])<=0.005:
                stop_func(1)
                if value == 2:
                    value = 0
                    self.check_val7=1
                    x_val=self.x_cascade+self.w_cascade/2
                    x_val = x_val - 200                  
                    x_val = x_val * dist
                    x_val = x_val/self.focal_length
                    y_val=self.y_cascade+self.h_cascade/2
                    y_val = y_val - 200
                    
                    #Setting the marker locations as the new coordinates
                    
                    y_val = y_val * dist
                    y_val = y_val/self.focal_length
                    self.set_pos[0] = self.set_pos[0] + x_val
                    self.set_pos[1] = self.set_pos[1] - y_val

            # if self.check_val7==1 and self.check_val8==0 and abs(self.set_pos[0]-self.pos[0])<0.1:
            #     stop_func(1)
            #     if value == 2:
            #         value =0
            #         self.check_val8=1

            if self.check_val7==1 and self.check_val9==0 and abs(self.set_pos[1]-self.pos[1])<=0.005 and abs(self.set_pos[0]-self.pos[0])<=0.005:
                stop_func(1)
                if value == 2:
                    self.check_val9=1
                    value = 0
                    print(self.set_pos[2])
                    self.set_pos[2] = self.set_pos[2] - dist
                    print(self.set_pos[2])
                    condition = False
            
            if self.check_val9==1 and self.check_val10==0 and abs(self.set_pos[2]-self.pos[2])<0.2:
                stop_func(1)
                if value == 2:
                    value=0
                    self.check_val10=1
                    self.gripp_activate(False)
                    self.set_pos[2] = 8 + dist 

            if self.check_val10 == 1 and self.check_val11 == 0 and abs(self.set_pos[2]-self.pos[2])<=0.2:
                stop_func(1)
                if value == 2:
                    value = 0
                    self.check_val11 = 1
                    clear_maze()
                    condition = True
                    self.set_pos[0] = lat_to_x(18.9999864489) 
                    self.set_pos[1] = long_to_x(71.9999430161)

            # if self.check_val11 ==1 and self.check_val12==0 and abs(self.set_pos[0]-self.pos[0])<=0.1:
            #     stop_func(1)
            #     if value == 2:
            #         value = 0
            #         self.check_val12 = 1

            if self.check_val11==1 and self.check_val13==0 and abs(self.set_pos[1]-self.pos[1])<=0.005 and abs(self.set_pos[0]-self.pos[0])<=0.005:
                stop_func(1)
                if value == 2:
                    value = 0
                    self.check_val13 = 1
                    condition = False
                    self.set_pos[2] = self.set_pos[2] - dist

            if self.check_val13 == 1 and self.check_val14 == 0 and (self.check_result==True or self.check_result=="True"):
                stop_func(1)
                self.gripp_activate(True)
                if value == 2:
                    value = 0
                    self.check_val14 = 1
                    #SECOND BUILDING HEIGHT
                    self.set_pos[2] = (float)(data[1][3]) + dist

            if self.check_val14 == 1 and self.check_val15 == 0 and abs(self.set_pos[2]-self.pos[2])<=0.2:
                stop_func(1)
                if value == 2:
                    value =0
                    self.check_val15=1
                    clear_maze()
                    condition = True
                    #SECOND BUILDING COORDINATES
                    self.set_pos[0] = lat_to_x((float)(data[1][1]))
                    self.set_pos[1] = long_to_x((float)(data[1][2]))

            # if self.check_val15==1 and self.check_val16==0 and abs(self.set_pos[0]-self.pos[0])<=0.1:
            #     stop_func(1)
            #     if value == 2:
            #         value =0
            #         self.check_val16=1


            if self.check_val15==1 and self.check_val17==0 and abs(self.set_pos[1]-self.pos[1])<=0.005 and abs(self.set_pos[0]-self.pos[0])<=0.005:
                stop_func(1)
                if value == 2:
                    value =0
                    self.check_val17=1
                    x_val=self.x_cascade+self.w_cascade/2
                    x_val = x_val - 200                  
                    x_val = x_val * dist
                    x_val = x_val/self.focal_length
                    y_val=self.y_cascade+self.h_cascade/2
                    y_val = y_val - 200
                    
                    #Setting the marker locations as the new coordinates
                    
                    y_val = y_val * dist
                    y_val = y_val/self.focal_length
                    self.set_pos[0] = self.set_pos[0] + x_val
                    self.set_pos[1] = self.set_pos[1] - y_val

            # if self.check_val17==1 and self.check_val18==0 and abs(self.set_pos[0] - self.pos[0])<=0.1:
            #     stop_func(1)
            #     if value == 2:
            #         value = 0
            #         self.check_val18 = 1 

            if self.check_val17 == 1 and self.check_val19 == 0 and abs(self.set_pos[1]-self.pos[1])<=0.005 and abs(self.set_pos[0]-self.pos[0])<=0.005:
                stop_func(1)
                if value == 2:
                    value = 0
                    self.check_val19 = 1
                    condition = False
                    self.set_pos[2] = self.set_pos[2] - dist

            if self.check_val19==1 and self.check_val20==0 and abs(self.set_pos[2]-self.pos[2])<=0.2:
                stop_func(1)
                if value ==2:
                    value =0
                    self.check_val20=1
                    self.gripp_activate(False)
                    self.set_pos[2] = 8 + dist


            if self.check_val20 == 1 and self.check_val21 == 0 and abs(self.set_pos[2]-self.pos[2])<=0.2:
                stop_func(1)
                if value == 2:
                    value = 0
                    self.check_val21 = 1
                    clear_maze()
                    condition = True
                    self.set_pos[0] = lat_to_x(18.9999864489) + 3
                    self.set_pos[1] = long_to_x(71.9999430161) 

            # if self.check_val21 ==1 and self.check_val22==0 and abs(self.set_pos[0]-self.pos[0])<=0.1:
            #     stop_func(1)
            #     if value == 2:
            #         value = 0
            #         self.check_val22 = 1

            if self.check_val21==1 and self.check_val23==0 and abs(self.set_pos[1]-self.pos[1])<=0.005 and abs(self.set_pos[0]-self.pos[0])<=0.005:
                stop_func(1)
                if value == 2:
                    value =0
                    self.check_val23 = 1
                    condition = False
                    self.set_pos[2] = self.set_pos[2] - dist

            if self.check_val23 == 1 and self.check_val24 == 0 and (self.check_result==True or self.check_result=="True"):
                stop_func(1)
                self.gripp_activate(True)
                if value == 2:
                    value = 0
                    self.check_val24 = 1
                    #THIRD BUILDING HEIGHT
                    self.set_pos[2] = (float)(data[2][3]) + dist 

            if self.check_val24 == 1 and self.check_val25 == 0 and abs(self.set_pos[2]-self.pos[2])<=0.2:
                stop_func(1)
                if value == 2:
                    value =0
                    self.check_val25=1
                    clear_maze()
                    condition = True
                    #THIRD BUILDING COORDINATES
                    self.set_pos[0] = lat_to_x((float)(data[2][1]))
                    self.set_pos[1] = long_to_x((float)(data[2][2]))
                    

            # if self.check_val25==1 and self.check_val26==0 and abs(self.set_pos[0]-self.pos[0])<=0.1:
            #     stop_func(1)
            #     if value == 2:
            #         value =0
            #         self.check_val26=1


            if self.check_val25==1 and self.check_val27==0 and abs(self.set_pos[1]-self.pos[1])<=0.005 and abs(self.set_pos[0]-self.pos[0])<=0.005:
                stop_func(1)
                if value == 2:
                    value =0
                    self.check_val27=1
                    x_val=self.x_cascade+self.w_cascade/2
                    x_val = x_val - 200                  
                    x_val = x_val * dist
                    x_val = x_val/self.focal_length
                    y_val=self.y_cascade+self.h_cascade/2
                    y_val = y_val - 200
                    
                    #Setting the marker locations as the new coordinates
                    
                    y_val = y_val * dist
                    y_val = y_val/self.focal_length
                    self.set_pos[0] = self.set_pos[0] + x_val
                    self.set_pos[1] = self.set_pos[1] - y_val

            # if self.check_val27==1 and self.check_val28==0 and abs(self.set_pos[0] - self.pos[0])<=0.1:
            #     stop_func(1)
            #     if value == 2:
            #         value = 0
            #         self.check_val28= 1 

            if self.check_val27==1 and self.check_val29==0 and abs(self.set_pos[1]-self.pos[1])<=0.005 and abs(self.set_pos[0]-self.pos[0])<=0.005:
                stop_func(1)
                if value == 2:
                    value = 0
                    self.check_val29 = 1
                    condition = False
                    self.set_pos[2] = self.set_pos[2] - dist
                    dist = dist + 5

            if self.check_val29==1 and self.check_val30==0 and abs(self.set_pos[2]-self.pos[2])<=0.2:
                stop_func(1)
                if value ==2:
                    value =0
                    self.check_val30=1
                    self.gripp_activate(False)
                    self.set_pos[2] = 8.44 + dist 

            if self.check_val30==1 and self.check_val31==0 and abs(self.set_pos[2]-self.pos[2])<=0.2:
                stop_func(1)
                if value == 2:
                    value=0
                    self.check_val31=1
                    condition=True
                    self.set_pos[0] = lat_to_x(19)
                    self.set_pos[1] = long_to_x(72)

            if self.check_val31==1 and self.check_val32==0 and abs(self.set_pos[0]-self.pos[0])<=0.005 and abs(self.set_pos[1] - self.pos[1])<=0.005:
                stop_func(1)
                if value == 2:
                    value = 0
                    condition=False
                    self.check_val32 = 1
                    self.set_pos[2] = self.set_pos[2] - dist

            if self.check_val32==1 and self.check_val33==0 and abs(self.set_pos[2]-self.pos[2])<=0.2:
                stop_func(1)
                if value==2:
                    value=0
                    self.check_val33 = 1
                    self.loop_break.publish("stop")
                    self.drone_cmd.rcThrottle = 1000   
                    

            self.drone_cmd.rcYaw = 1500 
            if self.drone_cmd.rcRoll < 1000:
                self.drone_cmd.rcRoll = 1000
            if self.drone_cmd.rcRoll > 2000:
                self.drone_cmd.rcRoll = 2000
            if self.drone_cmd.rcPitch < 1000:
                self.drone_cmd.rcPitch = 1000
            if self.drone_cmd.rcPitch > 2000:
                self.drone_cmd.rcPitch = 2000


            
                

            
                          
                           
            #publishing the velocity
            self.velocity_publisher.publish(self.drone_cmd) 
            self.loop_rate.sleep()
            print(self.pos)
            print(self.set_pos)
            print(self.nxt_lat,self.nxt_long)
            print("self.height = ",self.height)
            print("\n\n\n")
            # and breaking the loop , as it's posted now
            if(self.check_val33 == 1):  
                print("loop breaked")
                break



if __name__ == "__main__":
    try:
        publish_cmd = Position()
        time.sleep(20)
        publish_cmd.command()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")
        

    

    
