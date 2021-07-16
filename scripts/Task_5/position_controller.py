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


#*****************************GLOBAL VARIABLES****************************
maze=zeros([501,101],int)
var=0
err=4
prevchange=1
change=1
flag=0
flag2=0
dist = 10
factor = 10.0
inside = False
condition = False
safety = 1




#********************Conversion into metre************************
def lat_to_x(input_latitude):
    return 110692.0702932625*(input_latitude-18.9983)
def long_to_x(input_longitude):
    return -105292.0089353767*(input_longitude-72.002)

a1_lat = 18.9998102845
a1_long = 72.000142461
same_alt = 16.757981
x1_lat = 18.9999367615
x1_long = 72.000142461 


start_time=0
cur_time=0
value=0

#******Function to stop EDrone for some second at an instant*******
def stop_func(time1):
    global start_time,cur_time,value
    if value==0:
        value=1
        start_time=time.time()
        if time1==0:
            value=2
    elif value==1:
        cur_time=time.time()
        if cur_time-start_time>=time1:
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

        if temp[0]+1<500 and temp[1]+1<100 and insertq[temp[0]+1][temp[1]+1]==0 and maze[temp[0]+1][temp[1]+1]==0:
            insertq[temp[0]+1][temp[1]+1]=1
            q.append([temp[0]+1,temp[1]+1])
            parent[temp[0]+1][temp[1]+1][0]=temp[0]
            parent[temp[0]+1][temp[1]+1][1]=temp[1]

        if temp[0]+1<500 and temp[1]-1>=0 and insertq[temp[0]+1][temp[1]-1]==0 and maze[temp[0]+1][temp[1]-1]==0:
            insertq[temp[0]+1][temp[1]-1]=1
            q.append([temp[0]+1,temp[1]-1])
            parent[temp[0]+1][temp[1]-1][0]=temp[0]
            parent[temp[0]+1][temp[1]-1][1]=temp[1]

        if temp[0]-1>=0 and temp[1]+1<100 and insertq[temp[0]-1][temp[1]+1]==0 and maze[temp[0]-1][temp[1]+1]==0:
            insertq[temp[0]-1][temp[1]+1]=1
            q.append([temp[0]-1,temp[1]+1])
            parent[temp[0]-1][temp[1]+1][0]=temp[0]
            parent[temp[0]-1][temp[1]+1][1]=temp[1]

        if temp[0]-1>=0 and temp[1]-1>=0 and insertq[temp[0]-1][temp[1]-1]==0 and maze[temp[0]-1][temp[1]-1]==0:
            insertq[temp[0]-1][temp[1]-1]=1
            q.append([temp[0]-1,temp[1]-1])
            parent[temp[0]-1][temp[1]-1][0]=temp[0]
            parent[temp[0]-1][temp[1]-1][1]=temp[1]
        
    
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

    return path    
        
#***********************Funcion to call path generating function*******************
def operate_Astar(cur_x,cur_y,dest_x,dest_y,cur_obs,drone_x,drone_y):
    global prevchange,err
    global maze
    global inside
    global factor, safety
    inside = True
    start=((int)(cur_x/factor),(int)(cur_y/factor))
    end=((int)(dest_x/factor),(int)(dest_y/factor))
    k1=(int)(drone_x/factor)
    k2=(int)(drone_y/factor)

    
    
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


    #For obstacles in left    
    if cur_obs[2] > 0.5 and cur_obs[2] < 26:
        start_y=(int)((drone_y-cur_obs[2])/factor)-safety
        end_y = (int)((drone_y-cur_obs[2])/factor)+safety
        start_x = k1-safety
        end_x = k1+safety
        i=start_x
        while i<= end_x:
            j=start_y
            while j<= end_y:
                maze[i][j]=1
                j=j+1
            i=i+1

  

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

    


    #Marking the current location of EDrone as safe in case it is marked as blocked
    maze[start[0]][start[1]] = 0 

    
    #Calling the Astar function    
    path=astar(maze,start,end)
    global flag,flag2
    flag3=0   
    
    #Returning the next subsequent location for reaching the destination
    if(len(path)>1):
        if(path[1][0]==end[0] and path[1][1]==end[1]):
            final_dest =(dest_x,dest_y)
        else:
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
        err = 4
    else:
        err = 4

    prevchange = change
    inside = False

    return final_dest

def clear_maze():
    global maze
    for i in range(500):
        for j in range(100):
            maze[i][j]=0


#function to return distance
def calc(start_pos_x,start_pos_y,end_pos_x,end_pos_y):
    return (start_pos_x-end_pos_x)*(start_pos_x-end_pos_x)+(start_pos_y-end_pos_y)*(start_pos_y-end_pos_y)


#return nearest parcel as per current location    
def schedule(parcel_no,ret_arr,x_pos,y_pos,parcel_x,parcel_y):
    float_dist  = 100000000
    i=0
    j=0
    while i<9:
        new_dist = calc(x_pos,y_pos,parcel_x[ret_arr[i]],parcel_y[ret_arr[i]])
        if new_dist<float_dist:
            float_dist = new_dist
            j=ret_arr[i]
        i = i+1
    return j


#Whole Scheduler Algorithm
def prep_schedule(del_arr,ret_arr,parcel_size,initial_x,initial_y,start_x,start_y,end_x,end_y):
    i=0    
    loc_arr = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
    start_x1 = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
    start_y1 = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
    end_x1 = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
    end_y1 = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
    i=0
    while i<parcel_size:
        start_x1[i] = start_x[i]
        end_x1[i] = end_x[i]
        start_y1[i] = start_y[i]
        end_y1[i] = end_y[i]
        i=i+1
    i=0
    while i<parcel_size:
        if(i%2==0):
            loc_arr[i] = schedule(i,del_arr,initial_x,initial_y,end_x1,end_y1)
        else:
            loc_arr[i] = schedule(i,ret_arr,end_x[loc_arr[i-1]],end_y[loc_arr[i-1]],start_x1,start_y1)
        start_x1[loc_arr[i]]=10000
        end_x1[loc_arr[i]]=10000
        start_y1[loc_arr[i]]=10000
        end_y1[loc_arr[i]]=10000
        i=i+1
    return loc_arr




class Position():
    def __init__(self):
        global dist,x1_lat,x1_long,a1_lat,a1_long,same_alt
        #Initialising Node
        rospy.init_node('position_controller', anonymous=True)
        #Variable for current position of Drone
        self.pos = [0,0,0]
        #Variable tht sets the target position 
        self.set_pos = [lat_to_x(18.9998887906),long_to_x(72.0002184402), 16.7579714802+dist] 
        self.nxt_lat=self.set_pos[0]
        self.nxt_long=self.set_pos[1]
        self.parcel_x = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
        self.parcel_y = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
        self.parcel_alt=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
        self.final_dest_x=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
        self.final_dest_y=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
        self.final_dest_alt=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
        
        self.gripp_activate = rospy.ServiceProxy('/edrone/activate_gripper', Gripper)
        self.check_result = False
        self.del_arr=[0,0,0,0,0,0,0,0,0]
        self.ret_arr=[0,0,0,0,0,0,0,0,0]
        self.loc_arr=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]

        #shift it        
        #Publisher to Publish velocity 
        self.velocity_publisher = rospy.Publisher('/edrone/drone_command', edrone_cmd, queue_size=10)
        #Subscriber to get the current Position of Drone
        rospy.Subscriber("/edrone/gps", NavSatFix, self.PoseCallBack)
        rospy.Subscriber("/edrone/range_finder_top",LaserScan,self.obstacle_check)
        rospy.Subscriber("/edrone/range_finder_bottom",LaserScan,self.bottom_height)
        rospy.Subscriber("/edrone/gripper_check", String, self.gripp_check)
        self.loop_break = rospy.Publisher("/edrone/loop_break", String,queue_size=10)
        self.image_sub = rospy.Subscriber("/edrone/camera/image_raw", Image, self.image_callback) 
        self.logo_cascade = cv2.CascadeClassifier('/home/aman/catkin_ws/src/vitarana_drone/scripts/data/cascade.xml')
        self.img = np.empty([])
        self.bridge = CvBridge()
        self.x_cascade = 0
        self.y_cascade = 0
        self.w_cascade = 0
        self.h_cascade = 0
        self.decoded_data = ""
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
                logo = self.logo_cascade.detectMultiScale(gray, scaleFactor=1.07)
                for(x,y,w,h) in logo:
                    if(w*h>10000):
                        continue
                    self.x_cascade = x
                    self.y_cascade = y
                    self.w_cascade = w
                    self.h_cascade = h
                    break
                
            except:
                pass
        except CvBridgeError as e:
            print(e)
            return
    


    #Initialisation of differnt variables and parameters
    def command(self):
        self.obs = [0,0,0,0,0]
        self.parcel_no = 0
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
        self.drone_cmd =edrone_cmd()
        self.loop_rate = rospy.Rate(10)
        self.dest_lat = 0
        self.dest_long = 0
        self.dest_alt = 0
        self.x_mov = 0
        self.y_mov = 0
        self.focal_length = 238.350719
        self.building_no = 1
        self.detect_marker = 1
        self.initial_x = lat_to_x(18.9998887906)
        self.initial_y = long_to_x(72.0002184402)
        with open ("/home/aman/catkin_ws/src/vitarana_drone/scripts/manifest.csv") as myfile:
                reader = csv.reader(myfile)
                data = list(reader)
        i=0
        self.parcel_size = len(data)        
        i=0
        k1=0
        k2=0

        #Preparing List of all Scheduled Delivery and Scheduled Return 
        while i<len(data):
            if(data[i][0]=='DELIVERY'):
                self.del_arr[k1] = i
                k1 = k1+1
                self.parcel_x[i] = lat_to_x(a1_lat)+((ord(data[i][1][0])-ord('A')))*1.5
                self.parcel_y[i] = long_to_x(a1_long)-((ord(data[i][1][1])-ord('1')))*1.5
                self.parcel_alt[i] = same_alt
                split_data = data[i][2].split(";")
                self.final_dest_x[i] = lat_to_x((float)(split_data[0]))
                self.final_dest_y[i] = long_to_x((float)(split_data[1]))
                self.final_dest_alt[i] =(float)(split_data[2])
                if i==10:
                    self.final_dest_alt[i] = 17.87
            else:
                self.ret_arr[k2] = i
                k2 = k2+1
                split_data = data[i][1].split(";")
                self.parcel_x[i] = lat_to_x((float)(split_data[0]))
                self.parcel_y[i] = long_to_x((float)(split_data[1]))
                self.parcel_alt[i] =(float)(split_data[2])
                self.final_dest_alt[i] = same_alt
                self.final_dest_x[i] = lat_to_x(x1_lat)+(ord(data[i][2][0])-ord('X'))*1.5
                self.final_dest_y[i] = long_to_x(x1_long)-(ord(data[i][2][1])-ord('1'))*1.5
            i=i+1
        self.parcel_size = len(data)
        #Calling Scheduling Algo that returns List
        self.loc_arr = prep_schedule(self.del_arr,self.ret_arr,self.parcel_size,self.initial_x,self.initial_y,self.parcel_x,self.parcel_y,self.final_dest_x,self.final_dest_y)        
        self.next_ind = self.loc_arr[0]       
        



        while True :
            global dist
            global start_time,cur_time,value,condition            
            
            #-------------------------Altitude PID-----------------------------
            self.pose_err = self.set_pos[2] -self. pos[2]
            self.Iter = (self.Iter + self.pose_err)*0.35

            #Updating new Throttle value depending upon required increase or decrease in altitude 

            self.drone_cmd.rcThrottle =  (self.pose_err)*130 + (self.Iter)+ (self.pose_err-self.prev_err)*1410 + self.throttle
            self.prev_err = self.pose_err
            
                        

            #--------------------------Latitude PID-----------------------------

            
            self.lat_err = (self.set_pos[0] - self.pos[0])  #current error in latitude

            self.lat_iter=(self.lat_iter+self.lat_err)*0.9 #integral error in latitude

            #Updating new Pitch value depending upon required orientation 
            if(abs(self.set_pos[0]-self.pos[0])<=10):
                self.drone_cmd.rcPitch = 1500 + self.lat_err*270 +self.lat_iter + (self.lat_err-self.lat_prev_err)*4290  
            else:
                self.drone_cmd.rcPitch = 1500 + self.lat_err*235 +self.lat_iter + (self.lat_err-self.lat_prev_err)*5800  

            

            self.lat_prev_err=self.lat_err         

                

            #--------------------------Longitude PID-----------------------------

            self.long_err = (self.set_pos[1] - self.pos[1]) #current error in longitude

            self.long_iter=(self.long_iter+self.long_err)*0.9 #integral error in longitude

            #Updating new Roll value depending upon required orientation
            if(abs(self.set_pos[1]-self.pos[1])<=10):
                self.drone_cmd.rcRoll = 1500 - (self.long_err*270 +self.long_iter + (self.long_err-self.long_prev_err)*4290)
            else:
                self.drone_cmd.rcRoll = 1500 - (self.long_err*235 +self.long_iter + (self.long_err-self.long_prev_err)*5800)
                

            self.long_prev_err = self.long_err

            self.prev_loc = self.pos   


            #**************Heading Towards Parcel*********************
            if self.check_val0 == 1 and self.check_val1==0 and abs(self.pos[2]-self.set_pos[2])<=4:
                stop_func(0)
                if value ==2:
                    value=0
                    condition = True
                    clear_maze()
                    self.set_pos[0] = self.parcel_x[self.next_ind]
                    self.set_pos[1] = self.parcel_y[self.next_ind]
                    self.check_val1 = 1


            if self.check_val1==1 and self.check_val2==0 and abs(self.set_pos[1]-self.pos[1])<=0.05 and abs(self.set_pos[0]-self.pos[0])<=0.05:
                stop_func(1)
                if value == 2:
                    value = 0
                    self.set_pos[2]=self.set_pos[2] - dist - 0.35
                    self.check_val2=1
                    condition = False

            if self.check_val2==1 and self.check_val3==0 and (self.check_result==True or self.check_result=="True"):
                stop_func(0)
                self.gripp_activate(True)
                if value == 2:
                    value = 0 
                    self.check_val3 = 1
                    self.set_pos[2] = self.final_dest_alt[self.next_ind] + dist 
                    
                   


            #*****************Moving Towards Building**********************
            if self.check_val3==1 and self.check_val4==0 and abs(self.set_pos[2]-self.pos[2])<=4:
                stop_func(0)
                if value == 2:
                    value = 0
                    self.check_val4 = 1
                    clear_maze()
                    condition = True
                    #BUILDING COORINATE
                    self.set_pos[0] = self.final_dest_x[self.next_ind]
                    self.set_pos[1] = self.final_dest_y[self.next_ind]
                    if data[self.next_ind][0] == 'RETURN ':
                        self.check_val5=1
                    


            #*********************Detecting building Marker**********************
            if self.check_val4==1 and self.check_val5 == 0 and abs(self.set_pos[1]-self.pos[1])<=0.05 and abs(self.set_pos[0]-self.pos[0])<=0.05:
                stop_func(1)
                if value == 2:
                    self.check_val5=1
                    print("here2\n")
                    self.check_val5=1
                    value = 0
                    x_val=self.x_cascade+self.w_cascade/2
                    x_val = x_val - 200                  
                    x_val = x_val *max(self.height,dist)
                    x_val = x_val/self.focal_length
                    y_val=self.y_cascade+self.h_cascade/2
                    y_val = y_val - 200
                    
                    #Setting the marker locations as the new coordinates                        
                    y_val = y_val *max(self.height,dist)
                    y_val = y_val/self.focal_length
                    self.set_pos[0] = self.pos[0] + x_val
                    self.set_pos[1] = self.pos[1] - y_val
                    if self.next_ind==4:
                        self.set_pos[1] = self.set_pos[1]+1                

            if self.check_val5==1 and self.check_val6==0 and abs(self.set_pos[1]-self.pos[1])<=0.3 and abs(self.set_pos[0]-self.pos[0])<=0.3:
                stop_func(1)
                if value == 2:
                    self.check_val6=1
                    value = 0
                    self.set_pos[2] = self.set_pos[2] - dist
                    condition = False


            #Setting Up the Next Parcel
            if self.check_val6==1 and self.check_val7==0 and abs(self.set_pos[2]-self.pos[2])<=0.3:
                stop_func(0)
                self.gripp_activate(False) 
                if value == 2:
                    value = 0               
                    self.parcel_no = self.parcel_no+1
                    if self.parcel_no < self.parcel_size - 4:
                        self.check_val1=0
                        self.check_val2=0
                        self.check_val3=0
                        self.check_val4=0
                        self.check_val5=0
                        self.check_val6=0
                        self.next_ind = self.loc_arr[self.parcel_no]
                        if self.next_ind==4 or self.next_ind==13:
                            dist = 15
                        else:
                            dist = 10
                        self.set_pos[2] = self.parcel_alt[self.next_ind] + dist
                    else:
                        self.check_val7=1
                    
            #All Possible Parcel within time limit done   
            if self.check_val7==1 and self.check_val8==0 :
                stop_func(1)
                if value==2:
                    value=0
                    self.check_val8 = 1 
                    self.loop_break.publish("stop")
                    self.drone_cmd.rcThrottle = 1000   
                    

            self.drone_cmd.rcYaw = 1500 
            if self.drone_cmd.rcRoll < 0:
                self.drone_cmd.rcRoll = 0
            if self.drone_cmd.rcRoll > 3000:
                self.drone_cmd.rcRoll = 3000
            if self.drone_cmd.rcPitch < 0:
                self.drone_cmd.rcPitch = 0
            if self.drone_cmd.rcPitch > 3000:
                self.drone_cmd.rcPitch = 3000          
                

            
                          
                           
            #publishing the velocity
            self.velocity_publisher.publish(self.drone_cmd) 
            self.loop_rate.sleep()


            # and breaking the loop , as it's posted now
            if(self.check_val8 == 1):  
                break



if __name__ == "__main__":
    try:
        publish_cmd = Position()
        time.sleep(20)
        publish_cmd.command()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")
        

    

    
