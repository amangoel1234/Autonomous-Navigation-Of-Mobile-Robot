#!/usr/bin/env python
import rospy
import struct
import time
from multiprocessing import Queue
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist



k=0
Matrix = [[]]
path=[]
parent=[]
width=252
height=245
q =Queue()
start_x=110 #1.8cm
start_y=74 #2cm
clockwise=0



def move():
    print("Move")
    velocity_publisher = rospy.Publisher('/RosAria/cmd_vel', Twist, queue_size=50)
    #tpub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=50)
    vel_msg = Twist()
    
    #Receiveing the user's input
    speed = 0.1
    distance = 0.05
    isForward = 1
    
    #Checking if the movement is forward or backwards
    if(isForward):
        vel_msg.linear.x = abs(speed)
    else:
        vel_msg.linear.x = -abs(speed)
    #Since we are moving just in x-axis
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0
    

    #Setting the current time for distance calculus
    t0 = rospy.Time.now().to_sec()
    current_distance = 0
    #Loop to move the turtle in an specified distance
    while(current_distance < distance):
        #Publish the velocity
        velocity_publisher.publish(vel_msg)
        #Takes actual time to velocity calculus
        t1=rospy.Time.now().to_sec()
        #Calculates distancePoseStamped
        current_distance= speed*(t1-t0)
    vel_msg.linear.x = 0
    velocity_publisher.publish(vel_msg)
    #tpub.publish(vel_msg)


def rotate():
    print("Rotate")
    global clockwise
    PI = 3.1415926535897
    velocity_publisher = rospy.Publisher('/RosAria/cmd_vel', Twist, queue_size=50)
    #tpub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=50)
    vel_msg = Twist()

    # Receiveing the user's input
    speed = 10
    #Converting from angles to radians
    angular_speed = speed*2*PI/360
    relative_angle = 90*2*PI/360
    #We wont use linear components
    vel_msg.linear.x=0
    vel_msg.linear.y=0
    vel_msg.linear.z=0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0

    # Checking if our movement is CW or CCW
    if clockwise:
        vel_msg.angular.z = -abs(angular_speed)
    else:
        vel_msg.angular.z = abs(angular_speed)
    # Setting the current time for distance calculus
    t0 = rospy.Time.now().to_sec()
    current_angle = 0
    while(current_angle < relative_angle):
        velocity_publisher.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()
        current_angle = angular_speed*(t1-t0)

    #Forcing our robot to stop
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)
    #tpub.publish(vel_msg)
    time.sleep(2)
    move()
    time.sleep(2)


def inflate():
	global Matrix
	global width
	global height
	for k in range(6):
		tmp_mat = [row[:] for row in Matrix]
		for i in range(1,height-20):
			for j in range(1,width-20):
				if(tmp_mat[i][j]==100):
					Matrix[i+1][j]=100
					Matrix[i][j+1]=100
					Matrix[i-1][j]=100
					Matrix[i][j-1]=100
					Matrix[i+1][j+1]=100
					Matrix[i+1][j-1]=100
					Matrix[i-1][j-1]=100
					Matrix[i-1][j-1]=100


def final_path(dest_x,dest_y):
	global path
	global start_x
	global start_y
	global width
	global height

	if((dest_x==start_x and dest_y==start_y) or parent[dest_x][dest_y]==-1):
		return 
	path.append(parent[dest_x][dest_y])
	temp=parent[dest_x][dest_y]
	final_path(temp%width,temp/width)

def bfs(start_x, start_y):
	global Matrix
	global parent
	global width
	global height
	for i in range(100000):
		if((start_x+1<width) and (start_y<height) and Matrix[start_x+1][start_y]==0 and (parent[start_x+1][start_y]==-1)):
			parent[start_x+1][start_y]=start_x+start_y*width
			q.put(start_x+1+start_y*width)
		if((start_x-1<width) and (start_y<height) and Matrix[start_x-1][start_y]==0 and (parent[start_x-1][start_y]==-1)):
        	        parent[start_x-1][start_y]=start_x+start_y*width
        	        q.put(start_x-1+start_y*width)
		if((start_x<width) and (start_y+1<height) and Matrix[start_x][start_y+1]==0 and (parent[start_x][start_y+1]==-1)):
       		        parent[start_x][(start_y+1)]=start_x+(start_y)*width
       	        	q.put(start_x+(start_y+1)*width)
		if((start_x<width) and (start_y-1<height) and Matrix[start_x][start_y-1]==0 and (parent[start_x][start_y-1]==-1)):
	                parent[start_x][start_y-1]=start_x+start_y*width
	                q.put(start_x+(start_y-1)*width)
		temp=q.get()
		if(q.empty()):
			break
		start_x=temp%width
		start_y=temp/width
	return

def Callback(data):
	global k
	global Matrix
	global width
	global height
	global start_x
	global start_y
	global path
	global clockwise

	if(k==0):
		k=k+1
		Matrix = [[0 for x in range(data.info.height)] for y in range(data.info.width)]
		width = data.info.width
		height = data.info.height
		print(width)
		for i in range (width*height):
			Matrix[i%width][i/width] = data.data[i]
	for i in range(220):
		print(Matrix[i][110],i)

	#inflate()
	temp_x = 0
	temp_y = 0
	q.put(start_x+start_y*width)
	for i in range(height):
		new=[]
		for j in range(width):
			new.append(-1)
		parent.append(new)
	parent[start_x][start_y]=-50
	print(q.get())
	dest_x=170 #3.5cm
	dest_y=120 #2cm
	bfs(start_x, start_y)
	path.append(dest_y*width+dest_x)
	final_path(dest_x, dest_y)
	path.append(start_x+(start_y-1)*width)
	print len(path)
	for i in range(1,len(path)):
		print(path[-i]%width," ",path[-i]/width)
	print("kkk")
        for i in range(1,len(path)-1):
		temp_x=path[-1*(i+2)]%width
		temp_y=path[-1*(i+2)]/width
		if(path[-1*i]%width==path[-1*(i+2)]%width or path[-1*i]/width==path[-1*(i+2)]/width):
			move()
			time.sleep(2)
		elif (((temp_y-path[-1*(i)]/width)/(temp_x - path[-1*(i)]%width)==-1 and path[-1*(i+2)]/width==path[-1*(i+1)]/width) or ((temp_y-path[-1*(i)]/width)/(temp_x - path[-1*(i)]%width)==1 and path[-1*(i+2)]%width==path[-1*(i+1)]%width) ):
			clockwise=1
			rotate()
			time.sleep(2)
		elif ( ((temp_y-path[-1*(i)]/width)/(temp_x - path[-1*(i)]%width)==-1 and path[-1*(i+2)]%width==path[-1*(i+1)]%width) or ((temp_y-path[-1*(i)]/width)/(temp_x - path[-1*(i)]%width)==1 and path[-1*(i+2)]/width==path[-1*(i+1)]/width)):
			clockwise=0
                        rotate()
			time.sleep(2)

rospy.init_node("listener", anonymous=True)
sub = rospy.Subscriber("/map", OccupancyGrid, Callback)
print("000")
#print(sub)
rospy.spin()
