from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16
import math
import rospy
svet=0
def callback(msg):
	global svet
	svet=msg.data
	print svet

from sensor_msgs.msg import LaserScan

rospy.init_node("finish")

pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
pub1=rospy.Publisher("Turtle", Int16)
kp = 0.05
pos = 0
ls = 0
cv = Twist()
x = 0
y = 0
theta_nol = 0
theta_curs = 0

def publ(x, z):
    global cv
    cv.linear.x = x
    cv.angular.z = z
    pub.publish(cv)
    rospy.sleep(0.05)

def func_odom(msg):
    global pos
    pos = msg

def func_scan(msg):
    global ls
    ls = msg

def func_ls():
    global ls
    while True:
        if ls != 0:
            if ls.ranges[0] == float("inf"):
                pass
            else:
                if ls.ranges[0] < 0.3:
                    return True
            return False

def quaternion_to_theta():
    global pos
    while True:
        if pos != 0:
            t1 = +2.0 * (pos.pose.pose.orientation.w * pos.pose.pose.orientation.z + pos.pose.pose.orientation.x * pos.pose.pose.orientation.y)
            t2 = +1.0 - 2.0 * (pos.pose.pose.orientation.y ** 2 + pos.pose.pose.orientation.z ** 2)
            return math.degrees(math.atan2(t1, t2))

def povorot(msg):
    nol = quaternion_to_theta()
    while abs(quaternion_to_theta() - nol) <= msg:
        publ(0, 0.5)
    publ(0, 0)

def nol_xy():
    global x, y, pos  
    while True:
        if pos != 0: 
            x = pos.pose.pose.position.x
            y = pos.pose.pose.position.y
            break

def dist_func():
    global x, y, pos
    r = ((pos.pose.pose.position.x - x)**2 + (pos.pose.pose.position.y - y)**2)**0.5
    return r

def pid():
    global theta_nol
    theta_curs = quaternion_to_theta()
    if theta_curs != theta_nol:
        return (theta_nol - theta_curs)*kp

def dist_race(msg):
    global cv, theta_nol 
    nol_xy()
    theta_nol = quaternion_to_theta()
    while dist_func() < msg:
        if not func_ls():
            publ(0.1, pid())
        else:
            publ(0, 0)
    publ(0, 0)


rospy.Subscriber("scan", LaserScan, func_scan)
rospy.Subscriber("odom", Odometry, func_odom)
rospy.Subscriber("turtle_foto", Int16, callback)
while True:
	if svet>300:
		povorot(90)
pub1.Publish(200)
