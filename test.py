import rospy
import math
import cv2
from clover import srv
from std_srvs.srv import Trigger
from pyzbar import pyzbar
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, Range

hsv_min = (0, 200, 100)
hsv_max = (180, 255, 255)
n = []
r = 0
b = 0
y = 0
one = 0.0
two = 0.0
three = 0.0

rospy.init_node("test2")
bridge = CvBridge()

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)
pub = rospy.Publisher("/img_debug", Image, queue_size=1)


def navigate_wait(x=0, y=0, z=0, yaw=float('nan'), speed=0.5, frame_id='', auto_arm=False, tolerance=0.2):
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.2)
    rospy.sleep(3)

def measure():
    global dist
    telem = get_telemetry(frame_id='aruco_map')
    data = rospy.wait_for_message('rangefinder/range', Range)
    dist = telem.z - data.range
    print (dist)

def image_callback(data):
    global hue
    global n
    cv_image = bridge.imgmsg_to_cv2(data, 'bgr8') 
    barcodes = pyzbar.decode(cv_image)
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV )
    y = hsv.shape[0] / 2
    x = hsv.shape[1] / 2
    hue = hsv[y][x][0]
    v = []
    if len(barcodes) > 0:
        b_data = barcodes[0].data.encode("utf-8")
        a = b_data.split(" ")
        for i in a:
            c = float(i)
            v.append(c)
        n = v

sub = rospy.Subscriber('main_camera/image_raw', Image, image_callback, queue_size=1)

navigate_wait(z=1, frame_id='body', auto_arm=True)

navigate_wait(x=n[0], y=n[1], z=2, frame_id='aruco_map')

print("point one dist")
measure()
one = dist
if hue > 80 and  hue < 150:
    b += 1
    b_x, b_y = n[0], n[1]
    print ("blue")
if hue > 20 and hue < 40:
    y += 1
    y_x, y_y = n[0], n[1]
    print("yellow")
if hue > 0 and hue < 20:
    r += 1
    r_x, r_y = n[0], n[1]
    print("red")

navigate_wait(x=n[2], y=n[3], z=2, frame_id='aruco_map')

print("point two dist")
measure()
two = dist
if hue > 80 and  hue < 150:
    b += 1
    b_x, b_y = n[2], n[3]
    print ("blue")
if hue > 20 and hue < 40:
    y += 1
    y_x, y_y = n[2], n[3]
    print("yellow")
if hue > 0 and hue < 20:
    r += 1
    r_x, r_y = n[2], n[3]
    print("red")

navigate_wait(x=n[4], y=n[5], z=2, frame_id='aruco_map')

print("point three dist")
measure()
three = dist
if hue > 80 and  hue < 150:
    b += 1
    b_x, b_y = n[4], n[5]
    print ("blue")
if hue > 20 and hue < 40:
    y += 1
    y_x, y_y = n[4], n[5]
    print("yellow")
if hue > 0 and hue < 20:
    r += 1
    r_x, r_y = n[4], n[5]
    print("red")


file = open("out.txt", 'w')
file = open("out.txt", 'a')
file.write("There is {} red markers,{} blue and {} yellow".format(r,b,y))
file.write("Point one{} measure,{} two and {} three point".format(one,two,three))
file.close()

raw = raw_input("choose color(r,b,y):")

if raw == 'r':
    print("Red point selected")
    navigate_wait(x=r_x, y=r_y, z=2, frame_id='aruco_map')
    rospy.sleep(5)
    navigate(yaw=math.radians(-360), frame_id='body')
    rospy.sleep(1)
if raw =='b':
    print("Blue point selected")
    navigate_wait(x=b_x, y=b_y, z=2, frame_id='aruco_map')
    rospy.sleep(5)
    navigate(yaw=math.radians(-360), frame_id='body')
    rospy.sleep(1)
else:
    print("Yellow point selected")
    navigate_wait(x=y_x, y=y_y, z=2, frame_id='aruco_map')
    rospy.sleep(5)
    navigate(yaw=math.radians(-360), frame_id='body')
    rospy.sleep(1)
land()
print("mission complete")
