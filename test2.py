import rospy
import math
from clover import srv
from std_srvs.srv import Trigger
from sensor_msgs.msg import Range

rospy.init_node('two')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
land = rospy.ServiceProxy('land', Trigger)

def navigate_wait(x=0, y=0, z=0, yaw=float('nan'), speed=0.5, frame_id='', auto_arm=False, tolerance=0.2):
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.2)
    rospy.sleep(1)
def measure():
    global dist
    telem = get_telemetry(frame_id='aruco_map')
    data = rospy.wait_for_message('rangefinder/range', Range)
    dist = telem.z - data.range
    print (dist)

one, two = 0, 0
h_x, h_y = 0, 0
l_x, l_y = 0, 0

navigate_wait(z=2, speed=0.5, frame_id='body', auto_arm=True)

navigate_wait(x=0.2, y=2, z=2, speed=0.5, frame_id='aruco_map')
print("point one dist")
measure()
one = dist
navigate_wait(x=1.5, y=1, z=2, speed=0.5, frame_id='aruco_map')
print("point two dist")
measure()
two = dist
"sit on high(h) or low(l):"
if one > two:
    h_x = 0.2 
    h_y = 2
    l_x = 1.5
    l_y = 1
else:
    h_x = 1.5
    h_y = 1
    l_x = 0.2
    l_y = 2




    
a = raw_input("sit on high(h) or low(l):")
if a == 'h':
    print("high point selected")
    navigate_wait(x=h_x, y=h_y, z=2, speed=0.5, frame_id='aruco_map')
else:
    print("low point selected")
    navigate_wait(x=l_x, y=l_y, z=2, speed=0.5, frame_id='aruco_map')
land()
print("mission complete")
