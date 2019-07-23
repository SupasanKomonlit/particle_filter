import rospy

import tf

import numpy as np
import math
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

from syrena_utils.nav import distance


rospy.init_node("beacon_sim")

beacon_pub = rospy.Publisher("/beacon/range",Float64,queue_size=2)

broadcaster = tf.TransformBroadcaster()

first_odom = None
ever_set = False
xyz = np.array( ( 0 , 0 , 0) )
def cb(msg):

    global first_odom,xyz,ever_set

    pose = msg.pose.pose.position
	
    xyz = np.array([pose.x,pose.y , pose.z])
	
    if not ever_set :
        rx = (np.random.rand() * 40) - 20
        ry = (np.random.rand() * 40) - 20
        rz = (np.random.rand() * 2) - 5
        ever_set = True
        first_odom = xyz + np.array([rx,ry,rz])
        if( first_odom[2] > 0 ):
            first_odom *= -1
        print( first_odom )

    broadcaster.sendTransform( first_odom   
        , ( 0, 0, 0, 1 )
        , rospy.Time.now()
        , 'beacon'
        , 'odom' )
	
rospy.Subscriber("/syrena/odom",Odometry,cb)


while not rospy.is_shutdown():
	
    if ever_set :
        r = xyz - first_odom
        r = np.linalg.norm( r )	
        r += np.random.rand()*4 - 2
		
        print r
        beacon_pub.publish(r)	

    rospy.sleep(1)
