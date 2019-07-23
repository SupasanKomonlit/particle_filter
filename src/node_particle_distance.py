#!/usr/bin/env python2
# FILE			: node_particle_distance.py
# AUTHOR		: K.Supasan
# CREATE ON		: 2019, July 22 (UTC+0)
# MAINTAINER	: K.Supasan

# REFERENCE
#   ref01   : http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29
#   ref02   : https://docs.scipy.org/doc/numpy/reference/generated/numpy.copy.html
#   ref03   : http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29

# README

import numpy as np
import rospy
import math
import tf

from particle_distance import ParticleDistance
from generate_distance import generate_distance

from nav_msgs.msg import Odometry                       # Receive current state
from std_msgs.msg import Float64                        # Receive message of beacon
from sensor_msgs.msg import PointCloud, ChannelFloat32  # Use to publish particles
from geometry_msgs.msg import Point32, Twist
# Twist have 2 variable linear and angular so
#   ===> linear is distance for frame in topic_state position
#   ===> angular we will use to represent about variance

def array_to_point32( points ):
    answer = []
    for point in points:
        answer.append( Point32( point[0] , point[1] , point[2] ) )
    return tuple(answer)

class NodeParticlesDistance:

    def __init__( self ):

        # Below three parameter will use to estimate first point
        start_distance = rospy.get_param( '~start_distance' , 2 )
        end_distance = rospy.get_param( '~end_distance' , 20 )
        step_distance = rospy.get_param( '~step_distance' , 2 )
        # Below parameter is about particle filter
        self.variance = rospy.get_param( '~standard_deviation' , 1 )
        self.variance *= self.variance
        self.multiply = rospy.get_param( '~multiply' , 0.5 )

        self.rate = rospy.Rate( rospy.get_param( '~frequency' , 5 ) )

        # Below parameter will use to set topic in ros topic
        topic_state = rospy.get_param( "~topic_state" , "/syrena/odom" )
        topic_data = rospy.get_param( "~topic_data" , "/beacon/range" )
        # Always output is distance from robot
        topic_output = rospy.get_param( "~topic_output" , "/beacon/position" )
        topic_particle = rospy.get_param( "~topic_particle" , "/beacon/particle" )

        # Below paraqmeter manage about tf of this
        self.parent_frame = rospy.get_param( "~parent_frame" , "odom" )
        self.child_frame = rospy.get_param( "~child_frame" , "estimate_beacon" )
        self.broadcaster = tf.TransformBroadcaster()
        # ====> Below variable is assign type data to broadcaster
        # ========> if true show your data from topic_state is position of parent frame
        # ========> if false show your data from topic_state have to sum before publish
        self.direct_to_parent = rospy.get_param( "~connect_with_paraent" , False )

        # Below will set about variable in ROS system
        rospy.Subscriber( topic_state , Odometry , self.listen_state )
        rospy.Subscriber( topic_data , Float64 , self.listen_data )

        self.pub_output = rospy.Publisher( topic_output , Twist , queue_size = 10 )
        self.pub_particle = rospy.Publisher( topic_particle , PointCloud , queue_size = 10 )

        self.state = np.array( [0. , 0. , 0. ] , dtype=np.float )
        self.time_receive_state = rospy.get_rostime()
        self.data = 0.
        self.time_receive_data = rospy.get_rostime()

        self.filter = ParticleDistance( 
            generate_distance( start_distance , end_distance , step_distance ) )

    def loop( self ):
        
        state_time = rospy.get_rostime()
        data_time = rospy.get_rostime()

        # For fist time this will make we can use particle filter
        print( "Waiting first data")
        while not rospy.is_shutdown():
            self.rate.sleep()
            if( self.time_receive_state > state_time ):
                state_time = self.time_receive_state
                state = np.copy( self.state )
        
            if( self.time_receive_data > data_time ):
                data_time = self.time_receive_data
                data = self.data
                self.broadcaster_particles()
                break

            self.broadcaster_particles()

        print( "Normal case loop particle filter")
        while not rospy.is_shutdown():
            self.rate.sleep()
            if( self.time_receive_state > state_time ):
                state_time = self.time_receive_state
                movement = state - self.state 
                state = np.copy( self.state )
                self.filter.predict( movement , self.multiply)

            if( self.time_receive_data > data_time ):
                data_time = self.time_receive_data 
                self.filter.update( self.data , self.variance )

                estimate = self.filter.estimate()
                data_output = Twist()
                data_output.linear.x = estimate[0][0]
                data_output.linear.y = estimate[0][1]
                data_output.linear.z = estimate[0][2]
                data_output.angular.x = estimate[1][0]
                data_output.angular.y = estimate[1][1]
                data_output.angular.z = estimate[1][2]
                self.pub_output.publish( data_output )
                self.broadcaster.sendTransform( state + estimate[0]   
                    , ( 0, 0, 0, 1 )
                    , rospy.Time.now()
                    , self.child_frame
                    , self.parent_frame )
                print( repr( self.filter ) )

            self.broadcaster_particles()

    def broadcaster_particles( self ):
        # Part for export point clound
        message = PointCloud()
        message.header.stamp = rospy.get_rostime()
        message.header.frame_id = self.parent_frame
        message.channels = []
        message.channels.append( ChannelFloat32() )
        message.channels[0].name  = "intensities"
        if self.direct_to_parent :
            message.points = array_to_point32( self.filter.particles )
        else:
            message.points = array_to_point32( self.filter.particles + self.state )

        message.channels[0].values = tuple( self.filter.weights )
        self.pub_particle.publish( message )
            
    def listen_state( self , message ):
        self.state = np.array( [ message.pose.pose.position.x 
            , message.pose.pose.position.y
            , message.pose.pose.position.z ] , dtype=np.float )
        self.time_receive_state = rospy.get_rostime()

    def listen_data( self , message ):
        self.data = message.data
        self.time_receive_data = rospy.get_rostime() 

if __name__=="__main__":

    rospy.init_node( "find_beacon")

    find_beacon = NodeParticlesDistance()  
    find_beacon.loop()
