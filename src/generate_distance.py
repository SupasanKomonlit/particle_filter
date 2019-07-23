#!/usr/bin/env python2
# FILE			: generate_distance.py
# AUTHOR		: K.Supasan
# CREATE ON		: 2019, July 23 (UTC+0)
# MAINTAINER	: K.Supasan

# REFERENCE
#   ref01   : https://quaternions.online/
#   ref02   : http://docs.ros.org/kinetic/api/tf/html/python/transformations.html

# README
#   Quaternion represent by tuple in term of xi + yj +zk + w to ( x , y , z , w )

import numpy as np
from tf import transformations as tf_handle

QUATERNIONS = (
        (0.000 , 0.000 , 0.000 , 1.000 ) , (0.000 , 0.000 , 0.383 , 0.924 )
        ,( 0.000 , 0.000 , 0.707 , 0.707 ) , ( 0.000 , 0.000 , 0.924 , 0.383 )
        ,( 0.000 , 0.000 , -1.000 , 0.000 ) , ( 0.000 , 0.000 , -0.924 , 0.383 )
        ,( 0.000 , 0.000 , -0.707 , 0.707 ) , ( 0.000 , 0.000 , -0.383 , 0.924 )
        ,( 0.000 , 0.383 , 0.000 , 0.924 ) , ( 0.000 , 0.707 , 0.000 , 0.707 )
        ,( 0.000 , 0.924 , 0.000 , 0.383 ) , ( 0.000 , 1.000 , 0.000 , 0.000 )
        ,( 0.000 , 0.000 , 1.000 , 0.000 ) , ( 0.000 , -0.924 , 0.000 , 0.383 )
        ,( 0.000 , -0.707 , 0.000 , 0.707 ) , ( 0.000 , -0.383 , 0.000 , 0.924 )
        ,( 0.146 , 0.354 , 0.354 , 0.854 ) , ( -0.146 , 0.354 , 0.354 , 0.854 )
        ,( 0.271 , 0.653 , 0.271 , 0.653 ) , ( -0.271 , 0.653 , -0.271 , 0.653 )
        ,( 0.354 , 0.854 , 0.146 , 0.354 ) , ( -0.354 , 0.854 , -0.146 , 0.354 )
        ,( 0.383 , 0.924 , 0.000 , 0.000 ) , ( -0.383 , 0.924 , -0.000 , 0.000 )
        ,( -0.354 , -0.854 , 0.146 , 0.354 ) , ( 0.354 , -0.854 , -0.146 , 0.354 )
        ,( -0.271 , -0.653 , 0.271 , 0.653 ) , ( 0.271 , -0.653 , -0.271 , -0.653 )
)

def generate_distance( start , end , step ):

    print( "Generating point by distance {:6.3f}:{:6.3f}:{:6.3f}".format(
        start , step , end ) )

    points = np.array( [] , dtype=np.float )

    current  = start 

    answer = np.array( [] , dtype=np.float )

    while current < end :

        for quaternion in QUATERNIONS :
            answer = np.append( answer , tf_handle.quaternion_multiply(
                tf_handle.quaternion_multiply( tf_handle.quaternion_inverse( quaternion ) 
                    , ( current , 0 , 0 , 0 ) )
                , quaternion
            )[0:3] )

        current += step

    return answer.reshape( len( answer ) / 3 , 3 )
