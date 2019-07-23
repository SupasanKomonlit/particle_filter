#!/usr/bin/env python2
# FILE			: particle_distance.py
# AUTHOR		: K.Supasan
# CREATE ON		: 2019, July 23 (UTC+0)
# MAINTAINER	: K.Supasan

# README

# REFERENCE
#   ref01   : https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/12-Particle-Filters.ipynb
#   ref02   : https://matplotlib.org/users/pyplot_tutorial.html
#   ref03   : https://docs.scipy.org/doc/numpy/reference/generated/numpy.array.html
#   ref04   : https://stackoverflow.com/questions/20059766/handle-exception-in-init
#   ref05   : https://docs.scipy.org/doc/scipy/reference/generated/scipy.stats.norm.html
#   ref06   : https://jakevdp.github.io/PythonDataScienceHandbook/02.02-the-basics-of-numpy-arrays.html
#   ref07   : https://docs.scipy.org/doc/numpy-1.11.0/reference/generated/numpy.full.html
#   ref08   : https://docs.scipy.org/doc/numpy-1.15.1/reference/generated/numpy.copysign.html

from __future__ import print_function, division

import numpy as np              # Develop on version 2.2.4
import scipy.stats              # Develop on version 1.2.2
import filterpy.monte_carlo     # Develop on version 1.4.5

def random_error( multiply ):

    error = np.random.ranf( 3 )
    count = 0
    for run in np.random.randint( -2 , 2 , 3 ):
        if run < 0 :
            error[count] = np.copysign( error[count] , -1 )
        count += 1

    return error * multiply
