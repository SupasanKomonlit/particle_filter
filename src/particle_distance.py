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

def neff( weights ):
    return 1. /np.sum( weights )

class ParticleDistance:

    # init function use to setup a bunch of particle
    def __init__( self , particles ):

        self.particles = np.array(particles)
        self.weights = np.full( len( self.particles ) , 1.0/len( self.particles) , dtype=np.float )

        self.mean = ( 0 , 0 , 0 )
        self.var = ( 0 , 0 , 0 )

    # In your input please input about ( position(t-1) - position(t) ) or inverse movement of your
    # About concept of update
    #   If you input your update distance will more if you move more
    # Two parameter is
    #   movement in this class this movement show you ditance which should to be
    #   multiply will use to gain error to spread particle
    def predict( self, movement, multiply ):

        for particle in self.particles:
            error = random_error( multiply )
            particle += movement + error

    # Update function is function to updated measurement to help use decrease spread of particle
    def update( self , measurement , variance ):

        for run in range( len( self.particles ) ): 
            self.weights[run] = scipy.stats.norm( np.linalg.norm( self.particles[run] ),variance).pdf(
                measurement )

        self.weights += 1.e-3000                    # avoids round of zero
        self.weights /= np.sum( self.weights )      # normalize

        if neff( self.weights ) < ( len( self.particles ) / 2 ):
            indexes = filterpy.monte_carlo.systematic_resample( self.weights )
            self.particles[:] = self.particles[indexes]
            self.weights.fill( 1 / len( self.weights ) )
            assert np.allclose( self.weights , 1.0 / len( self.particles ) )
        else:
            print( "Don't resample")

    # Estimate function use algorithm average to find answer and variance
    def estimate( self ):
        mean = np.average( self.particles , weights=self.weights , axis=0 )
        var = np.average( ( mean - self.particles )**2 , weights=self.weights , axis=0 )
        self.mean = mean
        self.var = var
        return mean , var

    def __repr__( self ):
        return "========================RESULT=============================\nDISTANCE : {:7.3f} {:7.3f} {:7.3f}\nVARIANCE : {:7.3f} {:7.3f} {:7.3f}".format(
            self.mean[0] , self.mean[1] , self.mean[2] , self.var[0] , self.var[1] , self.var[2] )
