# -*- coding: utf-8 -*-
"""
Created on Mon May 27 16:27:45 2019

@author: Or
"""
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

def scan2cart(scan, X, angle_min, angle_increment, max_rays):
    # the robot orientation in relation to the world [m,m,rad]
    mu_x = X[0]
    mu_y = X[1]
    theta = X[2]  
    # converting the laserscan measurements from polar coordinate to cartesian and create matrix from them.
    n = len(scan); i = np.arange(len(scan))
    angle = np.zeros(n); x = np.zeros(n); y = np.zeros(n)
    rng = np.asarray(scan)
    angle = np.add(angle_min, np.multiply(i, angle_increment)) + theta
    idxs = (np.linspace(0,n-1,max_rays)).astype(int)
    #print idxs,rng.shape
    x = np.multiply(rng[idxs], np.cos(angle[idxs])) + mu_x
    y = np.multiply(rng[idxs], np.sin(angle[idxs])) + mu_y
    x[~np.isfinite(x)] = -1
    y[~np.isfinite(y)] = -1
    Y = np.stack((x,y))
    idx = (x==-1) + (y==-1)
    Y = np.delete(Y,np.where(idx),axis=1)
    return Y.T

def get_data():
    odoms_array = pd.read_csv('../data_csv/odoms_array.csv')
    scans_array_info = pd.read_csv('../data_csv/scans_array_info.csv')
    scans_array = pd.read_csv('../data_csv/scans_array.csv')
    odoms_array = np.array(odoms_array.values)
    scans_array_info = np.array(scans_array_info.values)
    scans_array = np.array(scans_array.values)
    return odoms_array, scans_array_info, scans_array


odoms_array, scans_array_info, scans_array = get_data()

X0 = [0.0, 0.0, 0.0]
s0 = scan2cart(scans_array[0],X0, scans_array_info[0,1],scans_array_info[0,3],scans_array[0].shape[0])
s1 = scan2cart(scans_array[100],X0, scans_array_info[100,1],scans_array_info[100,3],scans_array[100].shape[0])
plt.scatter(s0[:,0],s0[:,1],c='k')
plt.scatter(s1[:,0],s1[:,1],c='r')





