# -*- coding: utf-8 -*-
"""
Created on Mon May 27 16:27:45 2019

@author: Or
"""
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from sklearn.neighbors import NearestNeighbors
from scipy.optimize import differential_evolution
from scipy.stats import multivariate_normal


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

def obs2GMM(obs,res = 0.1,reg = 5e-5):
    nn = int(np.ceil(res / np.mean(np.linalg.norm(np.diff(obs,axis = 0),axis = 1))))
    if nn < 3:
        print('resolution too small!')
        return None, None
    n = len(obs) -1
    m = n//nn
    idxs = np.linspace(0,n,m).astype(int)
    Mus = obs[idxs].reshape((-1,2))
    nbrs = NearestNeighbors(n_neighbors=nn, algorithm='ball_tree').fit(Mus)
    Sigmas = []
    for ii in range(m):
        idxs = nbrs.kneighbors(Mus[ii].reshape((1,2)),n_neighbors = nn, return_distance=False)
        c = np.cov(Mus[idxs].reshape((-1,2)).T) + reg*np.eye(2)
        Sigmas.append(c)
    return Mus, Sigmas

class dendt():
    def __init__(self,last_scan,new_scan,bounds, Mus, Sigmas, maxiter=200,popsize=6,tol=0.0001):
        self.new_scan = new_scan
        self.Mus, self.Sigmas = Mus, Sigmas
        self.result = differential_evolution(self.func, bounds,maxiter=maxiter,popsize=popsize,tol=tol)
        self.T = self.result.x
    
    def transform(self,X,T):
        T = np.array(T).reshape(3)
        c, s = np.cos(T[2]), np.sin(T[2])
        j = np.matrix([[c, s], [-s, c]])
        m = np.dot(j, X.T).T + T[0:2].T
        return m
    
    def likelihood(self,X, T):
        X = self.transform(X,T)
        p = np.zeros(len(X))
        for ii in range(len(self.Mus)):
            p += multivariate_normal.pdf(X, mean = self.Mus[ii], cov = self.Sigmas[ii])
        return p
    
    def func(self,T):
        X = self.new_scan.T
        return -np.sum(self.likelihood(X, T))

def likelihood(X, Mus, Sigmas):
    p = np.zeros(len(X))
    for ii in range(len(Mus)):
        p += multivariate_normal.pdf(X, mean = Mus[ii], cov = Sigmas[ii])
    return p 

def update_map(new_scan, gmm_map, scan_TH = 1.0):
    mus = gmm_map[0]
    sigmas = gmm_map[1]
    P = likelihood(new_scan, mus, sigmas)
    if len(new_scan) < 3:
        print('scan threshold (scan_TH) is too small!')
    new_scan = new_scan[P < scan_TH] 
    mus_new, sigmas_new = obs2GMM(new_scan,0.5)
    if mus_new is not None:
        sigmas.append(sigmas_new)
        mus = np.append(mus, mus_new, axis=0)
    return mus, sigmas

def main():
    
    odoms_array, scans_array_info, scans_array = get_data()
    
    X0 = [0.0, 0.0, 0.0]
    Ns = 1
    
    for ii in range (0,100,Ns):
        s0 = scan2cart(scans_array[ii],X0, scans_array_info[ii,1],scans_array_info[ii,3],scans_array[ii].shape[0])
        s1 = scan2cart(scans_array[ii+Ns],X0, scans_array_info[ii+Ns,1],scans_array_info[ii+Ns,3],scans_array[ii+Ns].shape[0])
        
        bounds = [(-0.5,0.5),(-0.5,0.5),(-0.2,0.2)]
        mus, sigmas = obs2GMM(s0,0.5)
        Dendt = dendt(last_scan=s0.T,new_scan=s1.T,bounds=bounds,Mus = mus, Sigmas=sigmas, maxiter=4,popsize=4,tol=0.0001)
        X0 += Dendt.T
        s2 = scan2cart(scans_array[ii+Ns*2],X0, scans_array_info[ii+Ns*2,1],scans_array_info[ii+Ns*2,3],scans_array[ii+Ns*2].shape[0])
        mus, sigmas = obs2GMM(s1,0.5)
        
        bounds = [(-0.5+X0[0],0.5+X0[0]),(-0.5+X0[1],0.5+X0[1]),(-0.2+X0[2],0.2+X0[2])]
        Dendt = dendt(last_scan=s1.T,new_scan=s2.T,bounds=bounds,Mus = mus, Sigmas=sigmas, maxiter=4,popsize=4,tol=0.0001)
        
        s2T = np.array(Dendt.transform(s2,Dendt.T))
        
        mus, sigmas = update_map(s2T,gmm_map=(mus, sigmas))
        
    plt.scatter(s2[:,0],s2[:,1],c='r')
    plt.scatter(s2T[:,0],s2T[:,1],c='b')
    plt.scatter(s1[:,0],s1[:,1],c='k')
    plt.scatter(mus[:,0],mus[:,1],c='y')
    plt.show()

if __name__== "__main__":
    main()








