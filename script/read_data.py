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

def find_nearest(array, value):
    array = np.asarray(array)
    idx = (np.abs(array - value)).argmin()
    return array[idx]

def get_odom_at_time(time):
    idx = np.where(odoms_array[:,0] == find_nearest(odoms_array[:,0], time))
    return odoms_array[idx,1].reshape(-1),odoms_array[idx,2].reshape(-1)
    


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
    nbrs = NearestNeighbors(n_neighbors=nn, algorithm='ball_tree').fit(obs)
    Sigmas = []
    for ii in range(m):
        idxs = nbrs.kneighbors(Mus[ii].reshape((1,2)),n_neighbors = nn, return_distance=False)
        c = np.cov(obs[idxs].reshape((-1,2)).T) + reg*np.eye(2)
        Sigmas.append(c)
    return Mus, Sigmas

class dendt():
    def __init__(self,new_scan,bounds, Mus, Sigmas, maxiter=200,popsize=6,tol=0.0001):
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


def likelihood(gmm_map, scan):
    p = np.zeros(len(scan))
    Mus, Sigmas = gmm_map[0], gmm_map[1]
    for ii in range(len(Mus)):
        p += multivariate_normal.pdf(scan, mean = Mus[ii], cov = Sigmas[ii])
    return p

def update_map(gmm_map, new_scan, res = 0.5):
    Mus, Sigmas = gmm_map[0], gmm_map[1]
    P = likelihood(gmm_map, new_scan)
    new_scan = new_scan[P < 10.0]
    if new_scan.shape[0] < 2:
        return Mus, Sigmas
    new_mus, new_sigmas = obs2GMM(new_scan,res=res)
    if new_sigmas is None:
        return Mus, Sigmas
    Sigmas.extend(new_sigmas)
    Mus = np.vstack((Mus, new_mus))
    return Mus, Sigmas

class MCL():
    def __init__(self,Np = 100, X0 = np.zeros(3), P0 = np.eye(3)):
        self.Np = Np
        self.init(X0,P0)
        self.v_std = 0.01
        self.omega_std = 0.01
        
    def init(self, X0, P0):
        self.X = np.random.multivariate_normal(X0, P0, self.Np)
        self.W = np.ones(self.Np) / self.Np
        
    def predict(self, v, omega, dt):
        V = np.random.normal(v, self.v_std)
        O = np.random.normal(omega, self.omega_std)
        self.X[:,0] = self.X[:,0] + np.multiply(V,np.cos(self.X[:,2]))*dt
        self.X[:,1] = self.X[:,1] + np.multiply(V,np.sin(self.X[:,2]))*dt
        self.X[:,2] = self.X[:,2] + O*dt
        
    def update(self, gmm_map, scan, scan_info, max_rays):
        Mus, Sigmas = gmm_map[0], gmm_map[1]
        knn = NearestNeighbors(n_neighbors=1, algorithm='ball_tree').fit(Mus)
        for i in range(self.Np):
            scanT = scan2cart(scan,self.X[i], scan_info[1], scan_info[3], max_rays)
            _, indices = knn.kneighbors(scanT)
            p = np.zeros(len(scanT))
            for ii in range(len(Mus)):
                p += multivariate_normal.pdf(scanT[ii], mean = Mus[int(indices[ii])], cov = Sigmas[int(indices[ii])])
            self.W[i] = np.prod(p) + 1e-10
        self.W = self.W/np.sum(self.W)
        
    def MMSE(self):
        return np.mean(self.X.T*self.W, axis = 1)
    
    def MAP(self):
        return self.X[np.argmax(self.W)]
        
odoms_array, scans_array_info, scans_array = get_data()

X0 = [0.0, 0.0, 0.0]
s0 = scan2cart(scans_array[200],X0, scans_array_info[200,1],scans_array_info[200,3],scans_array[200].shape[0])
bounds = [(-0.5,0.5),(-0.5,0.5),(-0.2,0.2)]
mus_last, sigmas_last = obs2GMM(s0,0.5)
mus, sigmas = mus_last, sigmas_last
X = X0
mcl = MCL(Np = 100, X0 = np.zeros(3), P0 = 0.01*np.eye(3) )
for ii in range(201,1000):
    v , omega = get_odom_at_time(scans_array_info[ii,0])
    dt = scans_array_info[ii,0] - scans_array_info[ii-1,0]
    mcl.predict( v, omega, dt)
    s = scan2cart(scans_array[ii],mcl.MMSE(), scans_array_info[ii,1],scans_array_info[ii,3],scans_array[ii].shape[0])
    Dendt = dendt(new_scan=s.T,bounds=bounds,Mus = mus_last, Sigmas=sigmas_last, maxiter=4,popsize=4,tol=0.0001)
    sT = np.array(Dendt.transform(s,Dendt.T))
    mus_last, sigmas_last = obs2GMM(sT,0.5)
    bounds = [(-0.05 ,0.05 ),(-0.05 ,0.05 ),(-0.05,0.05 )]
    mus, sigmas = update_map(gmm_map = (mus, sigmas), new_scan = sT,res= 1.0)
    mcl.update(gmm_map = (mus, sigmas), scan = scans_array[ii],scan_info= scans_array_info[ii],max_rays= 20)
    plt.scatter(mus[:,0],mus[:,1],c='k')
    plt.scatter(sT[:,0],sT[:,1],c='r')
    plt.scatter(X[0],X[1],c='b')
    plt.show()

mus, sigmas = obs2GMM(s0,0.5)
Dendt = dendt(last_scan=s0.T,new_scan=s1.T,bounds=bounds,Mus = mus, Sigmas=sigmas, maxiter=4,popsize=4,tol=0.0001)
s1T = np.array(Dendt.transform(s1,Dendt.T))
new_mus, new_sigmas = update_map((mus, sigmas),s1T,1.0)
plt.scatter(new_mus[:,0],new_mus[:,1],c='y')

plt.scatter(s1[:,0],s1[:,1],c='r')
plt.scatter(s1T[:,0],s1T[:,1],c='b')
plt.scatter(s0[:,0],s0[:,1],c='k')
plt.scatter(mus[:,0],mus[:,1],c='y')







