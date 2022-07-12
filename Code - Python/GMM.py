# GMM  algorithm

import numpy as np
from numpy import *
import pylab
import random,math

import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
from scipy.stats import multivariate_normal
plt.style.use('seaborn')

def Gaussian_fun(x,mu,sigma):  #x : N*d input
    dim=x.shape[1]
    coff= 1/(np.power(2*np.pi,dim/2.0)*np.power(np.linalg.det(sigma),0.5))
    temp_m=np.matmul(x-mu,np.linalg.inv(sigma))
    index=np.sum(temp_m*(x-mu),axis=1)
    return (coff*np.exp(-0.5*index))
class GMM(object):
    def __init__(self, n_clusters, max_iter=50):
        self.n_clusters = n_clusters
        self.max_iter = max_iter

    def fit(self, data):
       
        k = self.n_clusters
        max_iter = self.max_iter
        dim=data.shape[1]
        N=data.shape[0]
      
        mu_points = random.choices(data, k=k)
       
        mu_points = []
        mu_points.append(random.choice(data))
        for ii in range(1, k):
            dis = np.zeros(data.shape[0])
            for jj in range(ii):
                dis += np.linalg.norm(data - mu_points[jj], axis=1)
            mu_points.append(data[np.argmax(dis), :])
        #initialization Sigma，Pi
        Sigma=[]
        Pi=[]
        temp_sigma=np.matmul(data.T,data)/N
        pass
        for ii in range(k):
            Sigma.append(temp_sigma) 
            Pi.append(np.array(1/k,dtype=np.float))  
        for iter in range(max_iter):
            
            Gamma=np.zeros([N,k])
            for ii in range(k):
                Gamma[:,ii]=Pi[ii]*Gaussian_fun(data,mu_points[ii],Sigma[ii])
            Gamma=Gamma/np.expand_dims(np.sum(Gamma,axis=1),axis=1)
            # Nk Normalized
            Nk = np.sum(Gamma, axis=0)
        
            for ii in range(k):
                mu_points[ii]=np.sum(np.expand_dims(Gamma[:,ii],axis=1)*data,axis=0)/Nk[ii]
                Sigma[ii]=np.matmul((np.expand_dims(Gamma[:,ii],axis=1)*(data-mu_points[ii])).T,(data-mu_points[ii]))/Nk[ii]
                Pi[ii]=Nk[ii]/N

        self.Pi=Pi
        self.Sigma=Sigma
        self.mu=mu_points
    
    def predict(self, data):
     
        N = data.shape[0]
        k = self.n_clusters
        Pi=self.Pi
        mu_points=self.mu
        Sigma=self.Sigma
        probability=np.zeros([N,k])
        for ii in range(k):
            probability[:,ii]=Pi[ii]*Gaussian_fun(data,mu_points[ii],Sigma[ii])
        point_cls = np.argmax(probability, axis=1)
       
        result = list(point_cls)
        return result

def generate_X(true_Mu, true_Var):
   
    num1, mu1, var1 = 400, true_Mu[0], true_Var[0]
    X1 = np.random.multivariate_normal(mu1, np.diag(var1), num1)
    
    num2, mu2, var2 = 600, true_Mu[1], true_Var[1]
    X2 = np.random.multivariate_normal(mu2, np.diag(var2), num2)
    
    num3, mu3, var3 = 1000, true_Mu[2], true_Var[2]
    X3 = np.random.multivariate_normal(mu3, np.diag(var3), num3)
    
    X = np.vstack((X1, X2, X3))
   
    plt.figure(figsize=(10, 8))
    plt.axis([-10, 15, -5, 15])
    plt.scatter(X1[:, 0], X1[:, 1], s=5)
    plt.scatter(X2[:, 0], X2[:, 1], s=5)
    plt.scatter(X3[:, 0], X3[:, 1], s=5)
    plt.show()
    return X

if __name__ == '__main__':
   
    true_Mu = [[0.5, 0.5], [5.5, 2.5], [1, 7]]
    true_Var = [[1, 3], [2, 2], [6, 2]]
    X = generate_X(true_Mu, true_Var)

    gmm = GMM(n_clusters=3)
    gmm.fit(X)
    cat = gmm.predict(X)
    print(cat)
    

    

