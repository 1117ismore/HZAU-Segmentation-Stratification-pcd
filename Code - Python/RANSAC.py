import numpy as np
import sympy
import random

class RANSAC_plane(object):
    def __init__(self,delta=1):
        self.n_para=3 
        self.para=np.zeros([self.n_para]) 
        self.inlier_num=-1
        self.delta=delta
    def fit(self,data,iter=300):
        #data: N*3
        n=self.n_para
        for ii in range(iter):
            choose_points=random.choices(data,k=n)
            C=np.zeros([n,n])
            for jj in range(n):
                C[jj,:]=choose_points[jj]
            if np.linalg.matrix_rank(C)==0:  
                continue
            #Calculate the positional relationship between each point, which is convenient to find the normal vector
            D=np.zeros([n,n])
            D[1,:]=C[0,:]-C[n-1,:]
            for jj in range(1,n):
                D[jj,:]=C[jj,:]-C[jj-1,:]
            x = sympy.symbols('x')
            y = sympy.symbols('y')
            z = sympy.symbols('z')
            
            eqs=D@[x,y,z]
            s=sympy.solve(eqs,[x,y,z])
            #This solution must have a basic solution system, and the basic solution system is constructed according to the situation of the solution.
            if x not in s:
                x_v=1
                normal=np.array([x_v,s[y].subs(x,x_v),s[z].subs(x,x_v)]).astype(np.float)
                normal=normal/np.linalg.norm(normal)
            elif y not in s:
                y_v=1
                normal = np.array([s[x].subs(y,y_v), y_v, s[z].subs(y,y_v)]).astype(np.float)
                normal = normal / np.linalg.norm(normal)
            elif z not in s:
                z_v=1
                normal = np.array([s[x].subs(z,z_v), s[y].subs(z,z_v), z_v]).astype(np.float)
                normal = normal / np.linalg.norm(normal)
           
            D=-C[1,:]@normal.T
           
            dis=np.abs(data@normal.T+D)
            
            inlier_num=np.sum(dis<self.delta)
           
            if inlier_num>self.inlier_num:
                self.inlier_num=inlier_num
                self.normal=normal
                self.D=D
    def predict(self,data):
        D=self.D
        normal=self.normal
        
        dis = data @ normal.T + D
        # Classification of Judgment Points(inliers=1/outliers=0)
        cls = dis < self.delta
        return cls.astype(np.int)





