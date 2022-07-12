import numpy as np
import kdtree
import result_set
import sys
class DBSCAN(object):
    def __init__(self,radius=0.5,leaf_size=32,Min_Pts=10):
        self.radius=radius
        self.leaf_size=32
        self.cls=[]
        self.access=[]
        self.Min_Pts=Min_Pts
    def fit(self,data):
        # data: N*3
        N = data.shape[0]
        # Modify the maximum number of recursion, otherwise it is easy to exceed the maximum number of recursion (3000)
        sys.setrecursionlimit(N)
        cls = -1*np.ones(N)
        access = np.zeros(N)
        
        search_index=list(range(N))
        
        Priority_search=[]
       
        label=-1
        #kdtree for proximity search
        KD_root = kdtree.kdtree_construction(data, leaf_size=self.leaf_size)
        while search_index:
            if not Priority_search:
                ind=search_index.pop()
                #Modify access status
                if access[ind]==1:
                    continue
                else:
                    access[ind]=1
               
                query = data[ind, :]
                radius_result = result_set.RadiusNNResultSet(self.radius)
                kdtree.kdtree_radius_search(KD_root, data, radius_result, query)
                nn_indices = [radius_result.dist_index_list[jj].index for jj in range(radius_result.size())]
                
                if len(nn_indices)-1 < self.Min_Pts:
                    cls[ind] = -1
                    continue
                else:
                    label+=1
                    cls[ind] = label
                    Priority_search.extend(nn_indices)
            else:
                ind=Priority_search.pop()
                
                if access[ind] == 1:
                    continue
                else:
                    access[ind] = 1
                    cls[ind]=label
               
                query = data[ind, :]
                radius_result = result_set.RadiusNNResultSet(self.radius)
                kdtree.kdtree_radius_search(KD_root, data, radius_result, query)
                nn_indices = [radius_result.dist_index_list[jj].index for jj in range(radius_result.size())]
                
                if len(nn_indices) - 1 < self.Min_Pts:
                    continue
                else:
                    Priority_search.extend(nn_indices)
                    Priority_search=list(np.unique(Priority_search))
        self.cls = cls
        self.access = access



    def fit_recursive(self,data): #Recursively classify points
        #data: N*3
        N=data.shape[0]
        #Modify the maximum number of recursion, otherwise it is easy to exceed the maximum number of recursion (3000)
        sys.setrecursionlimit(N)
        cls=np.zeros(N)
        access=np.zeros(N)
        radius_result=result_set.RadiusNNResultSet(self.radius)
        KD_root=kdtree.kdtree_construction(data,leaf_size=self.leaf_size)
        label=0
        for ii in range(N):
            if access[ii]==1:
                continue
            query = data[ii, :]
            radius_result = result_set.RadiusNNResultSet(self.radius)
            kdtree.kdtree_radius_search(KD_root, data, radius_result, query)
            indices = [radius_result.dist_index_list[jj].index for jj in range(radius_result.size())]
            if len(indices) < self.Min_Pts:
                cls[ii] = -1
                access[ii] = 1
                continue
            else:
                access[ii] = 1
                cls[ii] = label
                for ind in indices:
                    self.recursive_scan(KD_root,data,cls,access,label,ind)
                label+=1
        # cls[cls==-1]=label

        self.cls=cls
        self.access=access
    def recursive_scan(self,root,data,cls,access,label,index):
        #cls: record classification
        #access：record access
        #label
        if access[index]==1:
            return
        else:
            access[index]=1
            cls[index]=label
        query=data[index,:]
        radius_result = result_set.RadiusNNResultSet(self.radius)
        kdtree.kdtree_radius_search(root,data,radius_result,query)
        indices=[radius_result.dist_index_list[jj].index for jj in range(radius_result.size())]
        if len(indices)<self.Min_Pts:
            return
        for ind in indices:
            self.recursive_scan(root,data, cls, access, label, ind)
    def output_cls(self):
        return self.cls.astype(np.int)
