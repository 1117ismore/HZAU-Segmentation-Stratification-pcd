import numpy as np
import open3d as o3d
from matplotlib import pyplot as plt
import copy
import kdtree
import KMeans
from result_set import KNNResultSet

def draw_labels_on_model(pcl, labels):
    cmap = plt.get_cmap("tab20")
    pcl_temp = copy.deepcopy(pcl)
    max_label = labels.max()
    colors = cmap(labels / (max_label if max_label > 0 else 1))
    pcl_temp.colors = o3d.utility.Vector3dVector(colors[:, :3])
    o3d.visualization.draw_geometries([pcl_temp], window_name="visualization",
                                      width=800, height=800, left=50, top=50,
                                      mesh_show_back_face=False)


class Spectral_Cluster(object):
    
    def __init__(self, n_clusters=2,normalization=True):
        self.k_ = n_clusters
        self.adaptive=False
        self.normalization=normalization
    def fit(self, data):    #data :N*d
        self.predict(data)  

    def get_cluster_labels(self, clusters, X):
        y_pred = np.zeros(np.shape(X)[0])
        for cluster_i, cluster in enumerate(clusters):
            for sample_i in cluster:
                y_pred[sample_i] = cluster_i
        return y_pred

    def predict(self, data):
        result = []
        k = self.k_
        N = data.shape[0]
        normalization = self.normalization
       
        KDT = kdtree.kdtree_construction(data, leaf_size=16)
       
        W = np.eye(N)
        
        nn_k = 8
        
        for ii in range(N):
            nn_result = KNNResultSet(capacity=nn_k+1)  
            kdtree.kdtree_knn_search(KDT, data, nn_result,data[ii,:])
            index = [nn_result.dist_index_list[jj].index for jj in range(1,nn_k+1)]
            dis = [nn_result.dist_index_list[jj].distance for jj in range(1,nn_k+1)]

            W[ii, index] =  np.exp(-np.power(dis,2))
            W[index, ii] = np.exp(-np.power(dis,2))
        W=(W+W.T)/2
        # Computational Degree Matrix
        D = np.diag(np.sum(W, axis=1))

        # Calculate the Laplace matrix
        if normalization == False:
            L = D - W
        else:
            temp_D = np.linalg.inv(np.sqrt(D))
            L = np.eye(N) - np.matmul(np.matmul(temp_D,W),temp_D)

        # Eigenvalue Decomposition
        vals, vecs = np.linalg.eigh(L)
        vals=np.real(vals)
        vecs=np.real(vecs)
        # sort_ind = np.argsort(vals)
        # vals = vals[sort_ind]
        # vecs = vecs[:,sort_ind]

        # get the feature representation of the data point
        F = vecs[:, :k]
        # F=F/np.linalg.norm(F,axis=0)
        return self.get_cluster_labels(clusters, X)

if __name__ == "__main__":
  
    pcd = o3d.io.read_point_cloud('*.pcd')
    points = np.asarray(pcd.points)
    o3d.visualization.draw_geometries([pcd], window_name="visualization",
                                      width=800, height=800, left=50, top=50,
                                      mesh_show_back_face=False)
    # K-means  cluster
    clf = Spectral_Cluster()
    labels = clf.predict(points)
    
    draw_labels_on_model(pcd, labels)