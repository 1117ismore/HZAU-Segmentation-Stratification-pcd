import numpy as np
import open3d as o3d
import copy
from matplotlib import pyplot as plt

# label
def draw_labels_on_model(pcl, labels):
    cmap = plt.get_cmap("tab20")
    pcl_temp = copy.deepcopy(pcl)
    max_label = labels.max()
    colors = cmap(labels / (max_label if max_label > 0 else 1))
    pcl_temp.colors = o3d.utility.Vector3dVector(colors[:, :3])
    o3d.visualization.draw_geometries([pcl_temp], window_name="visualization",
                                      width=800, height=800, left=50, top=50,
                                      mesh_show_back_face=False)


# Calculate Euclidean distance
def euclidean_distance(one_sample, X):
    
    one_sample = one_sample.reshape(1, -1)
    
    X = X.reshape(X.shape[0], -1)
    
    distances = np.power(np.tile(one_sample, (X.shape[0], 1)) - X, 2).sum(axis=1)
    return distances


class Kmeans(object):
    
    def __init__(self, k=2, max_iterations=1500, tolerance=0.00001):
        self.k = k
        self.max_iterations = max_iterations
        self.tolerance = tolerance

    
    def init_random_centroids(self, X):
        # save the shape of X
        n_samples, n_features = np.shape(X)
        # make a zero matrix to store values
        centroids = np.zeros((self.k, n_features))
        
        for i in range(self.k):
            
            centroid = X[np.random.choice(range(n_samples))]
            centroids[i] = centroid
        return centroids

    def closest_centroid(self, sample, centroids):
        distances = euclidean_distance(sample, centroids)
        # np.argmin 
        closest_i = np.argmin(distances)
        return closest_i

    # cluster
    def create_clusters(self, centroids, X):
        # This is to construct a nested list for storing clusters
        clusters = [[] for _ in range(self.k)]
        for sample_i, sample in enumerate(X):
            centroid_i = self.closest_centroid(sample, centroids)
            clusters[centroid_i].append(sample_i)
        return clusters

    # Update centroids based on mean algorithm
    def update_centroids(self, clusters, X):
        n_features = np.shape(X)[1]
        centroids = np.zeros((self.k, n_features))
        for i, cluster in enumerate(clusters):
            centroid = np.mean(X[cluster], axis=0)
            centroids[i] = centroid
        return centroids

    # get label

    def get_cluster_labels(self, clusters, X):
        y_pred = np.zeros(np.shape(X)[0])
        for cluster_i, cluster in enumerate(clusters):
            for sample_i in cluster:
                y_pred[sample_i] = cluster_i
        return y_pred

    def predict(self, X):
        # select center point
        centroids = self.init_random_centroids(X)

        for _ in range(self.max_iterations):
            
            clusters = self.create_clusters(centroids, X)
            former_centroids = centroids
            
            centroids = self.update_centroids(clusters, X)
           
            diff = centroids - former_centroids
            if diff.any() < self.tolerance:
                break

        return self.get_cluster_labels(clusters, X)


if __name__ == "__main__":
    
    pcd = o3d.io.read_point_cloud('*.pcd')
    points = np.asarray(pcd.points)
    o3d.visualization.draw_geometries([pcd], window_name="visualization",
                                      width=800, height=800, left=50, top=50,
                                      mesh_show_back_face=False)
    # K-means cluster
    clf = Kmeans(k=14)
    labels = clf.predict(points)
   
    draw_labels_on_model(pcd, labels)

