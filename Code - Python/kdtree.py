# kdtree

import random
import math
import numpy as np

from result_set import KNNResultSet, RadiusNNResultSet

class Node:
    def __init__(self, axis, value, left, right, point_indices):
        self.axis = axis   
        self.value = value  
        self.left = left    
        self.right = right  
        self.point_indices = point_indices   

    def is_leaf(self):
        if self.value is None:
            return True
        else:
            return False

    def __str__(self):   
        output = ''
        output += 'axis %d, ' % self.axis
        if self.value is None:
            output += 'split value: leaf, '
        else:
            output += 'split value: %.2f, ' % self.value
        output += 'point_indices: '
        output += str(self.point_indices.tolist())
        return output


# input：
#     key
#     value
# output：
#     key_sorted
#     value_sorted
def sort_key_by_vale(key, value)
    assert key.shape == value.shape
    assert len(key.shape) == 1
    sorted_idx = np.argsort(value)
    key_sorted = key[sorted_idx]
    value_sorted = value[sorted_idx]
    return key_sorted, value_sorted


def axis_round_robin(axis, dim):  
    if axis == dim-1:
        return 0
    else:
        return axis + 1

# 
# input：
#     root
#     db: point cloud
#     point_indices: key
#     axis: scalar
#     leaf_size: scalar
# output：
#     root
def kdtree_recursive_build(root, db, point_indices, axis, leaf_size): #db is point cloud，N*3
    if root is None:
        root = Node(axis, None, None, None, point_indices)

    # determine whether to split into left and right
    if len(point_indices) > leaf_size:
        # --- get the split position ---
        point_indices_sorted, _ = sort_key_by_vale(point_indices, db[point_indices, axis])  # M

        mid_index=int(np.floor(point_indices_sorted.shape[0] / 2))
        mid_left_value_index = point_indices_sorted[mid_index]
        mid_right_value_index = point_indices_sorted[mid_index+1]
        root.value=(db[mid_left_value_index,axis]+db[mid_right_value_index,axis])/2.0
        left_child_point_indices=point_indices_sorted[:mid_index]
        right_child_point_indices=point_indices_sorted[mid_index:]
      
        dim=db.shape[1] 
        axis_child=axis_round_robin(axis,dim) 
        
        root.right = kdtree_recursive_build(root.right, db, right_child_point_indices, axis_child, leaf_size)
        root.left = kdtree_recursive_build(root.left, db, left_child_point_indices, axis_child, leaf_size)
       
    return root


# input：
#     root
#     depth
#     max_depth
def traverse_kdtree(root: Node, depth, max_depth):
    depth[0] += 1
    if max_depth[0] < depth[0]:
        max_depth[0] = depth[0]

    if root.is_leaf():
        print(root)
    else:
        traverse_kdtree(root.left, depth, max_depth)
        traverse_kdtree(root.right, depth, max_depth)

    depth[0] -= 1


# input：
#     db_np：original data
#     leaf_size：scale
# output：
#     root：kdtree
def kdtree_construction(db_np, leaf_size):
    N, dim = db_np.shape[0], db_np.shape[1]

    # build kd_tree recursively
    root = None
    root = kdtree_recursive_build(root,
                                  db_np,
                                  np.arange(N),
                                  axis=0,
                                  leaf_size=leaf_size)
    return root


# input：
#     root: kdtree
#     db: original data
#     result_set：search
#     query：index
# output：
#     else False
def kdtree_knn_search(root: Node, db: np.ndarray, result_set: KNNResultSet, query: np.ndarray):
    if root is None:
        return False

    if root.is_leaf():
        # compare the contents of a leaf
        leaf_points = db[root.point_indices, :]
        diff = np.linalg.norm(np.expand_dims(query, 0) - leaf_points, axis=1)
        for i in range(diff.shape[0]):
            result_set.add_point(diff[i], root.point_indices[i])
        return False

   
   
    current_axis=root.axis   
    if query[current_axis]<=root.value:  
        kdtree_knn_search(root.left,db,result_set,query)
        if np.fabs(query[current_axis]-root.value)<result_set.worstDist(): 
            kdtree_knn_search(root.right, db, result_set, query)
    
    else:
        kdtree_knn_search(root.right,db,result_set,query)
        if np.fabs(query[current_axis]-root.value)<result_set.worstDist(): 
            kdtree_knn_search(root.left, db, result_set, query)

 

    return False

# input：
#     root: kdtree
#     db: original data
#     result_set:search
#     query：index
# output：
#     else False
def kdtree_radius_search(root: Node, db: np.ndarray, result_set: RadiusNNResultSet, query: np.ndarray):
    if root is None:
        return False

    if root.is_leaf():
        # compare the contents of a leaf
        leaf_points = db[root.point_indices, :]
        diff = np.linalg.norm(np.expand_dims(query, 0) - leaf_points, axis=1)
        for i in range(diff.shape[0]):
            result_set.add_point(diff[i], root.point_indices[i])
        return False
    
    current_axis = root.axis  
    if query[current_axis] <= root.value:  
        kdtree_radius_search(root.left, db, result_set, query)
        if np.fabs(query[current_axis] - root.value) < result_set.worstDist():  
            kdtree_radius_search(root.right, db, result_set, query)
    
    else:
        kdtree_radius_search(root.right, db, result_set, query)
        if np.fabs(query[current_axis] - root.value) < result_set.worstDist():  
            kdtree_radius_search(root.left, db, result_set, query)

    return False



def main():
    # configuration
    db_size = 64
    dim = 3
    leaf_size = 4
    k = 3

    db_np = np.random.rand(db_size, dim)

    root = kdtree_construction(db_np, leaf_size=leaf_size)

    depth = [0]
    max_depth = [0]
    traverse_kdtree(root, depth, max_depth)
    print("tree max depth: %d" % max_depth[0])

    query = np.asarray([0, 0, 0])
    result_set = KNNResultSet(capacity=k)
    kdtree_knn_search(root, db_np, result_set, query)

    print(result_set)

    diff = np.linalg.norm(np.expand_dims(query, 0) - db_np, axis=1)
    nn_idx = np.argsort(diff)
    nn_dist = diff[nn_idx]
    print(nn_idx[0:k])
    print(nn_dist[0:k])


    # print("Radius search:")
    # query = np.asarray([0, 0, 0])
    # result_set = RadiusNNResultSet(radius = 0.5)
    # radius_search(root, db_np, result_set, query)
    # print(result_set)


if __name__ == '__main__':
    main()