#!/usr/bin/env python
import rospy
import open3d as o3d
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import ros_numpy
import numpy as np
import random
import time

def callback(data):
    pc = ros_numpy.numpify(data)
    points=np.zeros((pc.shape[0],3))
    points[:,0]=pc['x']
    points[:,1]=pc['y']
    points[:,2]=pc['z']
    points = (np.array(points, dtype=np.float32))
    points = RemoveNan(points)
    downsampled = DownSample(points, voxel_size=0.01)
    #print('1: ',len(downsampled))
    points = RemoveNoiseStatistical(downsampled, nb_neighbors=500, std_ratio=0.1)
    #points = NumpyToPCD(points)
    plane_list = DetectMultiPlanes(points)
    # print(len(plane_list))
    #print(plane_list[0][1])
    planes = []
    #colors = []
    i = 0
    for _, plane in plane_list:
        # i+=1
        # r = random.random()
        # g = random.random()
        # b = random.random()
        # print(plane)
        # color = np.zeros((plane.shape[0], plane.shape[1]))
        # color[:, 0] = r
        # color[:, 1] = g
        # color[:, 2] = b

        planes.append(plane)
        #colors.append(color)
    
    planes = (np.concatenate(planes, axis=0))
    points = (np.array(planes, dtype=np.float32))
    points = RemoveNan(points)
    downsampled = DownSample(points, voxel_size=0.01)
    planes = (NumpyToPCD(downsampled))
    #colors = NumpyToPCD(np.concatenate(colors, axis=0))
    #DrawResult(planes, colors)
    # plane_model, inliers = points.segment_plane(distance_threshold=0.01,
    #                                      ransac_n=3,
    #                                      num_iterations=1000)
    # [a, b, c, d] = plane_model
    # print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")

    # inlier_cloud = points.select_by_index(inliers)
    # print(inlier_cloud)
    # inlier_cloud.paint_uniform_color([1.0, 0, 0])
    # outlier_cloud = points.select_by_index(inliers, invert=True)
    mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
    size=0.2, origin=[0, 0, 0])
    o3d.visualization.draw_geometries([planes,mesh_frame])
    

def PCDToNumpy(pcd):
    """  convert open3D point cloud to numpy ndarray
    Args:
        pcd (open3d.geometry.PointCloud): 
    Returns:
        [ndarray]: 
    """

    return np.asarray(pcd.points)


def NumpyToPCD(xyz):
    """ convert numpy ndarray to open3D point cloud 
    Args:
        xyz (ndarray): 
    Returns:
        [open3d.geometry.PointCloud]: 
    """

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz)

    return pcd

def RemoveNan(points):
    """ remove nan value of point clouds
    Args:
        points (ndarray): N x 3 point clouds
    Returns:
        [ndarray]: N x 3 point clouds
    """

    return points[~np.isnan(points[:, 0])]

def RemoveNoiseStatistical(pc, nb_neighbors=20, std_ratio=2.0):
    """ remove point clouds noise using statitical noise removal method
    Args:
        pc (ndarray): N x 3 point clouds
        nb_neighbors (int, optional): Defaults to 20.
        std_ratio (float, optional): Defaults to 2.0.
    Returns:
        [ndarray]: N x 3 point clouds
    """

    pcd = NumpyToPCD(pc)
    cl, ind = pcd.remove_statistical_outlier(
        nb_neighbors=nb_neighbors, std_ratio=std_ratio)

    return PCDToNumpy(cl)

def DownSample(pts, voxel_size):
    """ down sample the point clouds
    Args:
        pts (ndarray): N x 3 input point clouds
        voxel_size (float, optional): voxel size. Defaults to 0.003.
    Returns:
        [ndarray]: 
    """

    p = NumpyToPCD(pts).voxel_down_sample(voxel_size=voxel_size)

    return PCDToNumpy(p)

def PlaneRegression(points, threshold=0.01, init_n=3, iter=1000):
    """ plane regression using ransac
    Args:
        points (ndarray): N x3 point clouds
        threshold (float, optional): distance threshold. Defaults to 0.003.
        init_n (int, optional): Number of initial points to be considered inliers in each iteration
        iter (int, optional): number of iteration. Defaults to 1000.
    Returns:
        [ndarray, List]: 4 x 1 plane equation weights, List of plane point index
    """

    pcd = NumpyToPCD(points)

    w, index = pcd.segment_plane(
        threshold, init_n, iter)
    #inlier_cloud = points.select_by_index(index)
    #outlier_cloud = points.select_by_index(index, invert=True)
    return w, index

def DetectMultiPlanes(points, min_ratio=0.05, threshold=0.01, iterations=1000):
    """ Detect multiple planes from given point clouds
    Args:
        points (np.ndarray): 
        min_ratio (float, optional): The minimum left points ratio to end the Detection. Defaults to 0.05.
        threshold (float, optional): RANSAC threshold in (m). Defaults to 0.01.
    Returns:
        [List[tuple(np.ndarray, List)]]: Plane equation and plane point index
    """

    plane_list = []
    N = len(points)
    target = points.copy()
    count = 0

    while count < (1 - min_ratio) * N:
        w, index = PlaneRegression(
            target, threshold=threshold, init_n=3, iter=iterations)
    
        count += len(index)
        plane_list.append((w, target[index]))
        target = np.delete(target, index, axis=0)

    return plane_list


def DrawResult(points,colors):
    pcd = o3d.geometry.PointCloud(points)
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors)
    o3d.visualization.draw_geometries([pcd])

rospy.init_node('listener', anonymous=True)
rospy.Subscriber("/camera/depth/color/points", PointCloud2, callback)
rospy.spin()
