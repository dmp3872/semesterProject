"""
File:           semester_project.py
Authors:        Kellen Bell (kdb9520), Derek Pruski (dmp3872)
Description:    TODO
"""

import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import os
import time

"""
main:   Runs the program
params: none
return: none
"""
def main():
    # open pointCloud and view
    ply_point_cloud = o3d.data.PLYPointCloud()
    # pcd0 = o3d.io.read_point_cloud('dataset\\PointClouds\\0.pcd')
    # pcd1 = o3d.io.read_point_cloud('dataset\\PointClouds\\1.pcd')
    # pcd2 = o3d.io.read_point_cloud('dataset\\PointClouds\\2.pcd')

    # distances = pcd0.compute_point_cloud_distance(pcd1)
    # distances2 = pcd1.compute_point_cloud_distance(pcd2)

    pcdlist = []
    directory = "dataset\\PointClouds"

    dir = os.listdir(directory)

    for i in range(len(dir) - 1):
        path = os.path.join(directory, "{0}.pcd".format(i))
        if os.path.isfile(path):
            pcdlist.append(path)

    # pcd = o3d.io.read_point_cloud("dataset\\PointClouds\\401.pcd")

    # with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
    #     labels = np.array(pcd.cluster_dbscan(eps=0.02, min_points=10, print_progress=True))

    #     max_label = labels.max()
    #     print(f"point cloud has {max_label + 1} clusters")
    #     colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
    #     colors[labels < 0] = 0
    #     pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])
    #     o3d.visualization.draw_geometries([pcd], zoom=0.455,
    #                                     front=[-0.4999, -0.1659, -0.8499],
    #                                     lookat=[2.1813, 2.0619, 2.0999],
    #                                     up=[0.1204, -0.9852, 0.1215])

    # geometry is the point cloud used in your animaiton
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    geometry = o3d.geometry.PointCloud()
    geometry.points = o3d.io.read_point_cloud(pcdlist[0]).points
    vis.add_geometry(geometry)

    for i in range(len(pcdlist) - 1):

        pcd = None
        nextpcd = None
        if os.path.isfile(pcdlist[i]):
            pcd = o3d.io.read_point_cloud(pcdlist[i])
        if os.path.isfile(pcdlist[i+1]):
            nextpcd = o3d.io.read_point_cloud(pcdlist[i+1])

        # plane_model, inliers = pcd.segment_plane(distance_threshold=0.6, ransac_n=5, num_iterations=1000)
        # pcd = pcd.select_by_index(inliers, invert=True)
        # plane_model, inliers = nextpcd.segment_plane(distance_threshold=0.6, ransac_n=5, num_iterations=1000)
        # nextpcd = pcd.select_by_index(inliers, invert=True)

        distances = pcd.compute_point_cloud_distance(nextpcd)

        outpoints = []

        t_dist = []
        for j in range(len(distances)):
            if distances[j] > 0.001:
                t_dist.append(distances[j])
                outpoints.append([pcd.points[j][0], pcd.points[j][1], pcd.points[j][2]])


        
        # now modify the points of your geometry
        # you can use whatever method suits you best, this is just an example
        geometry.points = o3d.utility.Vector3dVector(outpoints)
        with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
            labels = np.array(geometry.cluster_dbscan(eps=1.6, min_points=5, print_progress=True))

        colors = plt.get_cmap("tab10")(labels / (labels.max() if labels.max() > 0 else 1))
        unique_labels = set(labels) - {-1}
        points = np.asarray(geometry.points)

        BBox_X_Min = len(unique_labels) * [0]
        BBox_X_Max = len(unique_labels) * [0]
        BBox_Y_Min = len(unique_labels) * [0]
        BBox_Y_Max = len(unique_labels) * [0]
        BBox_Z_Min = len(unique_labels) * [0]
        BBox_Z_Max = len(unique_labels) * [0]

        for label in unique_labels:
            cluster_indices = np.where(labels == label)[0]
            cluster_points = points[cluster_indices]

            # Calculate bounding box
            min_bound = np.min(cluster_points, axis=0)
            max_bound = np.max(cluster_points, axis=0)

            x_min, y_min, z_min = min_bound
            x_max, y_max, z_max = max_bound

            BBox_X_Min[label] = x_min
            BBox_X_Max[label] = x_max
            BBox_Y_Min[label] = y_min
            BBox_Y_Max[label] = y_max
            BBox_Z_Min[label] = z_min
            BBox_Z_Max[label] = z_max

            # Create bounding box geometry
            bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound, max_bound)
            bbox.color = (1, 0, 0)  # Set color to red

            # Add bounding box to visualizer
            vis.add_geometry(bbox)

        frame = i

        # Add original point cloud to visualizer
        o3d.geometry.colors = o3d.utility.Vector3dVector(colors[:, :3])
        vis.update_geometry(geometry)
        vis.poll_events()
        vis.update_renderer()


    pcd = o3d.io.read_point_cloud("dataset\\PointClouds\\482.pcd")

    plane_model, inliers = pcd.segment_plane(distance_threshold=0.6, ransac_n=5, num_iterations=1000)
    pcd = pcd.select_by_index(inliers, invert=True)

    with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
        labels = np.array(pcd.cluster_dbscan(eps=.6, min_points=8, print_progress=True))

    # print(np.asarray(pcd0.points))
    # o3d.visualization.draw_geometries([pcd0],
    #                               zoom=0.3412,
    #                               front=[0.4257, -0.2125, -0.8795],
    #                               lookat=[2.6172, 2.0475, 1.532],
    #                               up=[-0.0694, -0.9768, 0.2024])

def visualize_clusters(pcd, labels):
    # Visualize clustered dynamic objects
    colors = plt.get_cmap("tab10")(labels / (labels.max() if labels.max() > 0 else 1))
    pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])
    o3d.visualization.draw_geometries([pcd])

if __name__ == "__main__":
    main()