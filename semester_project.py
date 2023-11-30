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

    for file in os.listdir(directory):
        path = os.path.join(directory, file)
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

        distances = pcd.compute_point_cloud_distance(nextpcd)

        outpoints = []

        t_dist = []
        for j in range(len(distances)):
            if distances[j] > 0.05:
                t_dist.append(distances[j])
                outpoints.append([pcd.points[j][0], pcd.points[j][1], pcd.points[j][2]])

        
        # now modify the points of your geometry
        # you can use whatever method suits you best, this is just an example
        geometry.points = o3d.utility.Vector3dVector(outpoints)
        vis.update_geometry(geometry)
        vis.poll_events()
        vis.update_renderer()

    # print(np.asarray(pcd0.points))
    # o3d.visualization.draw_geometries([pcd0],
    #                               zoom=0.3412,
    #                               front=[0.4257, -0.2125, -0.8795],
    #                               lookat=[2.6172, 2.0475, 1.532],
    #                               up=[-0.0694, -0.9768, 0.2024])

if __name__ == "__main__":
    main()