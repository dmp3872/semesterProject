"""
File:           semester_project.py
Authors:        Kellen Bell (kdb9520), Derek Pruski (dmp3872)
Description:    TODO
"""

import open3d as o3d
import numpy as np

"""
main:   Runs the program
params: none
return: none
"""
def main():
    # open pointCloud and view
    ply_point_cloud = o3d.data.PLYPointCloud()
    pcd0 = o3d.io.read_point_cloud('dataset\\PointClouds\\0.pcd')
    pcd1 = o3d.io.read_point_cloud('dataset\\PointClouds\\1.pcd')

    distances = pcd0.compute_point_cloud_distance(pcd1)

    for i, dist in enumerate(distances):
        if dist > 0.0009:
            print(f"Point {i}: Distance = {dist:.4f}")

    # print(np.asarray(pcd.points))
    # o3d.visualization.draw_geometries([pcd],
    #                               zoom=0.3412,
    #                               front=[0.4257, -0.2125, -0.8795],
    #                               lookat=[2.6172, 2.0475, 1.532],
    #                               up=[-0.0694, -0.9768, 0.2024])

if __name__ == "__main__":
    main()