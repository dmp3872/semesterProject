"""
File:           semester_project.py
Authors:        Kellen Bell (kdb9520), Derek Pruski (dmp3872)
Description:    TODO
"""

import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import os
import csv

"""
main:   Runs the program
params: none
return: none
"""
def main():

    # getting the names of all of the files
    pcdlist = []
    directory = "dataset\\PointClouds"

    dir = os.listdir(directory)

    # getting all the files for future reference
    for i in range(len(dir) - 1):
        path = os.path.join(directory, "{0}.pcd".format(i))
        if os.path.isfile(path):
            pcdlist.append(path)

    # geometry is the point cloud used in your animaiton
    # setting up the visualization
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    geometry = o3d.geometry.PointCloud()
    geometry.points = o3d.io.read_point_cloud(pcdlist[0]).points
    vis.add_geometry(geometry)

    # setting up the data that stores the information we will output later
    prevBBs = {}
    BBox_X_Min = {}
    BBox_X_Max = {}
    BBox_Y_Min = {}
    BBox_Y_Max = {}
    BBox_Z_Min = {}
    BBox_Z_Max = {}
    BBox_X = {}
    BBox_Y = {}
    BBox_Z = {}
    Vec_X = {}
    Vec_Y = {}
    Vec_Z = {}

    # used to give an id to the cluster
    freeid = 0

    # loop through all the pcds
    for i in range(len(pcdlist) - 1):

        # read in 2 pcds so we can compare the first frame to the next
        pcd = None
        nextpcd = None
        if os.path.isfile(pcdlist[i]):
            pcd = o3d.io.read_point_cloud(pcdlist[i])
        if os.path.isfile(pcdlist[i+1]):
            nextpcd = o3d.io.read_point_cloud(pcdlist[i+1])

        # get the distances between all points
        distances = pcd.compute_point_cloud_distance(nextpcd)

        outpoints = []

        # get the points we want if they moved more than 0.004
        t_dist = []
        for j in range(len(distances)):
            if distances[j] > 0.004:
                t_dist.append(distances[j])
                outpoints.append([pcd.points[j][0], pcd.points[j][1], pcd.points[j][2]])
        
        # use the points that moved for our geometry
        geometry.points = o3d.utility.Vector3dVector(outpoints)

        # cluster the points with min number 3 and eps 1.9
        with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
            labels = np.array(geometry.cluster_dbscan(eps=1.9, min_points=3, print_progress=True))

        # color the clusters
        colors = plt.get_cmap("tab10")(labels / (labels.max() if labels.max() > 0 else 1))
        unique_labels = set(labels) - {-1}
        points = np.asarray(geometry.points)

        # set up writing the csvs for each frame
        # make perception_results dir
        if not os.path.exists('perception_results'):
            os.makedirs('perception_results')
        csvfile =  open('perception_results\\frame_{0}.csv'.format(i), 'w', newline="")
        writer = csv.writer(csvfile)
        writer.writerow(["vehicle_id","position_x","position_y","position_z","mvec_x","mvec_y","mvec_z","bbox_x_min","bbox_x_max","bbox_y_min","bbox_y_max","bbox_z_min","bbox_z_max"])

        # get our center positions for calculating movement
        center_poses = []
        if len(prevBBs) > 0:
            for key in prevBBs.keys():
                center_poses.append([key, prevBBs.get(key).get_center()])

        # loop through all of the clusters
        for label in unique_labels:
            cluster_indices = np.where(labels == label)[0]
            cluster_points = points[cluster_indices]

            # Calculate bounding box
            min_bound = np.min(cluster_points, axis=0)
            max_bound = np.max(cluster_points, axis=0)

            # get min and max points
            x_min, y_min, z_min = min_bound
            x_max, y_max, z_max = max_bound

            # Create bounding box geometry
            bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound, max_bound)
            bbox.color = (1, 0, 0)  # Set color to red
            x, y, z = bbox.get_center()

            vec_x, vec_y, vec_z = 0, 0, 0

            found = False
            id = 0

            # if we have previous clusters to compare against
            if i != 0 and len(center_poses) > 0:
                # find the closest cluster
                _, index = Find_Nearest_Neighbor(bbox.get_center(), center_poses)

                # calculate vectors and update positions
                last_pos = center_poses.pop(index)
                id = last_pos[0]
                vec_x = x - last_pos[1][0]
                vec_y = y - last_pos[1][1]
                vec_z = z - last_pos[1][2]
                prevBBs[id] = bbox
                found = True

            # if no previous clusters
            if i == 0 or not found:
                # add cluster to list
                prevBBs[freeid] = bbox
                id = freeid
                freeid += 1

            # set all the vars with important info
            BBox_X_Min[id] = x_min
            BBox_X_Max[id] = x_max
            BBox_Y_Min[id] = y_min
            BBox_Y_Max[id] = y_max
            BBox_Z_Min[id] = z_min
            BBox_Z_Max[id] = z_max
            BBox_X[id] = x
            BBox_Y[id] = y
            BBox_Z[id] = z
            Vec_X[id] = vec_x
            Vec_Y[id] = vec_y
            Vec_Z[id] = vec_z

            # Add bounding box to visualizer
            vis.add_geometry(bbox)

        # remove clusters we couldn't find a match for in this frame
        for center in center_poses:
            id = center[0]
            prevBBs.pop(id)

        # write info to csv
        j = 0
        for id in prevBBs.keys():
            # IMPORTANT: grading script crashes if more than 6 objects are written to csv.  comment out the below line if this is fixed
            if j < 6:
                j += 1
                writer.writerow([id, BBox_X.get(id), BBox_Y.get(id), BBox_Z.get(id), Vec_X.get(id), Vec_Y.get(id), Vec_Z.get(id), 
                        BBox_X_Min.get(id), BBox_X_Max.get(id), BBox_Y_Min.get(id), BBox_Y_Max.get(id), BBox_Z_Min.get(id), BBox_Z_Max.get(id)])

        csvfile.close()

        # Add original point cloud to visualizer
        o3d.geometry.colors = o3d.utility.Vector3dVector(colors[:, :3])
        vis.add_geometry(geometry)
        vis.poll_events()
        vis.update_renderer()
        vis.clear_geometries()

def Find_3D_Distance ( pos_a, pos_b ):
    # Calculate the 3D distance between two points
    return np.sqrt( (pos_a[0] - pos_b[0])**2 + (pos_a[1] - pos_b[1])**2 + (pos_a[2] - pos_b[2])**2 )

def Find_Nearest_Neighbor ( pose, set_of_poses ):
    # Find the nearest neighbor to a given pose
    min_dist = 1000000000000
    min_index = -1
    for i in range(len(set_of_poses)):
        dist = Find_3D_Distance( pose, set_of_poses[i][1] )
        if dist < min_dist:
            min_dist = dist
            min_index = i
    return min_dist, min_index

if __name__ == "__main__":
    main()