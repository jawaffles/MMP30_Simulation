#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64
from time import sleep
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import ros_numpy

# This import registers the 3D projection, but is otherwise unused.
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 unused import

# import matplotlib.pyplot as plt
from matplotlib import pyplot as plt
import numpy as np
import time
import open3d as o3d
from sklearn import linear_model, datasets

import math  

def calculateDistance(x1,y1,x2,y2):  
     dist = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)  
     return dist  


def pcd_row_detection(pcd):
        #Downsampling pcd
                # print("Downsample the point cloud with a voxel of 0.05")
                downpcd = pcd.voxel_down_sample(voxel_size=0.05)
                # o3d.visualization.draw_geometries([downpcd])

        #Crop PCD File
                #   vol = o3d.visualization.read_selection_polygon_volume("/home/biosensing/catkin_ws/src/mmp30_gazebo/src/crop/crop.json")
                vol = o3d.visualization.SelectionPolygonVolume()
                vol.axis_min = .1
                vol.axis_max = 10.0

                bp_pt_botright = [0,0]
                bp_pt_topleft = [5,1]

                bounding_polygon_l =  np.array([ 
                [ 0.0, 1.0, 0.0],
                [ 5.0, 1.0, 0.0],
                [ 5.0, 0.0, 0.0],
                [ 0.0, 0.0, 0.0]
                ]).astype("float64")

                bounding_polygon_r =  np.array([ 
                [ 0.0, -1.0, 0.0],
                [ 5.0, -1.0, 0.0],
                [ 5.0, 0.0, 0.0],
                [ 0.0, 0.0, 0.0]
                ]).astype("float64")
                
                vol.bounding_polygon = o3d.utility.Vector3dVector(bounding_polygon_r)

                vol.orthogonal_axis = "Z"


                cropped_pcd_r = vol.crop_point_cloud(downpcd)

                vol.bounding_polygon = o3d.utility.Vector3dVector(bounding_polygon_l)

                cropped_pcd_l = vol.crop_point_cloud(downpcd)

                # o3d.visualization.draw_geometries([cropped_pcd_l])
                # o3d.visualization.draw_geometries([cropped_pcd_r])
       
        #Raduis outlier filter
                # print("filtered")

                filter_r = cropped_pcd_r.remove_radius_outlier(50,.25)

                # o3d.visualization.draw_geometries_with_editing([filter_r[0]])

                filtered_pcd_r = filter_r[0]
       
                filter_l = cropped_pcd_l.remove_radius_outlier(50,.25)

                # o3d.visualization.draw_geometries_with_editing([filter_r[0]])
                filtered_pcd_l = filter_l[0]
                # o3d.visualization.draw_geometries_with_editing([filtered_pcd_l])
                # o3d.visualization.draw_geometries_with_editing([filtered_pcd_r])

        #XYZ array point cloud + Reshape

                xyz_cropped_pcd_points_r = filtered_pcd_r.points
                xyz_cropped_pcd_points_l = filtered_pcd_l.points



                xyz_array_r = np.asarray(xyz_cropped_pcd_points_r)
                xyz_array_l = np.asarray(xyz_cropped_pcd_points_l)


                X_r = xyz_array_r[:,0]
                X_r = X_r.reshape(-1, 1)
                y_r = xyz_array_r[:,1]
                y_r = y_r.reshape(-1, 1)

                X_l = xyz_array_l[:,0]
                X_l = X_l.reshape(-1, 1)
                y_l = xyz_array_l[:,1]
                y_l = y_l.reshape(-1, 1)

        # Plot PCD Cloud
                # fig = plt.figure()
                # print(xyz_array[:,1])
                # ax = fig.add_subplot(111, projection='3d')
                # ax = fig
                # plt.scatter(xs, ys, color='yellowgreen' ,marker='.')
                # # ax.scatter(xs, ys, zs , color='yellowgreen' ,marker='.')
                # # ax.set_xlabel('X Label')
                # # ax.set_ylabel('Y Label')
                # # ax.set_zlabel('Z Label')
                # plt.show()

        # Fit Data for Left Row
                n_samples = 1000
                n_outliers = 50
                coef = True
                # Fit line using all data
                lr_l = linear_model.LinearRegression()
                lr_l.fit(X_l, y_l)
                # Robustly fit linear model with RANSAC algorithm
                ransac = linear_model.RANSACRegressor()
                ransac.fit(X_l, y_l)
                inlier_mask = ransac.inlier_mask_
                outlier_mask = np.logical_not(inlier_mask)
                # Predict data of estimated models
                line_X_l = np.arange(X_l.min(), X_l.max())[:, np.newaxis]
                line_y_l = lr_l.predict(line_X_l)
                line_y_l_ransac = ransac.predict(line_X_l)
                # Compare estimated coefficients
                # print("Estimated coefficients (true, linear regression, RANSAC):")
                # print(coef, lr_l.coef_, ransac.estimator_.coef_)
                lw = 2
                plt.scatter(X_l[inlier_mask], y_l[inlier_mask], color='yellowgreen', marker='.',
                label='Inliers')
                plt.scatter(X_l[outlier_mask], y_l[outlier_mask], color='gold', marker='.',
                label='Outliers')
                plt.plot(line_X_l, line_y_l, color='navy', linewidth=lw, label='Linear regressor')
                # plt.plot(line_X_l, line_y_l_ransac, color='cornflowerblue', linewidth=lw,
                # label='RANSAC regressor')
                plt.legend(loc='lower right')
                plt.xlabel("Input")
                plt.ylabel("Response")
                
        # Fit Data for Right Row
                n_samples = 1000
                n_outliers = 50
                coef = True
                # Fit line using all data
                lr_r = linear_model.LinearRegression()
                lr_r.fit(X_r, y_r)
                # Robustly fit linear model with RANSAC algorithm
                ransac_r = linear_model.RANSACRegressor()
                ransac_r.fit(X_r, y_r)
                inlier_mask_r = ransac_r.inlier_mask_
                outlier_mask_r = np.logical_not(inlier_mask_r)
                # Predict data of estimated models
                line_X_r = np.arange(X_r.min(), X_r.max())[:, np.newaxis]
                line_y_r = lr_r.predict(line_X_r)
                line_y_r_ransac = ransac_r.predict(line_X_r)
                # Compare estimated coefficients
                # print("Estimated coefficients (true, linear regression, RANSAC):")
                # print(coef, lr_r.coef_, ransac_r.estimator_.coef_)
                lw = 2
                plt.scatter(X_r[inlier_mask_r], y_r[inlier_mask_r], color='yellowgreen', marker='.',
                label='Inliers')
                plt.scatter(X_r[outlier_mask_r], y_r[outlier_mask_r], color='gold', marker='.',
                label='Outliers')
                plt.plot(line_X_r, line_y_r, color='navy', linewidth=lw, label='Linear regressor')
                # plt.plot(line_X_r, line_y_r_ransac, color='cornflowerblue', linewidth=lw,
                # label='RANSAC regressor')

        #Desired Heading + DISTANCE +  final Plot
                plt.plot([line_X_r[0],line_X_r[1]], [0,0], color='black', linewidth=lw, label='Rover Heading')           
                # print("left row heading: %f"% lr_l.coef_[0])
                # print("left row heading: %f" % lr_r.coef_[0])
                avg_heading = ((lr_l.coef_[0] + lr_r.coef_[0]) /2)
                # print("Avg Heading : %f" % avg_heading)

                plt.plot(line_X_r , avg_heading*line_X_r -(line_X_r[1]*avg_heading) , color='red' )
                plt.show()


                r_dist = calculateDistance(0,0,line_X_r[0],line_y_r[0])
                l_dist = calculateDistance(0,0,line_X_l[0],line_y_l[0])


                row_output = [avg_heading,r_dist,l_dist]

                return row_output



if __name__ == "__main__":

        #PCD Initial Read
        print("Load a ply point cloud, print it, and render it")
        pcd = o3d.io.read_point_cloud("/home/biosensing/catkin_ws/src/mmp30_gazebo/pcd/20191216/nod/545713000.pcd")
        print(pcd)
        # print(np.asarray(pcd.points))
        o3d.visualization.draw_geometries([pcd])

        print("Downsample the point cloud with a voxel of 0.05")
        downpcd = pcd.voxel_down_sample(voxel_size=0.05)
        o3d.visualization.draw_geometries([downpcd])

        pcd_row_detection(downpcd)
