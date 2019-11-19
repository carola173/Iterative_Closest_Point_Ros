#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from dynamic_reconfigure.server import Server as DynamicReconfigureServer #dynamic reconfiguration server
from ros_icp.cfg import dynamicConfigConfig
import math
import matplotlib.pyplot as plt
import numpy as np


class icp:
    def callback(self, config, level):
        self.number_points=config["number_points"] # Re-configurable parameter for accepting the string value
        self.motion_x=config["motion_x"] # Re-configurable parameter for  defining the motion/shift in x
        self.motion_y=config["motion_y"] # Re-configurable parameter for  defining the motion in y
        self.yaw=config["yaw"] # Re-configurable parameter for  defining the yaw change
        self.display_simulator=config["display_simulator"] #Re-configurable parameter to show simulator or not
        self.number_simulator=config["number_simulator"] # Re-configurable for diplaying the simulator "n" of times
        self.icp_error_threshold=config["icp_error_threshold"] #Re-configurable parameter for having the maximum error threshold for the ICP algorithm
        self.max_icp_iter=config["max_icp_iter"] # Re-configurable parameter to run the ICP for the convergence
        return config


    def update_h_matrix(self,curr_h,R,T):
        '''
            Homogenous matrix that stores the transalation vector and the rotation matrix
        '''
        H= np.zeros((3, 3))
        H[0, 0] = R[0, 0]
        H[1, 0] = R[1, 0]
        H[0, 1] = R[0, 1]
        H[1, 1] = R[1, 1]
        H[2, 2] = 1.0
        H[0, 2] = T[0]
        H[1, 2] = T[1]
        
        if curr_h is None:
            return H
        else:
            return np.matmul(curr_h,H)

    def calculate_association(self,previous_points, current_points):
        '''
            Association between the current and the previous point is calculated using the concept of lower distance between the two points
        '''
        # calc the sum of residual errors
        delta_points = previous_points - current_points
        d = np.linalg.norm(delta_points, axis=0)
        error = sum(d)
        # calc index with nearest neighbor assosiation
        d = np.linalg.norm(np.repeat(current_points, previous_points.shape[1], axis=1)
                       - np.tile(previous_points, (1, current_points.shape[1])), axis=0)
        indexes = np.argmin(d.reshape(current_points.shape[1], previous_points.shape[1]), axis=1)

        return indexes, error
    def svd_motion_estimation(self,previous_points, current_points):
        pm = np.mean(previous_points, axis=1)
        cm = np.mean(current_points, axis=1)

        p_shift = previous_points - pm[:, np.newaxis]
        c_shift = current_points - cm[:, np.newaxis]

        W = np.matmul(c_shift , p_shift.T)
        u, s, vh = np.linalg.svd(W)

        R = np.matmul(u , vh).T
        t = pm - np.matmul(R , cm)
        return R, t


    def plot(self,previous_points, current_points):
        plt.cla()
        plt.plot(previous_points[0, :], previous_points[1, :], ".r")
        plt.plot(current_points[0, :], current_points[1, :], ".b")
        plt.plot(0.0, 0.0, "xr")
        plt.axis("equal")
        plt.pause(0.1)

    def icp_algo(self,previous_points, current_points):
        '''
          Implementation of ICP Algorithm to find the correspondance between the the current and the previous frame points
          Input : previous and current frame points
          output : Rotation matirx and Translation vector required 
        '''

        count=0
        while self.total_error >= self.icp_error_threshold:
            count+=1
            if self.display_simulator ==1:
                self.plot(previous_points, current_points)
            
            id,error=self.calculate_association(previous_points, current_points)
            Rotation,translation=self.svd_motion_estimation(previous_points[:, id], current_points)

            current_points = np.matmul(Rotation , current_points) + translation[:, np.newaxis]

            self.H = self.update_h_matrix(self.H, Rotation, translation)

            self.total_error=abs(self.previous_error-error)

            self.previous_error= error

            rospy.loginfo("Residual Error %d",error )
            
            if self.total_error <= self.icp_error_threshold:
                print("Converge", error, self.total_error, count)
                break
            elif self.max_icp_iter <= count:
                print("Not Converge...", error, self.total_error, count)
                break

        R = np.array(self.H[0:2, 0:2])
        T = np.array(self.H[0:2, 2])

        return R,T


    def random_point_gen(self):
        for _ in range(self.number_simulator):
            # Defining the previous points
            px = (np.random.rand(self.number_points) - 0.5) * self.fieldLength
            py = (np.random.rand(self.number_points) - 0.5) * self.fieldLength
            print px,py
            previous_points = np.vstack((px, py))
            
            # current points
            cx = [math.cos(self.motion[2]) * x - math.sin(self.motion[2]) * y + self.motion[0]
                  for (x, y) in zip(px, py)]
            cy = [math.sin(self.motion[2]) * x + math.cos(self.motion[2]) * y + self.motion[1]
                  for (x, y) in zip(px, py)]
            current_points = np.vstack((cx, cy))

            self.total_error=1000
            self.previous_error=1000
            self.H=None

            #Calculating the ICP for the previous and the current points

            R, T = self.icp_algo(previous_points, current_points)
            print("Rotation:", R)
            print("Translation:", T)

    def __init__(self):
        rospy.init_node('IcpRos', anonymous=True) 
        #Configuring the parameter for the simulation
        self.server = DynamicReconfigureServer(dynamicConfigConfig, self.callback)
        self.fieldLength = 50.0
        self.motion = [ self.motion_x,  self.motion_y, np.deg2rad(self.yaw)]  # movement [x[m],y[m],yaw[deg]]
        self.H= None # Defining the homogenous matrix
        self.total_error = 1000.0 # stores the error in ICP from previous run to current ICP run, for intialization set to high value
        self.previous_error = 1000.0 # stores the error in previous ICP algo run, for intialization set to high value
        



if __name__=="__main__":
    try:
        i=icp()
        i.random_point_gen()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()
