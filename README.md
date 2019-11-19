# Iterative_Closest_Point_Ros
ICP algorithm implementation on ROS

Launch file to be launched : icp_ros.launch

ICP Algorithm is used to compute the translation and rotation matrix between the current and the previous set of points in such a way that it reduce the mean square error between the current_point and the previous point.ICP algorithm converge to a local minimum [1]. The algorithm terminates when the matching step does not change any of the previous matchings.
 
ICP doesn't assume the correspondence between the points, rather it calculates based on the concept of SVD-based least-squared best-fit algorithm for corresponding point sets.

The Nearest point association is done in the fuction "calculate_association(previous_points, current_points)" - This function uses the concept of lower error threshold between the two points set and associated the points which has the lower threshold with each other

The SVD algorithm implementation is done in the function :svd_motion_estimation(previous_points, current_points) - This function find the optimal alignment between corresponding points 

                    – Source points p1,…,pn with centroid location 
                    – Target points q1,…,qn with centroid location
                      • qi is the corresponding point of pi
                    – After centroid alignment and rotation by some R, a transformed source point is located
                    – find the R that minimizes sum of pair-wise distances 
                     
                        

Translation T is [px,py]= V.[px,py] where V= [vx , vy]

Rotation    R is [px,py] = R.[px,py] where R=[[cos(angle),sin(angle)],[sin(angle),cos(angle)]]
