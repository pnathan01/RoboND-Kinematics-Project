#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
		
        ### Your FK code here
        # Create symbols
	#

        print("Poses:")
        print(req.poses)

        a = pi.evalf()
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8') # Link Offset
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7') # Link Lengths
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')  # Twist angles
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')  # joint angles

	#   
	# Create Modified DH parameters
	#

        DH_Table = { alpha0:      0, a0:      0, d1:  0.75, q1:          q1,
	    	     alpha1:  -a/2., a1:   0.35, d2:     0, q2:  -a/2. + q2,
		     alpha2:      0, a2:   1.25, d3:     0, q3:          q3,
		     alpha3:  -a/2., a3: -0.054, d4:   1.5, q4:          q4,
		     alpha4:   a/2., a4:      0, d5:     0, q5:          q5,
		     alpha5:  -a/2., a5:      0, d6:     0, q6:          q6,
		     alpha6:      0, a6:      0, d7: 0.303, q7:           0}

	#            
	# Define Modified DH Transformation matrix
	#

        def TF_Matrix(alpha, a, d, q):
	     return Matrix([[           cos(q),           -sin(q),           0,             a],
		         [sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
		         [sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),   cos(alpha)*d],
		         [                0,                 0,           0,             1]])
        

	#
	# Create individual transformation matrices
	#

        alpha00 = alpha0.evalf(subs=DH_Table)
        a00 = a0.evalf(subs=DH_Table)
        d11 = d1.evalf(subs=DH_Table)
        T0_1  = TF_Matrix(alpha00, a00, d11, q1).subs(DH_Table)

        alpha11 = alpha1.evalf(subs=DH_Table)
        a11 = a1.evalf(subs=DH_Table)
        d22 = d2.evalf(subs=DH_Table)
        T1_2  = TF_Matrix(alpha11, a11, d22, q2).subs(DH_Table)

        alpha22 = alpha2.evalf(subs=DH_Table)
        a22 = a2.evalf(subs=DH_Table)
        d33 = d3.evalf(subs=DH_Table)
        T2_3  = TF_Matrix(alpha22, a22, d33, q3).subs(DH_Table)

        alpha33 = alpha3.evalf(subs=DH_Table)
        a33 = a3.evalf(subs=DH_Table)
        d44 = d4.evalf(subs=DH_Table)
        T3_4  = TF_Matrix(alpha33, a33, d44, q4).subs(DH_Table)

        alpha44 = alpha4.evalf(subs=DH_Table)
        a44 = a4.evalf(subs=DH_Table)
        d55 = d5.evalf(subs=DH_Table)
        T4_5  = TF_Matrix(alpha44, a44, d55, q5).subs(DH_Table)

        alpha55 = alpha5.evalf(subs=DH_Table)
        a55 = a5.evalf(subs=DH_Table)
        d66 = d6.evalf(subs=DH_Table)
        T5_6  = TF_Matrix(alpha55, a55, d66, q6).subs(DH_Table)

        alpha66 = alpha6.evalf(subs=DH_Table)
        a66 = a6.evalf(subs=DH_Table)
        d77 = d7.evalf(subs=DH_Table)
        T6_EE = TF_Matrix(alpha66, a66, d77, q7).subs(DH_Table)

	#
	# Extract rotation matrices from the transformation matrices
	#

        T0_EE = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_EE
        T0_EE = T0_EE.evalf()

	#
        ###

        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

	    # Extract end-effector position and orientation from request
	    # px,py,pz = end-effector position
	    # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])
     
            ### Your IK code here 
	    # Compensate for rotation discrepancy between DH parameters and Gazebo
	    #

            r, p, y = symbols('r p y')
    
            ROT_x = Matrix([[1,      0,       0],
    		            [0, cos(r), -sin(r)],
		            [0, sin(r),  cos(r)]])

            ROT_y = Matrix([[ cos(p), 0, sin(p)],
		            [      0, 1,      0],
		            [-sin(p), 0, cos(p)]])

            ROT_z = Matrix([[cos(y), -sin(y), 0],
		            [sin(y),  cos(y), 0],
		            [     0,       0, 1]])

            ROT_EE = ROT_z * ROT_y * ROT_x

            Rot_Error = ROT_z.subs(y, radians(180)) * ROT_y.subs(p, radians(-90))

            ROT_EE = ROT_EE * Rot_Error
            ROT_EE = ROT_EE.subs({'r':roll, 'p': pitch, 'y': yaw})

            EE = Matrix([[px],
                         [py],
		         [pz]])
            WC = EE - (0.303) * ROT_EE[:,2]

	    #
	    # Calculate joint angles using Geometric IK method
	    #

            theta1 = atan2(WC[1], WC[0])
            side_a = 1.501
            side_b = sqrt(pow((sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35), 2) + pow((WC[2] - 0.75), 2))
            side_c = 1.25

            angle_a = acos((side_b * side_b + side_c * side_c - side_a * side_a)/(2 * side_b * side_c))
            angle_b = acos((side_a * side_a + side_c * side_c - side_b * side_b)/(2 * side_a * side_c))
            angle_c = acos((side_a * side_a  * + side_b * side_b - side_c * side_c)/(2 * side_a * side_b))

            theta2 = pi/2. - angle_a - atan2(WC[2] - 0.75, sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35)
            theta3 = pi/2. - (angle_b + 0.036)

            R0_3 = T0_1[0:3, 0:3] * T1_2[0:3, 0:3] * T2_3[0:3, 0:3]
            R0_3 = R0_3.evalf(subs={q1:theta1, q2:theta2, q3:theta3})
#            R3_6 = R0_3.inv("LU") * ROT_EE
            R3_6 = R0_3.T * ROT_EE

            theta4 = atan2(R3_6[2,2], -R3_6[0,2])
            theta5 = atan2(sqrt(R3_6[0,2] * R3_6[0,2] + R3_6[2,2] * R3_6[2,2]), R3_6[1,2])
            theta6 = atan2(-R3_6[1,1], R3_6[1,0])

#            if sin(theta5)>0:
#                theta4 = atan2(-R3_6[2,2], R3_6[0,2])
#                theta6 = atan2(R3_6[1,1], -R3_6[1,0])
#            else:
#                theta4 = atan2(R3_6[2,2], -R3_6[0,2])
#                theta6 = atan2(-R3_6[1,1], R3_6[1,0])

            theta1 = theta1.evalf()
            theta2 = theta2.evalf()
            theta3 = theta3.evalf()
            theta4 = theta4.evalf()
            theta5 = theta5.evalf()
            theta6 = theta6.evalf()

            q1 = q1.evalf()
            q2 = q2.evalf()
            q3 = q3.evalf()
            q4 = q4.evalf()
            q5 = q5.evalf()
            q6 = q6.evalf()
            print("position:")
            print(px)
            print(py)
            print(pz)
            print("thetas:")
            print(theta1)
            print(theta2)
            print(theta3)
            print(theta4)
            print(theta5)
            print(theta6)
            FK = T0_EE.evalf(subs={q1: theta1, q2: theta2, q3: theta3, q4: theta4, q5: theta5, q6: theta6})

	    #
            ###
		
            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
	    joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
	    joint_trajectory_list.append(joint_trajectory_point)

	    print("Joint Trajectory List")
	    print(joint_trajectory_list)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
