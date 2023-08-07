###
#
# Demetrios Orton-Hatzis
# 2023-08-07 
#
# Built Using exemplary code from  
# KINOVA (R) KORTEX (TM)
# 01-compute-kinematics.py
#
###

import sys
import os
import json
import numpy as np

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.messages import Base_pb2
from kortex_api.Exceptions.KServerException import KServerException


def load_points_and_pose(Path = 'data.json'):
    with open(Path,'r') as fp:
        XYZ = json.load(fp)
    return XYZ['X'], XYZ['Y'], XYZ['Z'], XYZ['Duration']

def inv_kinematics(base, x, y, z, D, theta_x=170, theta_y=10, theta_z = 90):
    # get robot's pose (by using forward kinematics)
    try:
        input_joint_angles = base.GetMeasuredJointAngles()
        #pose = base.ComputeForwardKinematics(input_joint_angles)
    except KServerException as ex:
        print("Unable to get current robot pose")
        print("Error_code:{} , Sub_error_code:{} ".format(ex.get_error_code(), ex.get_error_sub_code()))
        print("Caught expected error: {}".format(ex))
        return False

    # Object containing cartesian coordinates and Angle Guess
    Trajectory_angles = np.zeros((6,len(x)))
    input_IkData = Base_pb2.IKData()

    for i in range(len(x)):
        # Fill the IKData Object with the cartesian coordinates that need to be converted
        input_IkData.cartesian_pose.x = x[i]
        input_IkData.cartesian_pose.y = y[i]
        input_IkData.cartesian_pose.z = z[i]
        input_IkData.cartesian_pose.theta_x = theta_x
        input_IkData.cartesian_pose.theta_y = theta_y
        input_IkData.cartesian_pose.theta_z = theta_z

        if(i==0): 
             # Fill the IKData Object with the guessed joint angles
            for joint_angle in [0,15,229,0,54,90] :
                jAngle = input_IkData.guess.joint_angles.add()
                # '- 1' to generate an actual "guess" for current joint angles
                jAngle.value = joint_angle
        else:
            # Fill the IKData Object with the guessed joint angles
            limits = [0, 128, 147, 0, 120, 0] 
            count=0
            for jAngle in input_IkData.guess.joint_angles:
                jAngle.value = Trajectory_angles[count,i-1]
                if (jAngle.value - limits[count] < 0 &  limits[count]!=0): 
                    jAngle.value = limits[count] -5
                elif (jAngle.value + limits[count] < 0 &  limits[count]!=0): 
                    jAngle.value = -limits[count] +5
                #print(Trajectory_angles[count,i-1])
                count +=1
        
        try:
            #print("Computing Inverse Kinematics using joint angles and pose...")
            computed_joint_angles = base.ComputeInverseKinematics(input_IkData)
            count=0
            for joint_angle in computed_joint_angles.joint_angles :
                Trajectory_angles[count,i] = joint_angle.value
                count +=1
           
        except KServerException as ex:
            print("Unable to compute inverse kinematics")
            print("Error_code:{} , Sub_error_code:{} ".format(ex.get_error_code(), ex.get_error_sub_code()))
            print("Caught expected error: {}".format(ex))
            print("Loop{}: {}".format(i,input_IkData))
            return False
        

    print("Saving inverse kinematics.")

    Nu_data = { 'Nu1': Trajectory_angles[0,:].tolist(),
                'Nu2': Trajectory_angles[1,:].tolist(),
                'Nu3': Trajectory_angles[2,:].tolist(),
                'Nu4': Trajectory_angles[3,:].tolist(),
                'Nu5': Trajectory_angles[4,:].tolist(),
                'Nu6': Trajectory_angles[5,:].tolist(),
                'Duration' : D}


    with open('1-Workspace\examples\PassiveHandwritting\Joint_data.json', 'w') as fp:
        json.dump(Nu_data, fp)

    return True

def main():
    # Import the utilities helper module
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
    import utilities

    # Parse arguments
    args = utilities.parseConnectionArguments()
    
    # Create connection to the device and get the router
    with utilities.DeviceConnection.createTcpConnection(args) as router:

        # Create required services
        base = BaseClient(router)

        X,Y,Z,D= load_points_and_pose('1-Workspace\examples\PassiveHandwritting\data.json')

        # Example core
        success = True
        success &= inv_kinematics(base, X, Y, Z, D)
        
        return 0 if success else 1

if __name__ == "__main__":
    exit(main())