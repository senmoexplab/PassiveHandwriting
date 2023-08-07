import json
import os
import sys
import threading
import time

import numpy as np
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import \
    BaseCyclicClient
from kortex_api.autogen.messages import Base_pb2, BaseCyclic_pb2, Common_pb2

# Maximum allowed waiting time during actions (in seconds)
TIMEOUT_DURATION = 200


# Create closure to set an event after an END or an ABORT
def check_for_end_or_abort(e):
    """Return a closure checking for END or ABORT notifications

    Arguments:
    e -- event to signal when the action is completed
        (will be set when an END or ABORT occurs)
    """

    def check(notification, e=e):
        #print("EVENT : " + Base_pb2.ActionEvent.Name(notification.action_event))
        if (
            notification.action_event == Base_pb2.ACTION_END
            or notification.action_event == Base_pb2.ACTION_ABORT
        ):
            e.set()

    return check


def example_move_to_home_position(base):
    # Make sure the arm is in Single Level Servoing mode
    base_servo_mode = Base_pb2.ServoingModeInformation()
    base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
    base.SetServoingMode(base_servo_mode)

    # Move arm to ready position
    print("Moving the arm to a safe position")
    action_type = Base_pb2.RequestedActionType()
    action_type.action_type = Base_pb2.REACH_JOINT_ANGLES
    action_list = base.ReadAllActions(action_type)
    action_handle = None
    for action in action_list.action_list:
        if action.name == "Home":
            action_handle = action.handle

    if action_handle == None:
        print("Can't reach safe position. Exiting")
        return False

    e = threading.Event()
    notification_handle = base.OnNotificationActionTopic(
        check_for_end_or_abort(e), Base_pb2.NotificationOptions()
    )

    base.ExecuteActionFromReference(action_handle)
    finished = e.wait(TIMEOUT_DURATION)
    base.Unsubscribe(notification_handle)

    if finished:
        print("Safe position reached")
    else:
        print("Timeout on action notification wait")
    return finished


def populateAngularPose(jointPose, duration):
    waypoint = Base_pb2.AngularWaypoint()
    waypoint.angles.extend(jointPose)
    waypoint.duration = duration
    return waypoint

def prepare_trajectory(base, base_cyclic, Nu):
    base_servo_mode = Base_pb2.ServoingModeInformation()
    base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
    base.SetServoingMode(base_servo_mode)

    waypoints = Base_pb2.WaypointList()
    waypoints.duration = 0.0
    waypoints.use_optimal_blending = False
            
    waypoint = waypoints.waypoints.add()
    waypoint.name = "waypoint_" + str(0)

    waypoint.angular_waypoint.CopyFrom(
            populateAngularPose( (Nu[0],Nu[1],Nu[2],Nu[3],Nu[4],Nu[5]),
                                  duration = 3
                                )
        )

    # Verify validity of waypoints
    result = base.ValidateWaypointList(waypoints)
    if len(result.trajectory_error_report.trajectory_error_elements) == 0:
        e = threading.Event()
        notification_handle = base.OnNotificationActionTopic(
            check_for_end_or_abort(e), Base_pb2.NotificationOptions()
        )

        base.ExecuteWaypointTrajectory(waypoints)
        finished = e.wait(TIMEOUT_DURATION)
        base.Unsubscribe(notification_handle)

        return finished
    else:
        print("Error found in trajectory")
        print(result.trajectory_error_report)
        return finished


def Start_trajectory(base, base_cyclic, Nu, D):
    base_servo_mode = Base_pb2.ServoingModeInformation()
    base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
    base.SetServoingMode(base_servo_mode)

    sample = 4   

    seq_len = len(Nu[0][::sample]) 

    waypoints = Base_pb2.WaypointList()
    waypoints.duration = 0.0
    waypoints.use_optimal_blending = False

    for i in range(seq_len):
        
        waypoint = waypoints.waypoints.add()
        waypoint.name = "waypoint_" + str(i)

        waypoint.angular_waypoint.CopyFrom(
            populateAngularPose((Nu[0][i*sample], Nu[1][i*sample],  Nu[2][i*sample], Nu[3][i*sample], Nu[4][i*sample],  Nu[5][i*sample]) ,
                                 ( (D[1+i*sample])*sample))
        )
    
    SkipValidation = input('Skip the validation Step? (y/n): ').lower().strip() == 'y'

    #Nu_data = { 'Nu1': Nu[0], 'Nu2': Nu[1], 'Nu3': Nu[2], 'Nu4': Nu[3], 'Nu5': Nu[4], 'Nu6': Nu[5], 'Duration' : D}
    #with open('1-Workspace\examples\PassiveHandwritting\Joint_data.json', 'w') as fp:
    #    json.dump(Nu_data, fp)

    if( not SkipValidation):
        print("Verifying validity of {} waypoints".format(seq_len))
        # Verify validity of waypoints
        result = base.ValidateWaypointList(waypoints)

    if SkipValidation or len(result.trajectory_error_report.trajectory_error_elements) == 0:
        e = threading.Event()
        notification_handle = base.OnNotificationActionTopic(
            check_for_end_or_abort(e), Base_pb2.NotificationOptions()
        )

        base.ExecuteWaypointTrajectory(waypoints)

        finished = e.wait(TIMEOUT_DURATION)
        base.Unsubscribe(notification_handle)

        return finished
    else:
        print("Error found in trajectory")
        print(result.trajectory_error_report)
        return finished

def parseforFile(s):
    match(s):
        case 'c':
            return 'Joint_Circle'
        case 't':
            return'Joint_Triangle'
        case 'r':
            return 'Joint_Rectangle'
        case 's':
            return 'Joint_Square'
        case 'a':
            return 'Joint_Asterisk'
        case 'z':
            return 'Joint_ZigZag'
        case 'h':
            return 'Joint_Horizontal'
        case 'v':
            return 'Joint_Vertical'
        case _:
            return 'Joint_Circle'

def main():
    # Import the utilities helper module
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
    import utilities

    # Parse arguments
    args = utilities.parseConnectionArguments()
    shapeInput = input('Select Shape to draw ( C T R S A Z H V ): ').lower().strip()
                
    print('Opening Trajectory of Joint Targets :  ')
    #Joint_data
    with open('1-Workspace\PassiveHandwriting\WritingSamples\Joint-Shapes-200Hz\\' + parseforFile(shapeInput) + '.json','r') as fp:
        Nu = json.load(fp)
    # Create connection to the device and get the router
    with utilities.DeviceConnection.createTcpConnection(args) as router:
        # Create required services
        base = BaseClient(router)
        base_cyclic = BaseCyclicClient(router)

        success = True
        #success &= example_move_to_home_position(base)
        success &= prepare_trajectory(base, base_cyclic, [Nu['Nu1'][0], Nu['Nu2'][0],  Nu['Nu3'][0], Nu['Nu4'][0], Nu['Nu5'][0],  Nu['Nu6'][0]])
        success &= Start_trajectory(base, base_cyclic, [Nu['Nu1'][1:], Nu['Nu2'][1:],  Nu['Nu3'][1:], Nu['Nu4'][1:], Nu['Nu5'][1:],  Nu['Nu6'][1:]], Nu['Duration'][:])

        return 0 if success else 1

if __name__ == "__main__":
    exit(main())
