# oyster_controller

This is the main ROS package that interfaces with the other ROS nodes for the Oystamaran autonomous system. If looking for the overarching overview of the entire system, please see [master.md](master.md) oyster_controller publishes and maintains the state of the FSM (Finite State Machine) for the other nodes to use ([bag_detection](https://github.com/mattrix27/bag_detection) for perception, imu and gps for localization), and communicates with [ros_moos_bridge](https://github.com/mattrix27/ros_moos_bridge/) to send drive command to the MOOS as well as receives info from the RC controller. This was made with the intention of receiving and updating important info from the other nodes that are important to the other parts of the system.

## Quickstart

To run, open a terminal and run:
`roslaunch oyster_controller oyster_controller.launch`

On the current setup (Jetson TX2), an alias has been made to run above command with:
`oyster_controller`

## Configuring

As a ROS node, the main variables for configuring are set in the [params.yaml](params.yaml) file. This is currently the ROS topics to publish and subscribe to as well as other variables. The launch file can also be edited to run multiple nodes at the same time with this command

## Nodes

[boat_controller.py](scripts/boat_controller.py): This is currently the main (and only) node of this package. It has a class variable for the FSM Mode that it updates and publishes under certain criteria. 


0: IDLE, This is the idle state for when mission is started or we wish t orely on pure RC control, data will (or at least should) still be received and output, but no drive or flip commands should ever be issued unless set otherwise.

1: FRONT, Will listen to and process data received from the front facing camera to get relative error to target bag. `path_pos_callback` should receive a form of this data to then send to [ros_moos_bridge](https://github.com/mattrix27/ros_moos_bridge/). It should, find the corner of the array (left or right defined at beginning of launch) in the beginning, or determine the target bag if not. If transitioning from ROW CHECK, check if there are any bags in front, if not, go to TURN.

2: ALIGN, When the boat has entered the array (or the boat navigates to where it believes it is based off proposed localization info), then it will stop listening to the front-facing camera and start listening the bottom-facing camera to then find a bag. `flip_pos_callback` will receive this data from the camera as a relative error. If the error is 0 for long enough (based off counter parameter), it will assume bag is aligned and ready to flip, publishing to Arduino to flip (changing the state to 3)

3: FLIP, Publishes a message that gets sent to the Arduino, which runs the flipping process, then sends a ROS message back, which oyster_controller subscribes to then register and switch to mode 4.

4: FLIPPED, Once a bag gets flipped, we need to then move forward to the potential next bag, or exit the row. 
**TODO: give an arbitrary error/command to move forward a certain amount to move past already flipped bag. Once this is implemented, move to mode ROW CHECK.**

5: ROW CHECK, Listen to data from the bottom-facing camera to see if there is a possible bag. If so, go back to ALIGN, if not, assume that we hit a break in the row or exited the row, switch back to FRONT. 

6: TURN, When there are no bags in front, the boat must do a turn manuever to turn around and see array again. Should then use localization data (TODO) to determine where it should re-enter the array. 

7: FINISHED, when we recognize that we enter and then finish the last row, go to a idle form.
