# CAT - Vehicle Simulator
This repository UA-CATVehicle-dev aka Cognitive and Autonomous Test Vehicle is the repository from University Of Arizona Autonomous Driving Team led by Prof. Jonathon Sprinkle of EECS Department. Here, I have changed the number of sensors and written additional plugins for sensors, written extra nodes for other work and modularized the code for my understanding in this repository. For original repository please visit [this github repo](https://github.com/jmscslgroup/catvehicle).
Thank You !!!

## Getting Started

### System Requirements

- Ubuntu 18.04 LTS 
- RAM: 4GB required, > 8GB recommended.
  

### Installing ROS and Setting up Catkin Workspace

* Install ROS Melodic & Gazebo from instructions in this [website](http://wiki.ros.org/melodic/Installation/Ubuntu).
* Follow these commands for setting up catkin workspace
  ```sh
  cd ~
  mkdir -p catkin_ws/src
  cd catkin_ws
  catkin_make
  ```



### Dependencies

##### Installing CAT Vehicle dependent external ROS packages

Clone these packages into `~/catkin_ws/src` directory.

- Go to `~/catkin_ws/src` directory
    ```sh
    cd ~/catkin_ws/src/
    ```
- control_toolbox
    ```sh
    git clone https://github.com/jmscslgroup/sicktoolbox_wrapper
    ```
- sicktoolbox
    ```sh
    git clone https://github.com/jmscslgroup/control_toolbox
    ```
- sicktoolbox_wrapper
    ```sh
    git clone https://github.com/jmscslgroup/sicktoolbox
    ```
- cmdvel2gazebo
    ```sh
    git clone https://github.com/jmscslgroup/cmdvel2gazebo
    ```

##### Installing CAT Vehicle dependent ROS melodic packages

Install these ROS melodic packages from terminal.

- velocity-controllers
    ```sh
    sudo apt-get install ros-melodic-velocity-controllers
    ```
- velodyne-pointcloud
    ```sh
    sudo apt-get install ros-melodic-velodyne-pointcloud
    ```


### Setting up the Simulator

- Clone the CAT Vehicle package.
    ```sh
    cd ~/catkin_ws/src/
    git clone https://github.com/saipothanjanjanam/asu-catvehicle-dev.git
    ```
- Build the CAT Vehicle package and source it.
    ```sh
    cd ~/catkin_make
    catkin_make
    source ~/catkin_ws/devel/setup.bash
    ```


### Running CAT Vehicle Simulation

Follow these commands to run the CAT Vehicle simulation
- Sourcing the Catkin Workspace
    ```sh
    source ~/catkin_ws/devel/setup.bash
    ```
- Running this command loads the vehicle model, it's parameters, it's controllers, it's sensors and world parameters in Gazebo.  Here, Gazebo runs in headless mode
    ```sh
    roslaunch catvehicle catvehicle_skidspan.world
    ```
- Running this command opens the Gazebo simulator i.e Visual model of Gazebo.
    ```sh
    gzclient
    ```
- Publishing Velocity and Steering Angle to the CAT Vehicle
  ```sh
  rostopic pub /catvehicle/cmd_vel_safe geometry_msgs/Twist "linear:
      x: 0.5
      y: 0.0
      z: 0.0
    angular:
      x: 0.0
      y: 0.0
      z: 0.1"
  ```
  This command publishes linear velocity of 0.5 m/s   and steering angle of 0.1. Here, $-1.0 \leq$ z (steering angle) $\leq 1.0$. 
- If all commands went well, then you should see the vehicle is moving as shown below. 


### Writing your own code for controlling CAT Vehicle functionalities
We can also write our own `code` i.e `node` using any ROS client libraries like rospy, roscp, rosjava, roslisp, etc, . . .

We can access the vehicle's state information, control, sensor data, etc,...  which can be use to implement our algorithms on CAT Vehicle.
Follow these instructions below to write your own `code` i.e `node`, building it and executing it. Here, I am controlling velocity and steering angle of the CAT Vehicle using python script. Likewise you can write in C++ also. 

- Go to `src` directory of the `CAT Vehicle` package.
    ```sh
    cd ~/catkin_ws/src/asu-catvehicle-dev/src/
    ```
- Create the code file here, in this case `speed_steering_control.py`.
    ```sh
    touch speed_steering_control.py
    ```
- Copy this code to `speed_steering_control.py` file.
    ```python
    #!/usr/bin/env python
    import rospy
    from geometry_msgs.msg import Twist

    def main():
        velPub = rospy.Publisher('/catvehicle/cmd_vel_safe', Twist, queue_size=10)
        rospy.init_node('vel_steer_pub', anonymous=True)
        rate = rospy.Rate(10)
        i = 0
        while not rospy.is_shutdown():
            velMsg = Twist()
            velMsg.linear.x  = i
            velMsg.angular.z = 0.5
            velPub.publish(velMsg)
            rate.sleep()
            i+=0.01


    if __name__=="__main__":
        try:
            main()
        except rospy.ROSInterruptException:
            pass
    ```
- Give permissions to the file.
    ```sh
    chmod 777 speed_steering_control.py
    ```
- Go to `catkin_ws` directory.
    ```sh
    cd ~/catkin_ws/
    ```
- Build the CAT Vehicle package.
    ```
    catkin_make
    ```
- Source the package.
    ```sh
    source ~/catkin_ws/devel/setup.bash
    ```
- Run the script by this command.
    ```sh
    rosrun catvehicle speed_steering_control.py
    ```
- You should see the CAT Vehicle is moving with increasing speed in a circle as steering angle is constant i.e.., 0.5 as below.



## Citing this work
If you find this work useful please give credits to the authors and developers by citing:
```json
Rahul Bhadani, Jonathan Sprinkle, Matthew Bunting. "The CAT Vehicle Testbed: 
A Simulator with Hardware in the Loop for Autonomous Vehicle Applications". 
Proceedings 2nd International Workshop on Safe Control of Autonomous Vehicles (SCAV 2018),
Porto, Portugal, 10th April 2018, Electronic Proceedings in Theoretical Computer Science 269,
pp. 32–47.  Download:  http://dx.doi.org/10.4204/EPTCS.269.4.
```

bibtex:
```
@article{bhadani2018cat,
  title={{The CAT Vehicle Testbed: A Simulator with Hardware 
  in the Loop for Autonomous Vehicle Applications}},
  author={Bhadani, Rahul and Sprinkle, Jonathan and Bunting, Matthew},
  journal={{Proceedings of 2nd International Workshop on Safe Control of Autonomous Vehicles
  (SCAV 2018), Porto, Portugal, 10th April 2018, Electronic Proceedings
  in Theoretical Computer Science 269, pp. 32–47}},
year={2018}
}
```

## Issues
If you run into a problem, please feel free to post to [issues](https://github.com/jmscslgroup/catvehicle/issues). If the issue is urgent, please email to catvehicle@list.arizona.edu.

## Acknowledgements
### License
Copyright (c) 2013-2020 Arizona Board of Regents; The University of Arizona
All rights reserved

Permission is hereby granted, without written agreement and without 
license or royalty fees, to use, copy, modify, and distribute this
software and its documentation for any purpose, provided that the 
above copyright notice and the following two paragraphs appear in 
all copies of this software.
 
IN NO EVENT SHALL THE ARIZONA BOARD OF REGENTS BE LIABLE TO ANY PARTY 
FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES 
ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN 
IF THE ARIZONA BOARD OF REGENTS HAS BEEN ADVISED OF THE POSSIBILITY OF 
SUCH DAMAGE.

THE ARIZONA BOARD OF REGENTS SPECIFICALLY DISCLAIMS ANY WARRANTIES, 
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY 
AND FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER
IS ON AN "AS IS" BASIS, AND THE ARIZONA BOARD OF REGENTS HAS NO OBLIGATION
TO PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.

## Authors and contributors
* Jonathan Sprinkle (sprinkjm@email.arizona.edu)
* Rahul Bhadani (rahulbhadani@email.arizona.edu)
* Sam Taylor
* Kennon McKeever (kennondmckeever@email.arizona.edu)
* Alex Warren
* Swati Munjal (smunjal@email.arizona.edu)
* Ashley Kang (askang@email.arizona.edu)
* Matt Bunting (mosfet@email.arizona.edu)
* Sean Whitsitt

## Support
This work was supported by the National Science Foundation and AFOSR under awards 1659428, 1521617, 1446435, 1262960 and 1253334. Any opinions, findings, and conclusions or recommendations expressed in this material are those of the author(s) and do not necessarily reflect the views of the National Science Foundation.
