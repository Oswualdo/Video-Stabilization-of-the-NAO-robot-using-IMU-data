# Video-Stabilization-of-the-NAO-robot-using-IMU-data
The implementation of a video stabilization system of the NAO robot is presented through the data of the IMU, this stabilized image is used to detect and tracked QR codes. Once the QR code is located, the NAO robot tracks and monitors it. The system was developed under the ROS platform implementation of modules in C++ and Python language. The system provides data for subsequent processes, which need to use video data for object recognition, task tracking, among others. Can get sequences of stable images and with the least amount of vibrations or sudden movements. One of the main benefits of this work is the visual tracking of objects through stable images during walking of the NAO robot, which introduces an erratic motion of the head camera, the effect that is mitigated with the digital visual gyrostabilized method presented in this work


## 1.- NAO-ROS configuration
In the following link you can find the official page for NaoQI and the robots of Aldebaran where they provide the necessary configurations to be able to use the NAO robot under RO. Therefore, it is necessary to follow the steps detailed there.

http://wiki.ros.org/nao

### Necessary dependencies
It is necessary to install the following:

by terminal:
```
            sudo apt-get install ros-.*-nao-robot
            sudo apt-get install ros-.*-nao-extras
```         
It is also necessary to include the following in the workspace:

naoqi_driver http://wiki.ros.org/naoqi_driver ,naoqi_bridge http://wiki.ros.org/naoqi_bridge , nao_description http://wiki.ros.org/nao_description


## Usage

having everything configured, we proceed to compile the codes using the command catkin_make

To start the robot bringup, simply run:

C++:
>$ roslaunch nao_bringup nao_full.launch nao_ip:=<robot_ip> roscore_ip:=<roscore_ip>

Alternatively you can make use of the python SDK, which has to be installed and correctly setup in your PYTHONPATH environment variable. For more information on that, please refer to nao.

Python:
>$ roslaunch nao_bringup nao_full_py.launch nao_ip:=<robot_ip> roscore_ip:=<roscore_ip>

to execute the stabilizing video, it is necessary to launch the following:
>$ rosrun image_viewer imageviewer

Finally, to execute the control of the nao for the follow-up of the QR code, the following is launched (the file is inside the python_control folder):
>$ python operar_nao.py


